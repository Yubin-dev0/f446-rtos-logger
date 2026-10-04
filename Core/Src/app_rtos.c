/* FreeRTOS Day 2: LED / Heartbeat(UART) / Sensor(VL53L0X) -> Queue -> Consumer(UART)
 * API: CMSIS-RTOS v2 only (native FreeRTOS API 섞지 않음)
 *
 * Day 2 변경점
 *  - Producer(모의 데이터) -> SensorTask(VL53L0X 실측, I2C1 PB8/PB9)
 *  - 측정 방식: single-shot. 200ms 주기마다 1회 측정 -> 샘플 시각이 주기에 정렬됨
 *  - 센서 초기화는 태스크 안에서 수행 (ST API가 osDelay 기반 polling을 쓰므로 스케줄러 필요)
 *  - ST VL53L0X_ResetDevice()는 타임아웃 없는 while 루프가 있어서 사용하지 않음
 *    -> 재시도 횟수 제한 soft reset(vl_soft_reset) 직접 구현
 *  - PA6(MK_SENS)/PA7(MK_CONS): 로직 분석기용 태스크 실행 구간 마커 (CubeMX 라벨 있을 때만)
 *
 * Day 2 / 목표 4: 큐 정책 스위치 (QUEUE_POLICY, QUEUE_LEN, CONSUMER_DELAY_MS), 소비 시 age 출력
 *
 * Day 2 / 3단계: I2C 에러 처리
 *  - 센서 상태 머신: INIT -> RUN -> ERROR(backoff) -> INIT ...
 *  - RUN 중 연속 VL_ERR_THRESHOLD회 실패 -> ERROR (1회성 glitch는 흡수)
 *  - ERROR: I2C 버스 복구(SCL 9클럭 + STOP + I2C1 peripheral reset) 후 센서 재초기화
 *           재시도 간격 100 -> 200 -> ... -> 2000ms (지수 backoff)
 *  - osDelayUntil catch-up burst 방지: 주기를 놓쳤으면 밀린 슬롯을 건너뛰고 missed++
 *
 * 3단계 수정(T1~T4 결과 반영)
 *  - ST PerformSingleRangingMeasurement는 완료 대기를 2000회 x PollingDelay(2ms) = 약 4초까지 함.
 *    센서가 전원 글리치로 조용히 리셋되면 I2C는 ACK하지만 data-ready가 영원히 안 떠서
 *    측정 1회당 4초 블록 -> 3회 연속 판정까지 약 12초 동안 RUN으로 보이면서 데이터 없음.
 *    -> vl_measure(): 완료 대기를 VL_MEAS_TIMEOUT_MS로 직접 제한.
 *  - downtime을 '첫 실패 시점'부터 계산 (이전엔 3번째 실패 시점 기준이라 과소 측정)
 */
#include "app_rtos.h"
#include "main.h"
#include "vl53l0x_api.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

/* ===================== 실험 스위치 ===================== */
#define STACK_REPORT        1   /* 5초마다 태스크별 최소 스택 여유 출력 */
#define TASK_MARKERS        1   /* PA6/PA7 마커 토글 (로직 분석기)     */
#define FAULT_INJECT_EVERY  0   /* 0=끔. N이면 seq가 N의 배수일 때마다 센서에 I2C soft reset을 몰래 보냄
                                 * -> 전원 글리치로 센서만 리셋된 상황을 반복 재현 (I2C는 계속 ACK)
                                 *    200ms x 50 = 약 10초마다 1회 */

#define LED_PERIOD_MS       500
#define HB_PERIOD_MS        1000
#define SENSOR_PERIOD_MS    200
#define QUEUE_LEN           8

/* 큐가 가득 찼을 때의 정책 (목표 4 비교 실험)
 *  QP_DROP_NEWEST : 새 샘플을 버림 (1일차 방식). 큐에 오래된 값이 고여 age가 커짐
 *  QP_DROP_OLDEST : 가장 오래된 샘플을 꺼내 버리고 새 샘플을 넣음 (최신 값 우선)
 *  QUEUE_LEN=1 + QP_DROP_OLDEST = 메일박스 (항상 가장 최근 1개만 유지) */
#define QP_DROP_NEWEST      0
#define QP_DROP_OLDEST      1
#define QUEUE_POLICY        QP_DROP_OLDEST

#define CONSUMER_DELAY_MS   0     /* 스트레스 실험: 1000 -> 소비 1/s < 생산 5/s */

#define VL_I2C_ADDR         0x52      /* 8-bit (7-bit 0x29) */
#define VL_TIMING_BUDGET_US 33000U    /* ST 기본값. 200ms 주기 안에 여유 충분 */
#define VL_MODEL_ID         0xEEU     /* reg 0xC0 고정값 = 센서 존재 확인용 */
#define VL_RESET_POLL_MAX   50U       /* soft reset polling 최대 횟수 (x2ms) */

#define VL_ERR_THRESHOLD    3U        /* RUN -> ERROR 전환 연속 실패 횟수 */
#define VL_MEAS_TIMEOUT_MS  100U      /* 측정 완료 대기 상한 (버짓 33ms의 약 3배) */
/* 측정 시작 후 첫 data-ready 폴링까지 쉬는 시간 (0 = 기존: 바로 2ms 폴링)
 * 버짓 33ms 전에는 결과가 나올 수 없으므로 그 전의 폴링은 버스만 점유한다.
 * 버짓보다 작게 잡아야 함 (osDelay(n)은 최대 1틱 짧게 깨어날 수 있음 -> 여유 3ms) */
#define VL_FIRST_POLL_MS    30U
#if VL_FIRST_POLL_MS >= (VL_TIMING_BUDGET_US / 1000U)
  #error "VL_FIRST_POLL_MS must be shorter than the timing budget"
#endif
#define VL_BACKOFF_MIN_MS   100U
#define VL_BACKOFF_MAX_MS   2000U

/* I2C1 핀 (버스 복구 시 GPIO로 직접 제어) */
#define I2C_SCL_PORT        GPIOB
#define I2C_SCL_PIN         GPIO_PIN_8
#define I2C_SDA_PORT        GPIOB
#define I2C_SDA_PIN         GPIO_PIN_9

/* ===================== 보드 매핑 ===================== */
#if defined(LD2_Pin)
  #define APP_LED_PORT   LD2_GPIO_Port
  #define APP_LED_PIN    LD2_Pin
#else
  #define APP_LED_PORT   GPIOA          /* NUCLEO-F446RE LD2 = PA5 고정 */
  #define APP_LED_PIN    GPIO_PIN_5
#endif
#define APP_LED_TOGGLE()   HAL_GPIO_TogglePin(APP_LED_PORT, APP_LED_PIN)

/* 마커 핀은 CubeMX 라벨에 의존하지 않고 코드에서 직접 설정 (APP_MARKER_INIT)
 * PA6 = Arduino D12 (Sensor), PA7 = Arduino D11 (Consumer) */
#define MK_PORT       GPIOA
#define MK_SENS_PIN   GPIO_PIN_6
#define MK_CONS_PIN   GPIO_PIN_7

#if TASK_MARKERS
  #define MK_SENS_HI()  (MK_PORT->BSRR = MK_SENS_PIN)
  #define MK_SENS_LO()  (MK_PORT->BSRR = (uint32_t)MK_SENS_PIN << 16)
  #define MK_CONS_HI()  (MK_PORT->BSRR = MK_CONS_PIN)
  #define MK_CONS_LO()  (MK_PORT->BSRR = (uint32_t)MK_CONS_PIN << 16)
#else
  #define MK_SENS_HI()  ((void)0)
  #define MK_SENS_LO()  ((void)0)
  #define MK_CONS_HI()  ((void)0)
  #define MK_CONS_LO()  ((void)0)
#endif

static void APP_LED_INIT(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  g.Pin   = APP_LED_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(APP_LED_PORT, &g);
}

static void APP_MARKER_INIT(void)
{
#if TASK_MARKERS
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  MK_PORT->BSRR = ((uint32_t)(MK_SENS_PIN | MK_CONS_PIN)) << 16;   /* LOW로 시작 */
  g.Pin   = MK_SENS_PIN | MK_CONS_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MK_PORT, &g);
#endif
}

/* huart2가 main.c에 없으면(=BSP 모드) 링크 시 NULL이 되도록 weak 선언 */
extern UART_HandleTypeDef huart2 __attribute__((weak));

static UART_HandleTypeDef *app_uart(void)
{
  if (&huart2 != NULL) return &huart2;
#if defined(USE_BSP_COM_FEATURE) && (USE_BSP_COM_FEATURE > 0U)
  return &hcom_uart[COM1];
#else
  return NULL;
#endif
}

static void uart_write(const char *s, uint16_t len)
{
  UART_HandleTypeDef *h = app_uart();
  if (h != NULL) {
    HAL_UART_Transmit(h, (uint8_t *)s, len, 100);
  }
}

/* ===================== 데이터 타입 ===================== */
typedef struct {
  uint32_t seq;          /* 연속 번호: 빠진 번호 = 드롭된 샘플        */
  uint32_t tick;         /* 측정 시작 시각 (ms)                       */
  uint16_t dist_mm;      /* RangeMilliMeter                          */
  uint16_t signal_kcps;  /* SignalRateRtnMegaCps(16.16) -> kcps 정수 */
  uint8_t  range_status; /* 0 = valid, 그 외 = ST RangeStatus 코드    */
  uint8_t  meas_ms;      /* 측정 1회 소요 시간 (ms)                   */
} SensorSample_t;

/* ===================== 핸들 / 상태 ===================== */
static osThreadId_t       ledTask, hbTask, sensTask, consTask;
static osMessageQueueId_t sensorQueue;
static osMutexId_t        uartMutex;
static volatile uint32_t  dropCount;
static volatile uint32_t  i2cErrCount;  /* 실패한 측정/초기화 시도 누적      */
static volatile int32_t   lastVlErr;    /* 마지막 VL53L0X_Error 코드          */
static volatile uint32_t  recoverCount; /* ERROR -> RUN 복구 성공 횟수         */
static volatile uint32_t  missedSlots;  /* osDelayUntil 주기 놓쳐서 건너뛴 수  */

typedef enum { VL_ST_INIT = 0, VL_ST_RUN, VL_ST_ERROR } VlState_t;
static volatile VlState_t vlState = VL_ST_INIT;
static const char *const vlStateName[] = { "INIT", "RUN", "ERROR" };

static VL53L0X_Dev_t vlDev;            /* 수백 byte라 스택 대신 static */

/* 우선순위: Consumer > Sensor > Heartbeat > LED
 * stack_size 단위는 byte. ST API 호출하는 Sensor는 넉넉하게. */
static const osThreadAttr_t ledAttr  = { .name = "LED",  .stack_size = 128 * 4, .priority = osPriorityLow };
static const osThreadAttr_t hbAttr   = { .name = "HB",   .stack_size = 512 * 4, .priority = osPriorityBelowNormal };
static const osThreadAttr_t sensAttr = { .name = "Sens", .stack_size = 512 * 4, .priority = osPriorityNormal };
static const osThreadAttr_t consAttr = { .name = "Cons", .stack_size = 512 * 4, .priority = osPriorityAboveNormal };
static const osMessageQueueAttr_t queueAttr = { .name = "sensorQ" };
static const osMutexAttr_t uartMutexAttr = { .name = "uart", .attr_bits = osMutexPrioInherit };

/* ===================== UART 헬퍼 (태스크 전용) ===================== */
static void uart_printf(const char *fmt, ...)
{
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n >= (int)sizeof buf) n = (int)sizeof buf - 1;

  if (osMutexAcquire(uartMutex, osWaitForever) == osOK) {
    uart_write(buf, (uint16_t)n);
    osMutexRelease(uartMutex);
  }
}

/* ===================== I2C 버스 복구 ===================== */
static void i2c_bb_delay(void)
{
  for (volatile uint32_t i = 0; i < 200U; i++) { }   /* 수 us. 느려도 I2C는 상관없음 */
}

/* 슬레이브가 SDA를 LOW로 잡고 멈춘 경우(전송 중 끊김 등) 풀어주는 표준 절차 +
 * STM32F4 I2C의 BUSY 플래그 고착(errata) 해제용 peripheral reset. */
static void i2c_bus_recover(void)
{
  GPIO_InitTypeDef g = {0};

  HAL_I2C_DeInit(&hi2c1);                 /* MspDeInit: 핀 AF 해제, 클럭 off */

  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Mode  = GPIO_MODE_OUTPUT_OD;
  g.Pull  = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = I2C_SCL_PIN;  HAL_GPIO_Init(I2C_SCL_PORT, &g);
  g.Pin   = I2C_SDA_PIN;  HAL_GPIO_Init(I2C_SDA_PORT, &g);

  HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);
  for (int i = 0; i < 9; i++) {           /* SCL 9클럭: 슬레이브가 남은 비트 밀어내게 */
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET); i2c_bb_delay();
    HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);   i2c_bb_delay();
    if (HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN) == GPIO_PIN_SET) break;
  }
  /* STOP: SCL HIGH 상태에서 SDA LOW -> HIGH */
  HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET); i2c_bb_delay();
  HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET);   i2c_bb_delay();
  HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET);   i2c_bb_delay();

  __HAL_RCC_I2C1_FORCE_RESET();           /* BUSY 고착 해제 */
  __HAL_RCC_I2C1_RELEASE_RESET();

  HAL_I2C_Init(&hi2c1);                   /* MspInit: 핀 AF 복귀, 클럭 on */
}

/* ===================== VL53L0X ===================== */
/* ST VL53L0X_ResetDevice() 대체: polling 횟수 제한 + model ID로 존재 확인.
 * XSHUT 핀이 없는 4핀 모듈이라 이게 유일한 센서 리셋 수단. */
static uint8_t vlRstPhase;   /* soft reset 실패 진단: 1=리셋 진입 대기, 2=부팅 완료 대기 */
static uint8_t vlRstLastId;  /* 실패 시점에 마지막으로 읽은 MODEL_ID                     */

static VL53L0X_Error vl_soft_reset(VL53L0X_DEV dev)
{
  VL53L0X_Error st;
  uint8_t id = 0xFF;
  uint32_t i;

  vlRstPhase = 1;

  st = VL53L0X_WrByte(dev, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x00);
  if (st != VL53L0X_ERROR_NONE) return st;

  for (i = 0; i < VL_RESET_POLL_MAX; i++) {          /* 리셋 진입: ID가 0으로 읽힘 */
    st = VL53L0X_RdByte(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &id);
    if (st == VL53L0X_ERROR_NONE && id == 0x00) break;
    osDelay(2);
  }
  vlRstLastId = id;
  if (i == VL_RESET_POLL_MAX) return VL53L0X_ERROR_TIME_OUT;

  vlRstPhase = 2;

  st = VL53L0X_WrByte(dev, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x01);
  if (st != VL53L0X_ERROR_NONE) return st;

  for (i = 0; i < VL_RESET_POLL_MAX; i++) {          /* 부팅 완료: ID = 0xEE */
    st = VL53L0X_RdByte(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &id);
    if (st == VL53L0X_ERROR_NONE && id == VL_MODEL_ID) return VL53L0X_ERROR_NONE;
    osDelay(2);
  }
  vlRstLastId = id;
  return VL53L0X_ERROR_TIME_OUT;
}

/* single-shot 1회. ST PerformSingleRangingMeasurement와 같은 순서지만
 *  (1) 결과가 나올 수 없는 구간은 폴링하지 않고 osDelay로 양보 (VL_FIRST_POLL_MS)
 *  (2) 완료 대기 상한을 VL_MEAS_TIMEOUT_MS로 제한 (ST 원본은 약 4초)
 * StartMeasurement 안의 start-bit 대기 루프는 delay 없는 I2C read라 수 ms 안에 끝남. */
static VL53L0X_Error vl_measure(VL53L0X_DEV dev, VL53L0X_RangingMeasurementData_t *m)
{
  VL53L0X_Error st;
  uint8_t ready = 0;
  uint32_t t0 = osKernelGetTickCount();

  if ((st = VL53L0X_StartMeasurement(dev)) != VL53L0X_ERROR_NONE) return st;

#if VL_FIRST_POLL_MS
  osDelay(VL_FIRST_POLL_MS);
#endif

  for (;;) {
    if ((st = VL53L0X_GetMeasurementDataReady(dev, &ready)) != VL53L0X_ERROR_NONE) return st;
    if (ready) break;
    if (osKernelGetTickCount() - t0 >= VL_MEAS_TIMEOUT_MS) return VL53L0X_ERROR_TIME_OUT;
    osDelay(2);
  }

  if ((st = VL53L0X_GetRangingMeasurementData(dev, m)) != VL53L0X_ERROR_NONE) return st;
  return VL53L0X_ClearInterruptMask(dev, 0);
}

/* 논문 펌웨어와 같은 순서, 단 ResetDevice 대신 vl_soft_reset, 모드는 single-shot */
static VL53L0X_Error vl_init(VL53L0X_DEV dev)
{
  VL53L0X_Error st;
  uint32_t refSpadCount = 0;
  uint8_t  isAperture = 0, vhv = 0, phase = 0;

  memset(dev, 0, sizeof *dev);
  dev->I2cHandle  = &hi2c1;
  dev->I2cDevAddr = VL_I2C_ADDR;

  st = vl_soft_reset(dev);
  if (st == VL53L0X_ERROR_TIME_OUT) {
    /* I2C는 살아 있는데 리셋 상태 전이가 안 보이는 경우: 진단만 남기고 DataInit으로 진행.
     * DataInit/StaticInit이 필요한 레지스터를 다시 쓰므로 그걸로 복구되는지 확인한다. */
    uart_printf("[VL] soft reset timeout phase=%u id=0x%02X -> continue with DataInit\r\n",
                vlRstPhase, vlRstLastId);
  } else if (st != VL53L0X_ERROR_NONE) {
    return st;                               /* I2C 자체 실패 (-20): 센서 없음 */
  }
  if ((st = VL53L0X_DataInit(dev)) != VL53L0X_ERROR_NONE) return st;
  if ((st = VL53L0X_StaticInit(dev)) != VL53L0X_ERROR_NONE) return st;
  if ((st = VL53L0X_PerformRefCalibration(dev, &vhv, &phase)) != VL53L0X_ERROR_NONE) return st;
  if ((st = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isAperture)) != VL53L0X_ERROR_NONE) return st;
  if ((st = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING)) != VL53L0X_ERROR_NONE) return st;
  if ((st = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, VL_TIMING_BUDGET_US)) != VL53L0X_ERROR_NONE) return st;

  uart_printf("[VL] init OK spad=%lu aperture=%u vhv=%u phase=%u\r\n",
              (unsigned long)refSpadCount, isAperture, vhv, phase);
  return VL53L0X_ERROR_NONE;
}

/* ===================== 태스크 ===================== */
static void LedTask(void *arg)
{
  (void)arg;
  for (;;) {
    APP_LED_TOGGLE();
    osDelay(LED_PERIOD_MS);
  }
}

static void HeartbeatTask(void *arg)
{
  (void)arg;
  uint32_t n = 0;
  for (;;) {
    uint32_t qCount = sensorQueue ? osMessageQueueGetCount(sensorQueue) : 0;
    uart_printf("[HB] tick=%lu vl=%s q=%lu/%u drop=%lu miss=%lu err=%lu last=%ld rec=%lu\r\n",
                (unsigned long)osKernelGetTickCount(),
                vlStateName[vlState],
                (unsigned long)qCount, QUEUE_LEN,
                (unsigned long)dropCount, (unsigned long)missedSlots,
                (unsigned long)i2cErrCount, (long)lastVlErr,
                (unsigned long)recoverCount);

#if STACK_REPORT
    if ((++n % 5U) == 0U) {
      uart_printf("[STK] min free bytes: LED=%lu HB=%lu Sens=%lu Cons=%lu\r\n",
                  (unsigned long)osThreadGetStackSpace(ledTask),
                  (unsigned long)osThreadGetStackSpace(hbTask),
                  (unsigned long)osThreadGetStackSpace(sensTask),
                  (unsigned long)osThreadGetStackSpace(consTask));
    }
#else
    (void)n;
#endif
    osDelay(HB_PERIOD_MS);
  }
}

/* 큐 넣기. 정책에 따라 가득 찼을 때 새 값 또는 가장 오래된 값을 버린다.
 * DROP_OLDEST: Get(비움) -> Put 사이에 더 높은 우선순위인 Consumer가 끼어들어 하나를 가져가도
 * 그건 '소비'라 손실이 아니다. 두 번째 Put은 실패할 수 없지만 방어적으로 카운트한다. */
static void queue_push(const SensorSample_t *s)
{
  if (osMessageQueuePut(sensorQueue, s, 0U, 0U) == osOK) return;
#if QUEUE_POLICY == QP_DROP_OLDEST
  {
    SensorSample_t old;
    if (osMessageQueueGet(sensorQueue, &old, NULL, 0U) == osOK) dropCount++;
    if (osMessageQueuePut(sensorQueue, s, 0U, 0U) != osOK) dropCount++;
  }
#else
  dropCount++;
#endif
}

static void vl_set_state(VlState_t s)
{
  if (vlState != s) {
    uart_printf("[VL] state %s -> %s (tick=%lu)\r\n",
                vlStateName[vlState], vlStateName[s],
                (unsigned long)osKernelGetTickCount());
    vlState = s;
  }
}

static void SensorTask(void *arg)
{
  (void)arg;
  VL53L0X_Error st;
  uint32_t seq        = 0;
  uint32_t consecErr  = 0;
  uint32_t backoff    = VL_BACKOFF_MIN_MS;
  uint32_t attempts   = 0;      /* 이번 복구 구간에서 init 시도 횟수 */
  uint8_t  recovering = 0;      /* 1 = ERROR를 한 번이라도 거침       */
  uint32_t downSince  = osKernelGetTickCount();
  uint32_t next       = 0;

  for (;;) {
    switch (vlState) {

    case VL_ST_INIT:
      attempts++;
      st = vl_init(&vlDev);
      if (st == VL53L0X_ERROR_NONE) {
        if (recovering) {
          recoverCount++;
          uart_printf("[VL] recovered: attempts=%lu downtime=%lu ms\r\n",
                      (unsigned long)attempts,
                      (unsigned long)(osKernelGetTickCount() - downSince));
        }
        consecErr  = 0;
        attempts   = 0;
        recovering = 0;
        backoff    = VL_BACKOFF_MIN_MS;
        next      = osKernelGetTickCount();   /* 주기 기준점 재설정 -> 복구 후 burst 없음 */
        vl_set_state(VL_ST_RUN);
      } else {
        lastVlErr = st;
        i2cErrCount++;
        recovering = 1;
        uart_printf("[VL] init FAIL err=%d attempt=%lu, retry in %lu ms\r\n",
                    (int)st, (unsigned long)attempts, (unsigned long)backoff);
        vl_set_state(VL_ST_ERROR);
      }
      break;

    case VL_ST_RUN: {
      VL53L0X_RangingMeasurementData_t m;
      uint32_t t0 = osKernelGetTickCount();

      MK_SENS_HI();
      st = vl_measure(&vlDev, &m);
      MK_SENS_LO();

      if (st == VL53L0X_ERROR_NONE) {
        SensorSample_t s;
        uint32_t dt = osKernelGetTickCount() - t0;
        consecErr      = 0;
        s.seq          = seq++;
        s.tick         = t0;
        s.dist_mm      = m.RangeMilliMeter;
        s.signal_kcps  = (uint16_t)(((uint32_t)m.SignalRateRtnMegaCps * 1000U) >> 16);
        s.range_status = m.RangeStatus;
        s.meas_ms      = (uint8_t)(dt > 255U ? 255U : dt);

        queue_push(&s);
#if FAULT_INJECT_EVERY
        if (s.seq != 0U && (s.seq % FAULT_INJECT_EVERY) == 0U) {
          static uint32_t fiCount;
          uart_printf("[FI] #%lu inject sensor soft reset after seq=%lu\r\n",
                      (unsigned long)++fiCount, (unsigned long)s.seq);
          VL53L0X_WrByte(&vlDev, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x00);
          osDelay(2);
          VL53L0X_WrByte(&vlDev, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x01);
        }
#endif
      } else {
        lastVlErr = st;
        i2cErrCount++;
        if (++consecErr == 1U) downSince = t0;     /* 마지막 정상 샘플 이후 첫 실패 시점 */
        if (consecErr >= VL_ERR_THRESHOLD) {
          uart_printf("[VL] %lu consecutive errors (last=%d)\r\n",
                      (unsigned long)consecErr, (int)st);
          attempts   = 0;
          recovering = 1;
          backoff    = VL_BACKOFF_MIN_MS;
          vl_set_state(VL_ST_ERROR);
          break;
        }
      }

      /* 다음 주기. 이미 지났으면(측정이 주기보다 길었던 경우 등) 밀린 슬롯은 버린다.
       * 그냥 osDelayUntil 하면 지난 시각이라 즉시 리턴 -> 연속 측정 burst 발생. */
      next += SENSOR_PERIOD_MS;
      {
        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(next - now) <= 0) {
          uint32_t late = now - next;
          missedSlots += late / SENSOR_PERIOD_MS + 1U;
          next += (late / SENSOR_PERIOD_MS + 1U) * SENSOR_PERIOD_MS;
        }
      }
      osDelayUntil(next);
      break;
    }

    case VL_ST_ERROR:
    default:
      osDelay(backoff);
      backoff = (backoff * 2U > VL_BACKOFF_MAX_MS) ? VL_BACKOFF_MAX_MS : backoff * 2U;
      i2c_bus_recover();
      vl_set_state(VL_ST_INIT);
      break;
    }
  }
}

static void ConsumerTask(void *arg)
{
  (void)arg;
  SensorSample_t s;

  for (;;) {
    if (osMessageQueueGet(sensorQueue, &s, NULL, osWaitForever) == osOK) {
      /* age: 측정 시작(tick)부터 소비자가 꺼낸 시점까지. 로거가 보여주는 값이 얼마나 낡았는지 */
      uint32_t age = osKernelGetTickCount() - s.tick;
      MK_CONS_HI();
      uart_printf("[DATA] seq=%lu t=%lu dist=%u mm st=%u sig=%u kcps meas=%u ms age=%lu ms\r\n",
                  (unsigned long)s.seq, (unsigned long)s.tick,
                  (unsigned)s.dist_mm, (unsigned)s.range_status,
                  (unsigned)s.signal_kcps, (unsigned)s.meas_ms,
                  (unsigned long)age);
      MK_CONS_LO();
    }
#if CONSUMER_DELAY_MS
    osDelay(CONSUMER_DELAY_MS);
#endif
  }
}

/* ===================== 초기화 ===================== */
void App_RTOS_Init(void)
{
  APP_LED_INIT();
  APP_MARKER_INIT();

  /* 스케줄러 시작 전이라 mutex 없이 직접 송신 */
  static const char banner[] = "\r\n=== f446-rtos-logger day2 boot ===\r\n";
  uart_write(banner, sizeof banner - 1);

  /* 리셋 원인 기록: RCC->CSR 플래그는 다음 리셋까지 남아 있음.
   * F4는 내부 리셋 시 NRST도 LOW로 끌려서 PIN은 거의 항상 같이 뜬다.
   *  POR PIN BOR = 전원 저하(약 1.7V 미만), PIN만 = 버튼/ST-LINK/NRST 노이즈 */
  {
    char line[96];
    uint32_t csr = RCC->CSR;
    int n = snprintf(line, sizeof line, "[RST] csr=0x%08lX%s%s%s%s%s%s%s\r\n",
                     (unsigned long)csr,
                     (csr & RCC_CSR_LPWRRSTF) ? " LPWR" : "",
                     (csr & RCC_CSR_WWDGRSTF) ? " WWDG" : "",
                     (csr & RCC_CSR_IWDGRSTF) ? " IWDG" : "",
                     (csr & RCC_CSR_SFTRSTF)  ? " SW"   : "",
                     (csr & RCC_CSR_PORRSTF)  ? " POR"  : "",
                     (csr & RCC_CSR_PINRSTF)  ? " PIN"  : "",
                     (csr & RCC_CSR_BORRSTF)  ? " BOR"  : "");
    uart_write(line, (uint16_t)n);
    __HAL_RCC_CLEAR_RESET_FLAGS();   /* 다음 리셋 원인만 남도록 지움 */
  }

  /* 실제로 어떤 클럭으로 돌고 있는지 레지스터에서 직접 확인 */
  {
    char line[96];
    uint32_t sws = RCC->CFGR & RCC_CFGR_SWS;
    int n = snprintf(line, sizeof line, "[CLK] sysclk=%lu Hz sws=%s pll_in=%s hse_on=%d hse_byp=%d\r\n",
                     (unsigned long)HAL_RCC_GetSysClockFreq(),
                     (sws == RCC_CFGR_SWS_PLL) ? "PLL" : (sws == RCC_CFGR_SWS_HSE) ? "HSE" : "HSI",
                     (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) ? "HSE" : "HSI",
                     (RCC->CR & RCC_CR_HSERDY) ? 1 : 0,
                     (RCC->CR & RCC_CR_HSEBYP) ? 1 : 0);
    uart_write(line, (uint16_t)n);
  }

  {
    char line[80];
    int n = snprintf(line, sizeof line, "[CFG] queue=%s len=%u cons_delay=%u ms\r\n",
                     (QUEUE_POLICY == QP_DROP_OLDEST) ? "DROP_OLDEST" : "DROP_NEWEST",
                     (unsigned)QUEUE_LEN, (unsigned)CONSUMER_DELAY_MS);
    uart_write(line, (uint16_t)n);
  }

  /* I2C 버스 스캔: 센서 응답만 먼저 확인 (실제 초기화는 SensorTask에서) */
  {
    char line[64];
    HAL_StatusTypeDef hs = HAL_I2C_IsDeviceReady(&hi2c1, VL_I2C_ADDR, 3, 10);
    int n = snprintf(line, sizeof line, "[I2C] VL53L0X @0x29: %s\r\n",
                     hs == HAL_OK ? "ACK" : "NO RESPONSE");
    uart_write(line, (uint16_t)n);
  }

  uartMutex   = osMutexNew(&uartMutexAttr);
  sensorQueue = osMessageQueueNew(QUEUE_LEN, sizeof(SensorSample_t), &queueAttr);
  ledTask     = osThreadNew(LedTask, NULL, &ledAttr);
  hbTask      = osThreadNew(HeartbeatTask, NULL, &hbAttr);
  sensTask    = osThreadNew(SensorTask, NULL, &sensAttr);
  consTask    = osThreadNew(ConsumerTask, NULL, &consAttr);

  /* NULL이면 대부분 FreeRTOS heap 부족 -> TOTAL_HEAP_SIZE 확인 */
  if (uartMutex == NULL || sensorQueue == NULL ||
      ledTask == NULL || hbTask == NULL || sensTask == NULL || consTask == NULL) {
    Error_Handler();
  }
}
