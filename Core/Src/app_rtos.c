/* =============================================================================
 * f446-rtos-logger : app_rtos.c
 * NUCLEO-F446RE + VL53L0X(ToF, I2C1) 센서 로거. 애플리케이션 코드는 이 파일 하나가 전부다.
 * main.c 의 RTOS_THREADS 구간에서 App_RTOS_Init() 한 번 호출 -> 이후 스케줄러가 돌린다.
 *
 * RTOS API: CMSIS-RTOS v2 만 사용 (xTask..., xQueue... 같은 네이티브 FreeRTOS API 섞지 않음)
 *   왜: 같은 개념(대기, 큐, 뮤텍스)을 한 가지 이름으로만 다루려고. 이식성은 덤.
 *   대가: 일부 기능(예: 큐 overwrite, task notification)은 v2 에 없어서 직접 조합해야 한다.
 *
 * ---------------------------------------------------------------------------
 * 태스크 (우선순위 높은 순)
 *   ConsumerTask  AboveNormal  큐 대기      [DATA] 출력 + age 계산
 *   SensorTask    Normal       200 ms       VL53L0X single-shot 측정 -> 큐
 *   HeartbeatTask BelowNormal  1 s          상태/카운터 [HB], 5초마다 스택 여유 [STK]
 *   LedTask       Low          500 ms       살아 있음 표시
 *
 *   Consumer 를 Sensor 보다 높게 둔 이유: 샘플이 큐에 들어가는 즉시 Consumer 가 선점해서 꺼내 간다.
 *   -> 정상 상태에서 큐 깊이는 항상 0~1, age = 측정 시간(37~38 ms). 실측: put -> Consumer 실행 22 us.
 *   반대로 두면 Sensor 가 다음 osDelayUntil 로 잠들 때까지 Consumer 가 못 돈다(그래도 동작은 함).
 *
 * 데이터 흐름
 *   SensorTask --(osMessageQueuePut, timeout 0)--> sensorQ(8 x 16 B) --(Get, osWaitForever)--> ConsumerTask
 *   UART 는 태스크 3개가 공유 -> 우선순위 상속 뮤텍스로 직렬화 (uart_printf)
 *
 * 센서 상태 머신 (SensorTask)
 *
 *        init OK                    3회 연속 측정 실패
 *   INIT --------> RUN ----------------------------------> ERROR
 *    ^  \                                                    |
 *    |   \ init 실패 ---------------------------------------->|
 *    |                                                       |
 *    +---- backoff 대기(100->200->...->2000 ms) + I2C 버스 복구 +
 *
 * ---------------------------------------------------------------------------
 * 실험용 컴파일 스위치 (아래 "실험 스위치" 절). 기본값은 정상 운용 설정이다.
 *   QUEUE_POLICY / QUEUE_LEN   큐 overflow 정책 비교 (Day 2 Q0~Q3)
 *   CONSUMER_DELAY_MS          소비 < 생산 과부하 재현
 *   FAULT_INJECT_EVERY         "센서만 조용히 리셋" 고장 주입 -> 복구 경로 반복 시험
 *   VL_FIRST_POLL_MS           data-ready 첫 폴링 지연 (Day 3, 0 이면 Day 2 동작)
 *   TASK_MARKERS / STACK_REPORT 측정용 계측. 끄면 해당 코드가 컴파일에서 빠진다.
 *   남겨둔 이유: README 의 모든 수치를 같은 소스에서 스위치만 바꿔 재현할 수 있게.
 *
 * ST VL53L0X API 에서 우회한 것 2가지 (README "vendor code" 절)
 *   1) VL53L0X_ResetDevice(): MODEL_ID 폴링에 타임아웃 없음 -> 센서 없으면 영구 hang
 *      -> vl_soft_reset(): 폴링 횟수 제한 + MODEL_ID 0xEE 로 존재 확인
 *   2) PerformSingleRangingMeasurement(): 완료 대기 최대 약 4 s
 *      -> vl_measure(): 100 ms 상한 + 결과가 나올 수 없는 구간은 폴링하지 않음
 * ============================================================================= */
#include "app_rtos.h"
#include "main.h"
#include "vl53l0x_api.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;

/* ===================== 실험 스위치 ===================== */
#define STACK_REPORT        1   /* 5초마다 태스크별 최소 스택 여유 [STK] 출력 */
#define TASK_MARKERS        1   /* PA6/PA7 로 태스크 실행 구간 표시 (로직 분석기용) */

/* 고장 주입. 0 = 끔. N 이면 seq 가 N 의 배수일 때마다 센서에 soft reset 을 몰래 보낸다.
 * 재현하려는 상황: 전원 글리치로 센서만 리셋됨. I2C 는 계속 ACK 하지만 측정 설정이 날아가서
 * data-ready 가 영원히 안 뜬다 -> vl_measure 타임아웃(-7) x3 -> ERROR -> 재초기화.
 * 실제 글리치는 손으로 재현하기 어려워서(MCU 까지 POR 됨) 소프트웨어로 같은 결과를 만든다.
 * 50 이면 200 ms x 50 = 약 10초마다 1회. Day 2 결과: 13/13 복구, downtime 819 ms 고정. */
#define FAULT_INJECT_EVERY  0

#define LED_PERIOD_MS       500
#define HB_PERIOD_MS        1000
#define SENSOR_PERIOD_MS    200
#define QUEUE_LEN           8

/* 큐가 가득 찼을 때의 정책
 *  QP_DROP_NEWEST : 새 샘플을 버림. 큐에 오래된 값이 고여 age 가 커짐 (Day 2 Q1: 약 8 s)
 *  QP_DROP_OLDEST : 가장 오래된 샘플을 버리고 새 샘플을 넣음 (Q2: 약 1.5 s)
 *  QUEUE_LEN=1 + QP_DROP_OLDEST = 메일박스, 항상 최신 1개 (Q3: 40~234 ms)
 * 과부하에서 버려지는 '개수'는 생산/소비 속도 차이로 정해지고 정책과 무관하다(Q1 = Q2 = 222).
 * 정책은 '어떤 샘플이 살아남는지', 즉 출력이 얼마나 낡았는지만 바꾼다.
 * 대안이었던 블로킹 put(osWaitForever)은 손실은 없지만 생산자가 소비자 속도로 끌려가서
 * 샘플링 주기 자체가 무너진다(Day 1: 200 ms -> 1 s). 로거에서는 그게 더 나쁘다. */
#define QP_DROP_NEWEST      0
#define QP_DROP_OLDEST      1
#define QUEUE_POLICY        QP_DROP_OLDEST

#define CONSUMER_DELAY_MS   0     /* 과부하 실험: 1000 -> 소비 약 1/s < 생산 5/s */

#define VL_I2C_ADDR         0x52      /* 8-bit 표기 (7-bit 0x29). HAL 은 8-bit 를 받는다 */
#define VL_TIMING_BUDGET_US 33000U    /* ST 기본값. 200 ms 주기 안에 여유 충분 */
#define VL_MODEL_ID         0xEEU     /* reg 0xC0 고정값 = 센서 존재 확인용 */
#define VL_RESET_POLL_MAX   50U       /* soft reset 폴링 최대 횟수 (x 2 ms = 100 ms) */

#define VL_ERR_THRESHOLD    3U        /* RUN -> ERROR 로 가는 연속 실패 횟수. 1회성 glitch 는 흡수 */
#define VL_MEAS_TIMEOUT_MS  100U      /* 측정 완료 대기 상한 (버짓 33 ms 의 약 3배) */
#define VL_BACKOFF_MIN_MS   100U
#define VL_BACKOFF_MAX_MS   2000U

/* 측정 시작 후 첫 data-ready 폴링까지 쉬는 시간 (0 = Day 2 동작: 바로 2 ms 간격 폴링)
 * 버짓 33 ms 전에는 결과가 나올 수 없으므로 그 전의 폴링은 I2C 버스만 점유한다.
 * Day 3 측정 (logs/day3/la/poll0_measure.png vs poll30_measure.png):
 *   헛폴링 15회 -> 2회, 버스 점유 약 11.5 -> 6.3 ms/샘플.
 *   측정 시간 37.8 -> 38.8 ms 는 지연 때문이 아니라 2 ms 폴링 격자의 위상이 바뀐 것.
 *   (센서 실제 준비 시점은 시작 후 34.0~35.1 ms 사이. 감지는 격자에 따라 0~2 ms 늦어진다.)
 * 버짓보다 충분히 작게: osDelay(n) 은 최대 1틱 일찍 깨어날 수 있음 (캡처에서 29.3 ms 로 관찰). */
#define VL_FIRST_POLL_MS    30U
#if VL_FIRST_POLL_MS >= (VL_TIMING_BUDGET_US / 1000U)
  #error "VL_FIRST_POLL_MS must be shorter than the timing budget"
#endif

/* I2C1 핀 (버스 복구 때 GPIO 로 직접 제어) */
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

/* 타이밍 마커. CubeMX 설정에 의존하지 않고 APP_MARKER_INIT 에서 직접 초기화한다.
 * PA6 = Arduino D12 : SensorTask 의 vl_measure() 구간 (HIGH 폭 = 측정 시간)
 * PA7 = Arduino D11 : ConsumerTask 의 UART 출력 구간
 * BSRR 직접 쓰기: 한 명령으로 원자적 set/reset, HAL 호출 오버헤드 없음 -> 마커가 측정을 왜곡하지 않음 */
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

/* LD2(PA5)를 출력으로 설정 */
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

/* 마커 핀 PA6/PA7 을 LOW 로 시작하는 고속 출력으로 설정 */
static void APP_MARKER_INIT(void)
{
#if TASK_MARKERS
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  MK_PORT->BSRR = ((uint32_t)(MK_SENS_PIN | MK_CONS_PIN)) << 16;   /* 출력 켜기 전에 LOW 로 */
  g.Pin   = MK_SENS_PIN | MK_CONS_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(MK_PORT, &g);
#endif
}

/* huart2 가 main.c 에 없으면(BSP COM 모드) 링크 시 &huart2 == NULL 이 되도록 weak 선언 */
extern UART_HandleTypeDef huart2 __attribute__((weak));

/* 로그용 UART 핸들 선택: CubeMX huart2 우선, 없으면 BSP COM1 */
static UART_HandleTypeDef *app_uart(void)
{
  if (&huart2 != NULL) return &huart2;
#if defined(USE_BSP_COM_FEATURE) && (USE_BSP_COM_FEATURE > 0U)
  return &hcom_uart[COM1];
#else
  return NULL;
#endif
}

/* UART 송신 (blocking polling). 115200 bps 에서 한 줄 약 6 ms 동안 CPU 를 붙잡는다.
 * 뮤텍스 없이 부르는 곳은 스케줄러 시작 전(App_RTOS_Init)뿐. 태스크에서는 uart_printf 사용.
 * 한계: DMA 를 쓰면 이 6 ms 동안 다른 태스크가 돌 수 있다 (README next steps). */
static void uart_write(const char *s, uint16_t len)
{
  UART_HandleTypeDef *h = app_uart();
  if (h != NULL) {
    HAL_UART_Transmit(h, (uint8_t *)s, len, 100);
  }
}

/* ===================== 데이터 타입 ===================== */
/* 큐 원소 16 B. CMSIS 큐는 값 복사라 포인터가 아닌 구조체를 통째로 넣는다
 * -> 생산자 스택 변수 수명 문제가 없다. */
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

/* 카운터는 SensorTask 만 쓰고 HeartbeatTask 가 읽는다. 32-bit 정렬 읽기/쓰기는 Cortex-M4 에서
 * 원자적이라 잠금 없이 충분 (읽는 쪽이 한 박자 늦은 값을 볼 수는 있음). */
static volatile uint32_t  dropCount;    /* 큐가 가득 차서 버린 샘플 수                  */
static volatile uint32_t  vlFailCount;  /* 실패한 측정/초기화 시도 누적 ([HB] err=).
                                         * I2C NACK(-20) 뿐 아니라 타임아웃(-7)도 센다 */
static volatile int32_t   lastVlErr;    /* 마지막 VL53L0X_Error 코드 ([HB] last=)       */
static volatile uint32_t  recoverCount; /* ERROR -> RUN 복구 성공 횟수 ([HB] rec=)      */
static volatile uint32_t  missedSlots;  /* 주기를 놓쳐서 건너뛴 슬롯 수 ([HB] miss=)    */

typedef enum { VL_ST_INIT = 0, VL_ST_RUN, VL_ST_ERROR } VlState_t;
static volatile VlState_t vlState = VL_ST_INIT;
static const char *const vlStateName[] = { "INIT", "RUN", "ERROR" };

static VL53L0X_Dev_t vlDev;            /* 수백 B 라 태스크 스택 대신 static */

/* stack_size 단위는 byte (CMSIS v2). ST API 를 부르는 Sensor 는 넉넉하게.
 * 실측 최소 여유: LED 360~384, HB 1416, Sens 1132, Cons 1408 B ([STK]). LED 외에는 절반 이상 남음. */
static const osThreadAttr_t ledAttr  = { .name = "LED",  .stack_size = 128 * 4, .priority = osPriorityLow };
static const osThreadAttr_t hbAttr   = { .name = "HB",   .stack_size = 512 * 4, .priority = osPriorityBelowNormal };
static const osThreadAttr_t sensAttr = { .name = "Sens", .stack_size = 512 * 4, .priority = osPriorityNormal };
static const osThreadAttr_t consAttr = { .name = "Cons", .stack_size = 512 * 4, .priority = osPriorityAboveNormal };
static const osMessageQueueAttr_t queueAttr = { .name = "sensorQ" };

/* 우선순위 상속 뮤텍스. 상황: HB(BelowNormal)가 UART 를 잡고 6 ms 출력 중에 Consumer(AboveNormal)가
 * 같은 뮤텍스를 기다리면, 그 사이 Sensor(Normal)가 HB 를 선점해서 Consumer 가 HB 보다도 오래
 * 기다리는 '우선순위 역전'이 생길 수 있다. 상속이 켜져 있으면 HB 가 잠깐 AboveNormal 로 올라가
 * 출력을 빨리 끝낸다. (FreeRTOS 뮤텍스는 원래 상속을 하지만, CMSIS 의도로 명시해 둔다.)
 * 대안 세마포어(binary)는 소유자 개념이 없어서 상속이 불가능하다. */
static const osMutexAttr_t uartMutexAttr = { .name = "uart", .attr_bits = osMutexPrioInherit };

/* ===================== UART 헬퍼 (태스크 전용) ===================== */
/* printf 형식으로 한 줄 포맷 후 뮤텍스를 잡고 송신. 줄 단위로 섞이지 않게 하는 게 목적.
 * 포맷은 뮤텍스 밖(각 태스크 스택의 128 B 버퍼)에서 해서 임계 구역을 송신 시간으로만 줄인다.
 * ISR 에서는 부를 수 없다 (뮤텍스는 ISR 에서 사용 불가). */
static void uart_printf(const char *fmt, ...)
{
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof buf, fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n >= (int)sizeof buf) n = (int)sizeof buf - 1;   /* 잘린 경우 실제 들어간 길이 */

  if (osMutexAcquire(uartMutex, osWaitForever) == osOK) {
    uart_write(buf, (uint16_t)n);
    osMutexRelease(uartMutex);
  }
}

/* ===================== I2C 버스 복구 ===================== */
/* 비트뱅잉용 짧은 지연 (수 us). I2C 는 클럭을 마스터가 정하므로 느려도 문제없다. */
static void i2c_bb_delay(void)
{
  for (volatile uint32_t i = 0; i < 200U; i++) { }
}

/* 버스가 막혔을 때의 표준 복구 절차.
 *  1) 슬레이브가 전송 도중 끊겨 SDA 를 LOW 로 잡고 있으면 SCL 을 최대 9번 쳐서 남은 비트를 밀어내게 함
 *  2) STOP 조건을 직접 만들어 버스를 idle 로
 *  3) I2C1 peripheral 을 RCC 로 리셋: STM32F4 의 BUSY 플래그 고착(errata)은 DeInit/Init 만으로 안 풀린다
 * 비용은 수백 us 이고 ERROR 상태에서만 부르므로 정상 경로에 영향 없음. */
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
  for (int i = 0; i < 9; i++) {           /* SCL 9클럭: SDA 가 풀리면 중단 */
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
/* soft reset 실패 진단용 (vl_init 이 [VL] soft reset timeout 줄에 출력) */
static uint8_t vlRstPhase;   /* 1 = 리셋 진입 대기, 2 = 부팅 완료 대기 */
static uint8_t vlRstLastId;  /* 실패 시점에 마지막으로 읽은 MODEL_ID   */

/* ST VL53L0X_ResetDevice() 대체: 리셋 진입(ID=0x00)과 부팅 완료(ID=0xEE)를 각각 제한된 횟수만 폴링.
 * 원본은 타임아웃 없는 while 이라 센서가 빠지면 태스크가 영구히 멈춘다.
 * XSHUT 핀이 없는 4핀 모듈이라 이게 소프트웨어로 할 수 있는 유일한 센서 리셋이다.
 * 반환: NONE / TIME_OUT(I2C 는 되는데 상태 전이가 안 보임) / I2C 에러(-20, 센서 없음) */
static VL53L0X_Error vl_soft_reset(VL53L0X_DEV dev)
{
  VL53L0X_Error st;
  uint8_t id = 0xFF;
  uint32_t i;

  vlRstPhase = 1;

  st = VL53L0X_WrByte(dev, VL53L0X_REG_SOFT_RESET_GO2_SOFT_RESET_N, 0x00);
  if (st != VL53L0X_ERROR_NONE) return st;

  for (i = 0; i < VL_RESET_POLL_MAX; i++) {          /* 리셋 진입: ID 가 0 으로 읽힘 */
    st = VL53L0X_RdByte(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &id);
    if (st == VL53L0X_ERROR_NONE && id == 0x00) break;
    osDelay(2);                                      /* 폴링 사이에 CPU 양보 */
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

/* single-shot 측정 1회. ST PerformSingleRangingMeasurement 와 같은 순서지만
 *  (1) 결과가 나올 수 없는 구간(VL_FIRST_POLL_MS)은 폴링하지 않고 osDelay 로 CPU 와 버스를 양보
 *  (2) 완료 대기 상한을 VL_MEAS_TIMEOUT_MS 로 제한 (ST 원본은 약 4 s)
 *      -> 센서가 조용히 리셋돼 data-ready 가 안 뜨는 고장을 100 ms 안에 -7 로 감지
 * t0 를 Start 전에 잡으므로 첫 폴링 지연도 100 ms 상한 안에 포함된다.
 * StartMeasurement 안의 start-bit 대기 루프는 delay 없는 I2C read 라 수 ms 안에 끝난다.
 * 대안: GPIO1(인터럽트) 핀을 쓰면 폴링 자체가 없어지지만 이 모듈에는 핀이 없다. */
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
    osDelay(2);   /* 폴링 간격 2 ms = 감지 지연의 양자화 단위 (0~2 ms) */
  }

  if ((st = VL53L0X_GetRangingMeasurementData(dev, m)) != VL53L0X_ERROR_NONE) return st;
  return VL53L0X_ClearInterruptMask(dev, 0);
}

/* 센서 초기화 전체. 논문 펌웨어와 같은 순서, 단 ResetDevice 대신 vl_soft_reset, 모드는 single-shot.
 * 태스크 안에서만 부른다: ST API 의 폴링이 osDelay 를 쓰므로 스케줄러가 돌고 있어야 한다.
 * single-shot 을 고른 이유: continuous 모드는 센서 자체 타이머로 측정해서 샘플 시각이
 * MCU 주기(200 ms)와 어긋나며 흘러간다. single-shot 은 측정 시작을 osDelayUntil 격자에 고정한다. */
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
    /* I2C 는 살아 있는데 리셋 상태 전이가 안 보이는 경우: 진단만 남기고 DataInit 으로 진행.
     * DataInit/StaticInit 이 필요한 레지스터를 다시 쓰므로 그걸로 복구되는지 본다. */
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
/* LED 토글. 주기가 정확할 필요 없어서 osDelay (상대 지연) 사용. */
static void LedTask(void *arg)
{
  (void)arg;
  for (;;) {
    APP_LED_TOGGLE();
    osDelay(LED_PERIOD_MS);
  }
}

/* 1초마다 상태 한 줄, 5초마다 스택 최소 여유. 시스템이 살아 있다는 증거이자 카운터 대시보드.
 * osThreadGetStackSpace: 부팅 후 '가장 적게 남았던' 바이트 수(high-water mark). 0 에 가까우면 위험. */
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
                (unsigned long)vlFailCount, (long)lastVlErr,
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

/* 큐에 넣기. timeout 0 = 비블로킹: 가득 차 있으면 기다리지 않고 즉시 실패(osErrorResource)를 돌려준다.
 * 생산자가 절대 소비자에게 끌려가지 않게 하는 핵심 (Day 1 블로킹 실험 참고).
 * DROP_OLDEST: CMSIS v2 에는 '덮어쓰기 put' 이 없어서 Get(가장 오래된 것 버림) + Put 으로 조합한다.
 *   Get 과 Put 사이에 더 높은 우선순위인 Consumer 가 끼어들어 하나를 가져가도 그건 '소비'라 손실이 아니다.
 *   두 번째 Put 은 실패할 수 없지만(방금 한 칸 비웠으므로) 방어적으로 카운트한다. */
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

/* 상태 전이 + 로그. 같은 상태로의 '전이'는 출력하지 않는다. */
static void vl_set_state(VlState_t s)
{
  if (vlState != s) {
    uart_printf("[VL] state %s -> %s (tick=%lu)\r\n",
                vlStateName[vlState], vlStateName[s],
                (unsigned long)osKernelGetTickCount());
    vlState = s;
  }
}

/* 센서 태스크: INIT -> RUN -> ERROR 상태 머신 (파일 머리말의 그림 참고). */
static void SensorTask(void *arg)
{
  (void)arg;
  VL53L0X_Error st;
  uint32_t seq        = 0;
  uint32_t consecErr  = 0;
  uint32_t backoff    = VL_BACKOFF_MIN_MS;
  uint32_t attempts   = 0;      /* 이번 복구 구간에서 init 시도 횟수 */
  uint8_t  recovering = 0;      /* 1 = ERROR 를 한 번이라도 거침       */
  uint32_t downSince  = osKernelGetTickCount();
  uint32_t next       = 0;      /* 다음 측정 시작 예정 시각 (절대 tick) */

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
        /* 주기 기준점을 '지금'으로 재설정. 안 하면 next 가 과거라서 다운타임 동안 밀린 슬롯을
         * 몰아서 실행하려 든다 (catch-up burst). */
        next       = osKernelGetTickCount();
        vl_set_state(VL_ST_RUN);
      } else {
        lastVlErr = st;
        vlFailCount++;
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
        vlFailCount++;
        /* downtime 은 '마지막 정상 샘플 이후 첫 실패' 시점부터 잰다.
         * (3번째 실패 시점부터 재면 판정에 걸린 약 0.4 s 가 빠져 과소 측정된다) */
        if (++consecErr == 1U) downSince = t0;
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

      /* 주기 유지: osDelayUntil(절대 시각) vs osDelay(상대 지연)
       *  osDelay(200) 은 '지금부터 200 ms' 라서 측정 시간(약 38 ms)과 선점 지연이 매 주기 누적된다
       *  -> 주기가 238 ms 근처로 늘고 흔들린다. osDelayUntil 은 next 라는 고정 격자에 깨어나므로
       *  측정 시간이 얼마든 주기는 200 ms 그대로. 실측 200.020 ms, p-p 지터 약 1 us (HSE).
       * 함정: next 가 이미 지났으면 CMSIS 래퍼는 기다리지 않고 즉시 돌아온다(osErrorParameter)
       *  -> 밀린 만큼 연달아 측정하는 burst. 그래서 지난 슬롯은 건너뛰고 miss 로 센다. */
      next += SENSOR_PERIOD_MS;
      {
        uint32_t now = osKernelGetTickCount();
        if ((int32_t)(next - now) <= 0) {          /* tick 랩어라운드에도 안전한 비교 */
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
      /* 지수 backoff: 센서가 금방 돌아오면 빨리(100 ms), 오래 빠져 있으면 버스와 로그를 덜 괴롭힘(최대 2 s) */
      osDelay(backoff);
      backoff = (backoff * 2U > VL_BACKOFF_MAX_MS) ? VL_BACKOFF_MAX_MS : backoff * 2U;
      i2c_bus_recover();
      vl_set_state(VL_ST_INIT);
      break;
    }
  }
}

/* 소비자: 큐에서 꺼내 [DATA] 한 줄 출력. osWaitForever 로 대기하므로 샘플이 없을 때는 CPU 0.
 * 폴링(주기적으로 큐 확인)과 달리 샘플이 들어오는 순간 깨어난다 (실측 22 us). */
static void ConsumerTask(void *arg)
{
  (void)arg;
  SensorSample_t s;

  for (;;) {
    if (osMessageQueueGet(sensorQueue, &s, NULL, osWaitForever) == osOK) {
      /* age: 측정 시작(tick)부터 소비자가 꺼낸 시점까지 = 로거가 보여주는 값이 얼마나 낡았는지 */
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
    /* 과부하 실험용. osDelay(상대)라서 출력 시간 약 6 ms 가 매 줄 누적된다
     * -> Day 2 Q3 에서 age 가 줄마다 6 ms 씩 늘어나는 이유. */
    osDelay(CONSUMER_DELAY_MS);
#endif
  }
}

/* ===================== 초기화 ===================== */
/* 스케줄러 시작 전 main.c 에서 1회 호출. 부팅 진단 출력 후 커널 객체와 태스크를 만든다.
 * 이 시점에는 태스크가 아직 안 돌기 때문에 UART 는 뮤텍스 없이 직접 쓴다. */
void App_RTOS_Init(void)
{
  APP_LED_INIT();
  APP_MARKER_INIT();

  static const char banner[] = "\r\n=== f446-rtos-logger boot ===\r\n";
  uart_write(banner, sizeof banner - 1);

  /* 리셋 원인: RCC->CSR 플래그는 다음 리셋까지 남아 있으므로 읽고 나서 지운다.
   * F4 는 내부 리셋 시 NRST 도 LOW 로 끌려서 PIN 은 거의 항상 같이 뜬다.
   *  POR PIN BOR = 전원 저하(약 1.7 V 미만), PIN 만 = 버튼/ST-LINK/NRST 노이즈 */
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
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }

  /* 실제 클럭 소스를 레지스터에서 확인 (.ioc 설정이 아니라 지금 돌고 있는 값).
   * HSI(RC 발진) -> HSE bypass 로 바꾼 뒤 주기 오차 -1100 ppm -> -33 ppm, 지터 70 us -> 1 us. */
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

  /* 실험 스위치 값. 로그 파일 하나만으로 어떤 설정으로 찍었는지 증명하기 위함 */
  {
    char line[128];
    int n = snprintf(line, sizeof line,
                     "[CFG] queue=%s len=%u cons_delay=%u ms first_poll=%u ms fault_inject=%u\r\n",
                     (QUEUE_POLICY == QP_DROP_OLDEST) ? "DROP_OLDEST" : "DROP_NEWEST",
                     (unsigned)QUEUE_LEN, (unsigned)CONSUMER_DELAY_MS,
                     (unsigned)VL_FIRST_POLL_MS, (unsigned)FAULT_INJECT_EVERY);
    uart_write(line, (uint16_t)n);
  }

  /* I2C 버스 스캔: 센서 응답만 먼저 확인 (실제 초기화는 SensorTask 에서) */
  {
    char line[64];
    HAL_StatusTypeDef hs = HAL_I2C_IsDeviceReady(&hi2c1, VL_I2C_ADDR, 3, 10);
    int n = snprintf(line, sizeof line, "[I2C] VL53L0X @0x29: %s\r\n",
                     hs == HAL_OK ? "ACK" : "NO RESPONSE");
    uart_write(line, (uint16_t)n);
  }

  /* 커널 객체는 태스크보다 먼저 만든다: 태스크가 시작되자마자 큐/뮤텍스를 쓰기 때문.
   * 모두 FreeRTOS 힙(heap_4, 15360 B)에서 할당된다. */
  uartMutex   = osMutexNew(&uartMutexAttr);
  sensorQueue = osMessageQueueNew(QUEUE_LEN, sizeof(SensorSample_t), &queueAttr);
  ledTask     = osThreadNew(LedTask, NULL, &ledAttr);
  hbTask      = osThreadNew(HeartbeatTask, NULL, &hbAttr);
  sensTask    = osThreadNew(SensorTask, NULL, &sensAttr);
  consTask    = osThreadNew(ConsumerTask, NULL, &consAttr);

  /* NULL 이면 대부분 FreeRTOS 힙 부족 -> FreeRTOSConfig.h 의 TOTAL_HEAP_SIZE 확인 */
  if (uartMutex == NULL || sensorQueue == NULL ||
      ledTask == NULL || hbTask == NULL || sensTask == NULL || consTask == NULL) {
    Error_Handler();
  }
}
