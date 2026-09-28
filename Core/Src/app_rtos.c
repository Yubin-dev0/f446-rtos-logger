/* app_rtos.c  -> Core/Src/ 에 넣기
 * FreeRTOS Day 1: LED / Heartbeat(UART) / Producer -> Queue -> Consumer(UART)
 * API: CMSIS-RTOS v2 only (native FreeRTOS API 섞지 않음)
 */
#include "app_rtos.h"
#include "main.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* ===================== 실험 스위치 ===================== */
#define ENABLE_QUEUE_DEMO   0   /* 4단계(태스크 2개): 0,  5단계(Queue)부터: 1       */
#define QUEUE_FULL_TEST     0   /* 6단계: 1 -> 소비자를 느리게 해서 Queue 포화     */
#define STACK_REPORT        1   /* 5초마다 태스크별 최소 스택 여유 출력            */

#define LED_PERIOD_MS       500
#define HB_PERIOD_MS        1000
#define PRODUCER_PERIOD_MS  200
#define QUEUE_LEN           8

#if QUEUE_FULL_TEST
#define CONSUMER_DELAY_MS   1000   /* 소비 1개/1s < 생산 5개/1s -> 가득 참 */
#else
#define CONSUMER_DELAY_MS   0
#endif

/* ===================== 보드 매핑 =====================
 * CubeMX가 보드를 어떤 모드로 생성했는지에 따라 LED/UART 이름이 다르다.
 *  - 일반 모드: PA5 = LD2_GPIO_Port/LD2_Pin, UART = huart2 (main.c)
 *  - BSP  모드: PA5 = BSP LED_GREEN,        UART = hcom_uart[COM1] (stm32f4xx_nucleo.c)
 * 둘 다 자동으로 처리한다.
 */
#if defined(LD2_Pin)
  #define APP_LED_PORT   LD2_GPIO_Port
  #define APP_LED_PIN    LD2_Pin
#else
  #define APP_LED_PORT   GPIOA          /* NUCLEO-F446RE LD2 = PA5 고정 */
  #define APP_LED_PIN    GPIO_PIN_5
#endif
#define APP_LED_TOGGLE()   HAL_GPIO_TogglePin(APP_LED_PORT, APP_LED_PIN)

/* BSP API 이름은 패키지 버전마다 달라서 쓰지 않고, PA5를 직접 출력으로 설정.
 * MX_GPIO_Init()/BSP가 이미 설정했어도 같은 값으로 다시 쓰는 거라 무해. */
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
  uint32_t seq;         /* 연속 번호: 빠진 번호 = 드롭된 샘플 */
  uint32_t tick;        /* 생산 시각 (ms)                      */
  int16_t  temp_centi;  /* 0.01 degC (float printf 회피)        */
  uint16_t dist_mm;     /* 모의 거리                            */
} SensorSample_t;

/* ===================== 핸들 ===================== */
static osThreadId_t       ledTask, hbTask, prodTask, consTask;
static osMessageQueueId_t sensorQueue;
static osMutexId_t        uartMutex;
static volatile uint32_t  dropCount;

/* 우선순위: Consumer > Producer > Heartbeat > LED
 * stack_size 단위는 byte. printf 쓰는 태스크는 넉넉하게. */
static const osThreadAttr_t ledAttr  = { .name = "LED",  .stack_size = 128 * 4, .priority = osPriorityLow };
static const osThreadAttr_t hbAttr   = { .name = "HB",   .stack_size = 512 * 4, .priority = osPriorityBelowNormal };
#if ENABLE_QUEUE_DEMO
static const osThreadAttr_t prodAttr = { .name = "Prod", .stack_size = 256 * 4, .priority = osPriorityNormal };
static const osThreadAttr_t consAttr = { .name = "Cons", .stack_size = 512 * 4, .priority = osPriorityAboveNormal };
static const osMessageQueueAttr_t queueAttr = { .name = "sensorQ" };
#endif

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

  /* 여러 태스크가 동시에 찍으면 줄이 섞이므로 mutex로 직렬화 */
  if (osMutexAcquire(uartMutex, osWaitForever) == osOK) {
    uart_write(buf, (uint16_t)n);
    osMutexRelease(uartMutex);
  }
}

/* ===================== 태스크 ===================== */
static void LedTask(void *arg)
{
  (void)arg;
  for (;;) {
    APP_LED_TOGGLE();              /* PA5 (LD2) */
    osDelay(LED_PERIOD_MS);
  }
}

static void HeartbeatTask(void *arg)
{
  (void)arg;
  uint32_t n = 0;
  for (;;) {
    uint32_t qCount = sensorQueue ? osMessageQueueGetCount(sensorQueue) : 0;
    uart_printf("[HB] tick=%lu q=%lu/%u drop=%lu\r\n",
                (unsigned long)osKernelGetTickCount(),
                (unsigned long)qCount, QUEUE_LEN,
                (unsigned long)dropCount);

#if STACK_REPORT
    if ((++n % 5U) == 0U) {
      /* osThreadGetStackSpace = uxTaskGetStackHighWaterMark x 4 (byte, 지금까지의 최소 여유) */
      uart_printf("[STK] min free bytes: LED=%lu HB=%lu Prod=%lu Cons=%lu\r\n",
                  (unsigned long)osThreadGetStackSpace(ledTask),
                  (unsigned long)osThreadGetStackSpace(hbTask),
                  (unsigned long)(prodTask ? osThreadGetStackSpace(prodTask) : 0),
                  (unsigned long)(consTask ? osThreadGetStackSpace(consTask) : 0));
    }
#else
    (void)n;
#endif
    osDelay(HB_PERIOD_MS);
  }
}

#if ENABLE_QUEUE_DEMO
static void ProducerTask(void *arg)
{
  (void)arg;
  uint32_t seq  = 0;
  uint32_t next = osKernelGetTickCount();

  for (;;) {
    SensorSample_t s;
    uint32_t p   = seq % 40U;
    int32_t  tri = (p < 20U) ? (int32_t)p : (int32_t)(40U - p);   /* 0..20..0 */

    s.seq        = seq++;
    s.tick       = osKernelGetTickCount();
    s.temp_centi = (int16_t)(2400 + tri * 10);                    /* 24.00~26.00 C */
    s.dist_mm    = (uint16_t)(100U + (s.seq * 37U) % 900U);       /* 100~999 mm   */

    /* timeout 0: 가득 차면 기다리지 않고 osErrorResource -> 드롭 카운트 */
    if (osMessageQueuePut(sensorQueue, &s, 0U, 0U) != osOK) {
      dropCount++;
    }

    next += PRODUCER_PERIOD_MS;
    osDelayUntil(next);            /* 처리 시간과 무관하게 정확히 200ms 주기 */
  }
}

static void ConsumerTask(void *arg)
{
  (void)arg;
  SensorSample_t s;

  for (;;) {
    if (osMessageQueueGet(sensorQueue, &s, NULL, osWaitForever) == osOK) {
      uart_printf("[DATA] seq=%lu t=%lu temp=%d.%02d C dist=%u mm\r\n",
                  (unsigned long)s.seq, (unsigned long)s.tick,
                  s.temp_centi / 100, s.temp_centi % 100,
                  (unsigned)s.dist_mm);
    }
#if CONSUMER_DELAY_MS
    osDelay(CONSUMER_DELAY_MS);
#endif
  }
}
#endif /* ENABLE_QUEUE_DEMO */

/* ===================== 초기화 ===================== */
void App_RTOS_Init(void)
{
  APP_LED_INIT();

  /* 스케줄러 시작 전이라 mutex 없이 직접 송신 */
  static const char banner[] = "\r\n=== f446-rtos-logger day1 boot ===\r\n";
  uart_write(banner, sizeof banner - 1);

  uartMutex = osMutexNew(&uartMutexAttr);
  ledTask   = osThreadNew(LedTask, NULL, &ledAttr);
  hbTask    = osThreadNew(HeartbeatTask, NULL, &hbAttr);

#if ENABLE_QUEUE_DEMO
  sensorQueue = osMessageQueueNew(QUEUE_LEN, sizeof(SensorSample_t), &queueAttr);
  prodTask    = osThreadNew(ProducerTask, NULL, &prodAttr);
  consTask    = osThreadNew(ConsumerTask, NULL, &consAttr);
  if (sensorQueue == NULL || prodTask == NULL || consTask == NULL) Error_Handler();
#endif

  /* NULL이면 대부분 FreeRTOS heap 부족 -> TOTAL_HEAP_SIZE 확인 */
  if (uartMutex == NULL || ledTask == NULL || hbTask == NULL) Error_Handler();
}
