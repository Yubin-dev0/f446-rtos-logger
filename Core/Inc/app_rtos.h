/* app_rtos.h  -> Core/Inc/ 에 넣기 */
#ifndef APP_RTOS_H
#define APP_RTOS_H

#include "cmsis_os2.h"

void App_RTOS_Init(void);   /* freertos.c 의 USER CODE RTOS_THREADS 에서 호출 */

#endif /* APP_RTOS_H */
