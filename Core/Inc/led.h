#ifndef LED_H_
#define LED_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Led_Init(void);
void Led_Process(void);

/* Вызывается из CDC при активации/деактивации интерфейса (надёжнее одного только опроса стека). */
void Led_SetUsbConfigured(uint8_t configured);

/* Активность CAN, которую мост обрабатывает: TX на шину (из USB) или RX с шины (в сторону USB). */
void Led_NotifyCanTx(void);

#ifdef __cplusplus
}
#endif

#endif /* LED_H_ */
