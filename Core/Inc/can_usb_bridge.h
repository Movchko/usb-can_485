#ifndef CAN_USB_BRIDGE_H_
#define CAN_USB_BRIDGE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSU_PKT_TYPE_CAN   0u
#define BSU_PKT_TYPE_CAN2  1u

void Bridge_Init(void);
void Bridge_Process(void);

/* Вызывать при приёме USB-данных (из USB-слоя). */
void Bridge_UsbRx(uint8_t *buf, uint32_t len);

/* Вызывать из USB-слоя по завершению передачи. */
void Bridge_UsbTxComplete(void);

/* Вызывать из HAL_FDCAN_RxFifo0Callback.
 * can_bus: 1 = CAN1, 2 = CAN2 */
void Bridge_CanRxPush(uint8_t can_bus, uint32_t can_id, const uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* CAN_USB_BRIDGE_H_ */
