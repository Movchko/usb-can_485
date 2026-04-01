#ifndef USB_DEVICE_STATUS_H_
#define USB_DEVICE_STATUS_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CDC активен (хост открыл интерфейс после конфигурации). */
uint8_t UsbDevice_CdcReady(void);

/* 1 = USB связан с хостом (CDC / CONFIGURED / адрес на шине). */
uint8_t UsbDevice_IsConfigured(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_DEVICE_STATUS_H_ */
