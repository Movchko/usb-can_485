/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can_usb_bridge.h"
#include "led.h"
#include "ux_system.h"
#include "usb_device_status.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static UX_SLAVE_CLASS_CDC_ACM *g_cdc_acm = UX_NULL;
static volatile uint8_t g_cdc_ready = 0u;
volatile ULONG g_usb_cdc_rx_cb_count = 0u;
volatile ULONG g_usb_cdc_rx_cb_bytes = 0u;
volatile ULONG g_usb_cdc_tx_cb_count = 0u;
volatile UINT g_usb_cdc_start_status = UX_SUCCESS;
volatile UINT g_usb_cdc_last_write_status = UX_SUCCESS;
volatile ULONG g_usb_cdc_last_write_len = 0u;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static UINT CdcReadCb(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UINT status, UCHAR *data_pointer, ULONG length);
static UINT CdcWriteCb(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UINT status, ULONG length);
static UINT CdcStartTransmission(UX_SLAVE_CLASS_CDC_ACM *cdc_acm);
static UX_SLAVE_CLASS_CDC_ACM *CdcFindInstanceFromClassArray(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */
  UX_SLAVE_CLASS_CDC_ACM_CALLBACK_PARAMETER cb;
  g_cdc_acm = (UX_SLAVE_CLASS_CDC_ACM *)cdc_acm_instance;
  g_cdc_ready = 1u;
  Led_SetUsbConfigured(1u);
  cb.ux_device_class_cdc_acm_parameter_write_callback = CdcWriteCb;
  cb.ux_device_class_cdc_acm_parameter_read_callback = CdcReadCb;
  g_usb_cdc_start_status = ux_device_class_cdc_acm_ioctl(g_cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_START, &cb);
  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  g_cdc_ready = 0u;
  Led_SetUsbConfigured(0u);
  g_cdc_acm = UX_NULL;
  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */
  UX_PARAMETER_NOT_USED(cdc_acm_instance);
  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */
uint8_t UsbDevice_CdcReady(void)
{
  return g_cdc_ready;
}

static UINT CdcReadCb(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UINT status, UCHAR *data_pointer, ULONG length)
{
  UX_PARAMETER_NOT_USED(cdc_acm);
  if ((status == UX_SUCCESS) && (data_pointer != UX_NULL) && (length > 0u)) {
    g_usb_cdc_rx_cb_count++;
    g_usb_cdc_rx_cb_bytes += length;
    Bridge_UsbRx((uint8_t *)data_pointer, (uint32_t)length);
  }
  return UX_SUCCESS;
}

static UINT CdcWriteCb(UX_SLAVE_CLASS_CDC_ACM *cdc_acm, UINT status, ULONG length)
{
  UX_PARAMETER_NOT_USED(cdc_acm);
  UX_PARAMETER_NOT_USED(status);
  UX_PARAMETER_NOT_USED(length);
  g_usb_cdc_tx_cb_count++;
  Bridge_UsbTxComplete();
  return UX_SUCCESS;
}

INT USBD_CDC_ACM_Transmit(uint8_t *buf, uint16_t len)
{
  if ((g_cdc_ready == 0u) || (g_cdc_acm == UX_NULL) || (buf == UX_NULL) || (len == 0u)) {
    return -1;
  }
  g_usb_cdc_last_write_status = ux_device_class_cdc_acm_write_with_callback(g_cdc_acm, (UCHAR *)buf, (ULONG)len);
  g_usb_cdc_last_write_len = (ULONG)len;
  if (g_usb_cdc_last_write_status == UX_SUCCESS) {
    return 0;
  }
  return -1;
}

VOID USBD_CDC_ACM_Process(VOID)
{
  if ((g_cdc_acm == UX_NULL) && (_ux_system_slave != UX_NULL)) {
    g_cdc_acm = CdcFindInstanceFromClassArray();
    if (g_cdc_acm != UX_NULL) {
      g_cdc_ready = 1u;
    }
  }

  if ((g_cdc_acm != UX_NULL) && (g_cdc_ready != 0u) &&
      (g_cdc_acm->ux_slave_class_cdc_acm_transmission_status == UX_FALSE)) {
    g_usb_cdc_start_status = CdcStartTransmission(g_cdc_acm);
  }
}

int8_t Bridge_UsbTransmit(const uint8_t *buf, uint16_t len)
{
  return (int8_t)USBD_CDC_ACM_Transmit((uint8_t *)buf, len);
}

static UINT CdcStartTransmission(UX_SLAVE_CLASS_CDC_ACM *cdc_acm)
{
  UX_SLAVE_CLASS_CDC_ACM_CALLBACK_PARAMETER cb;
  if (cdc_acm == UX_NULL) {
    return UX_INVALID_PARAMETER;
  }
  cb.ux_device_class_cdc_acm_parameter_write_callback = CdcWriteCb;
  cb.ux_device_class_cdc_acm_parameter_read_callback = CdcReadCb;
  return ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_TRANSMISSION_START, &cb);
}

static UX_SLAVE_CLASS_CDC_ACM *CdcFindInstanceFromClassArray(void)
{
  UX_SLAVE_CLASS *cls;
  ULONG i;

  if ((_ux_system_slave == UX_NULL) || (_ux_system_slave->ux_system_slave_class_array == UX_NULL)) {
    return UX_NULL;
  }

  cls = _ux_system_slave->ux_system_slave_class_array;
  for (i = 0u; i < (ULONG)UX_SYSTEM_DEVICE_MAX_CLASS_GET(); i++) {
    if ((cls[i].ux_slave_class_status != UX_UNUSED) &&
        (cls[i].ux_slave_class_entry_function == ux_device_class_cdc_acm_entry) &&
        (cls[i].ux_slave_class_instance != UX_NULL)) {
      return (UX_SLAVE_CLASS_CDC_ACM *)cls[i].ux_slave_class_instance;
    }
  }

  return UX_NULL;
}

/* USER CODE END 1 */
