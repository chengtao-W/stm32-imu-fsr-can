/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
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

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_cdc.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "fsr.h"
/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define CDC_RX_BUF_SIZE  512
static uint8_t cdc_rx_buf[CDC_RX_BUF_SIZE];
static bool    fsr_connected = false;
static bool    cdc_rx_armed = false;
static bool    fsr_linecoding_pending = false;
static uint32_t fsr_linecoding_tick = 0;
static CDC_LineCodingTypeDef moxian_linecoding;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostHS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */
static void USBH_CDC_TryArmReceive(USBH_HandleTypeDef *phost)
{
    if (!fsr_connected || cdc_rx_armed) {
        return;
    }
    if (USBH_CDC_Receive(phost, cdc_rx_buf, CDC_RX_BUF_SIZE) == USBH_OK) {
        cdc_rx_armed = true;
    }
}


static void USBH_CDC_StartMoxian(USBH_HandleTypeDef *phost)
{
    if (fsr_connected) {
        return;
    }

    phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_TYPE_CLASS |
                                           USB_REQ_RECIPIENT_INTERFACE;
    phost->Control.setup.b.bRequest = CDC_SET_CONTROL_LINE_STATE;
    phost->Control.setup.b.wValue.w = 0x0003U;  /* DTR=1, RTS=1 */
    phost->Control.setup.b.wIndex.w = 0U;
    phost->Control.setup.b.wLength.w = 0U;
    USBH_CtlReq(phost, NULL, 0U);

    FSR_SendStartCapture(phost);
    fsr_connected = true;
    cdc_rx_armed = false;
    USBH_CDC_TryArmReceive(phost);
    if (!cdc_rx_armed) {
        printf("[USB CDC] WARN: initial RX arm failed, will retry\r\n");
    }
}
/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */

  /* USER CODE END USB_HOST_Init_PreTreatment */

  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostHS, USBH_UserProcess, HOST_HS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostHS, USBH_CDC_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_Start(&hUsbHostHS) != USBH_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */
  printf("USB Host CDC started. Waiting for sensor...\r\n");
  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * Background task
 */
void MX_USB_HOST_Process(void)
{
  /* USB Host Background task */
  USBH_Process(&hUsbHostHS);

  /* Print host state on every change (no delay) */
  {
    static uint8_t last_state = 0xFF;
    uint8_t s = hUsbHostHS.gState;
    if (s != last_state) {
        printf("USBH state: %d\r\n", s);
        last_state = s;
    }
  }

  /*
   * Fallback: if LineCodingChanged never fired (sensor doesn't support
   * GET_LINE_CODING), force initialization after timeout.
   */
  if (fsr_linecoding_pending && !fsr_connected && (HAL_GetTick() - fsr_linecoding_tick) > 300U) {
    fsr_linecoding_pending = false;
    printf("[USB CDC] WARN: line coding callback timeout, start anyway\r\n");
    USBH_CDC_StartMoxian(&hUsbHostHS);
  }
  USBH_CDC_TryArmReceive(&hUsbHostHS);
}
/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
  printf("USB config selected. VID=%04X PID=%04X interfaces=%u\r\n",
         (unsigned int)phost->device.DevDesc.idVendor,
         (unsigned int)phost->device.DevDesc.idProduct,
         (unsigned int)phost->device.CfgDesc.bNumInterfaces);
  for (uint8_t i = 0; i < phost->device.CfgDesc.bNumInterfaces; ++i) {
      printf("  interface %u: class=0x%02X sub=0x%02X proto=0x%02X eps=%u\r\n",
             (unsigned int)i,
             (unsigned int)phost->device.CfgDesc.Itf_Desc[i].bInterfaceClass,
             (unsigned int)phost->device.CfgDesc.Itf_Desc[i].bInterfaceSubClass,
             (unsigned int)phost->device.CfgDesc.Itf_Desc[i].bInterfaceProtocol,
             (unsigned int)phost->device.CfgDesc.Itf_Desc[i].bNumEndpoints);
  }
  break;

  case HOST_USER_DISCONNECTION:
  printf("USB device disconnected (gState=%d).\r\n", (int)phost->gState);
  Appli_state = APPLICATION_DISCONNECT;
  fsr_connected = false;
  cdc_rx_armed = false;
  fsr_linecoding_pending = false;
  break;

  case HOST_USER_CLASS_ACTIVE:
  printf("USB CDC class active. Set 460800 8N1 for Moxian sensor...\r\n");
  Appli_state = APPLICATION_READY;
  moxian_linecoding.b.dwDTERate = 460800U;
  moxian_linecoding.b.bCharFormat = 0U;  /* 1 stop bit */
  moxian_linecoding.b.bParityType = 0U;  /* none */
  moxian_linecoding.b.bDataBits = 8U;
  fsr_connected = false;
  cdc_rx_armed = false;
  fsr_linecoding_pending = true;
  fsr_linecoding_tick = HAL_GetTick();
  if (USBH_CDC_SetLineCoding(phost, &moxian_linecoding) != USBH_OK) {
      fsr_linecoding_pending = false;
      printf("[USB CDC] WARN: set line coding failed, start anyway\r\n");
      USBH_CDC_StartMoxian(phost);
  }
  break;
  case HOST_USER_CONNECTION:
  printf("USB device connected (gState=%d).\r\n", (int)phost->gState);
  Appli_state = APPLICATION_START;
  break;

  default:
  printf("USB event id=%d\r\n", (int)id);
  break;
  }
  /* USER CODE END CALL_BACK_1 */
}

/* USER CODE BEGIN 2 */

/**
 * @brief  CDC receive callback â€?called when sensor sends data.
 */
void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *phost)
{
    cdc_rx_armed = false;
    if (fsr_connected) {
        uint16_t len = USBH_CDC_GetLastReceivedDataSize(phost);
        if (len > 0 && len <= CDC_RX_BUF_SIZE) {
            {
                static uint32_t rx_call_count = 0;
                rx_call_count++;
                if (rx_call_count <= 5 || (rx_call_count % 200) == 0) {
                    printf("[USB CDC] RX#%lu: %u bytes\r\n", rx_call_count, len);
                }
            }
            FSR_ProcessUSBData(cdc_rx_buf, len);
        }
        USBH_CDC_TryArmReceive(phost);
    }
}

/**
 * @brief  CDC line coding changed callback â€?fires AFTER SetLineCoding
 *         completes and CDC state returns to IDLE.
 *         This is the safe point to send start-capture and arm RX.
 */
void USBH_CDC_LineCodingChanged(USBH_HandleTypeDef *phost)
{
    if (!fsr_linecoding_pending && fsr_connected) {
        return;
    }
    fsr_linecoding_pending = false;
    printf("[USB CDC] line coding OK: 460800 8N1\r\n");
    USBH_CDC_StartMoxian(phost);
}

/**
 * @brief  CDC transmit callback â€?called when TX completes.
 */
void USBH_CDC_TransmitCallback(USBH_HandleTypeDef *phost)
{
    (void)phost;
    printf("[USB CDC] TX done (start-capture sent)\r\n");
}

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

