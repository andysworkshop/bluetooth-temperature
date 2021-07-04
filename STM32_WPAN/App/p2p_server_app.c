/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : App/p2p_server_app.c
 * Description        : P2P Server Application
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "p2p_server_app.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
  uint8_t connected;
  uint8_t notificationEnabled;
  volatile uint16_t dmaBuffer[4];
  float readings[4];
} AppContext;

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim2;

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

static AppContext appContext;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

static void StartTimedAdcConversion();
static void StopTimedAdcConversion();
static float GetTemperature(uint32_t adcReading, uint32_t resistorValue);
static void SendTemperatureNotification();

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void P2PS_STM_App_Notification(P2PS_STM_App_Notification_evt_t *pNotification) {
  /* USER CODE BEGIN P2PS_STM_App_Notification_1 */

  /* USER CODE END P2PS_STM_App_Notification_1 */
  switch (pNotification->P2P_Evt_Opcode) {
  /* USER CODE BEGIN P2PS_STM_App_Notification_P2P_Evt_Opcode */

  /* USER CODE END P2PS_STM_App_Notification_P2P_Evt_Opcode */

  case P2PS_STM__NOTIFY_ENABLED_EVT:
    /* USER CODE BEGIN P2PS_STM__NOTIFY_ENABLED_EVT */
    appContext.notificationEnabled = TRUE;
    /* USER CODE END P2PS_STM__NOTIFY_ENABLED_EVT */
    break;

  case P2PS_STM_NOTIFY_DISABLED_EVT:
    /* USER CODE BEGIN P2PS_STM_NOTIFY_DISABLED_EVT */
    appContext.notificationEnabled = FALSE;
    /* USER CODE END P2PS_STM_NOTIFY_DISABLED_EVT */
    break;

  case P2PS_STM_WRITE_EVT:
    /* USER CODE BEGIN P2PS_STM_WRITE_EVT */

    /* USER CODE END P2PS_STM_WRITE_EVT */
    break;

  default:
    /* USER CODE BEGIN P2PS_STM_App_Notification_default */

    /* USER CODE END P2PS_STM_App_Notification_default */
    break;
  }
  /* USER CODE BEGIN P2PS_STM_App_Notification_2 */

  /* USER CODE END P2PS_STM_App_Notification_2 */
  return;
}

void P2PS_APP_Notification(P2PS_APP_ConnHandle_Not_evt_t *pNotification) {
  /* USER CODE BEGIN P2PS_APP_Notification_1 */

  /* USER CODE END P2PS_APP_Notification_1 */
  switch (pNotification->P2P_Evt_Opcode) {
  /* USER CODE BEGIN P2PS_APP_Notification_P2P_Evt_Opcode */

  /* USER CODE END P2PS_APP_Notification_P2P_Evt_Opcode */
  case PEER_CONN_HANDLE_EVT:
    /* USER CODE BEGIN PEER_CONN_HANDLE_EVT */
    StartTimedAdcConversion();
    /* USER CODE END PEER_CONN_HANDLE_EVT */
    break;

  case PEER_DISCON_HANDLE_EVT:
    /* USER CODE BEGIN PEER_DISCON_HANDLE_EVT */
    StopTimedAdcConversion();
    /* USER CODE END PEER_DISCON_HANDLE_EVT */
    break;

  default:
    /* USER CODE BEGIN P2PS_APP_Notification_default */

    /* USER CODE END P2PS_APP_Notification_default */
    break;
  }
  /* USER CODE BEGIN P2PS_APP_Notification_2 */

  /* USER CODE END P2PS_APP_Notification_2 */
  return;
}

void P2PS_APP_Init(void) {
  /* USER CODE BEGIN P2PS_APP_Init */

  // initialise the application context
  memset(&appContext, 0, sizeof(appContext));

  // create a task for sending the notification
  UTIL_SEQ_RegTask(1 << CFG_TASK_SEND_TEMPERATURE_NOTIFICATION, UTIL_SEQ_RFU, SendTemperatureNotification);

  /* USER CODE END P2PS_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/
/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/*
 * Start the 1Hz ADC conversion of the 4 thermistor channels
 */

static void StartTimedAdcConversion() {

  // just in case

  StopTimedAdcConversion();

  // start the timer

  if (!appContext.connected) {

    appContext.connected = TRUE;

    HAL_ADC_Start_DMA(&hadc1, (uint32_t*) appContext.dmaBuffer, 4);
    HAL_TIM_Base_Start_IT(&htim2);
  }
}

/*
 * Stop the 1Hz ADC conversion events
 */

static void StopTimedAdcConversion() {

  // stop only if actually started

  if (appContext.connected) {

    HAL_TIM_Base_Stop_IT(&htim2);
    HAL_ADC_Stop_DMA(&hadc1);

    appContext.connected = FALSE;
  }
}

/*
 * ADC DMA transfer complete
 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

  // read the thermistors from the ADC and convert to temperature in Celsius

  appContext.readings[0] = GetTemperature(appContext.dmaBuffer[0], 10005);
  appContext.readings[1] = GetTemperature(appContext.dmaBuffer[1], 10005);
  appContext.readings[2] = GetTemperature(appContext.dmaBuffer[2], 9977);
  appContext.readings[3] = GetTemperature(appContext.dmaBuffer[3], 10010);

  // schedule a task to send the notification immediately

  UTIL_SEQ_SetTask(1 << CFG_TASK_SEND_TEMPERATURE_NOTIFICATION, CFG_SCH_PRIO_0);
}

static float GetTemperature(uint32_t adcReading, uint32_t resistorValue) {

  // get the voltage across the thermocouple

  float voltage = 1.8 * (float) adcReading / 4096.0;

  // calculate resistance of thermistor

  float rt = ((1.8 * resistorValue) / voltage) - resistorValue; //(voltage * resistorValue) / (1.8 - voltage);

  // convert the resistance to a temperature

  float t = (1 / (0.003354016 + (0.000295858 * log(rt / 10000)))) - 273.15;
  return t;
}

/*
 * Send a temperature notification
 */

static void SendTemperatureNotification() {

  // can't do anything unless connected and notification is enabled

  if (!appContext.connected || !appContext.notificationEnabled) {
    return;
  }

  P2PS_STM_App_Update_Char(P2P_NOTIFY_CHAR_UUID, pPayload);
}

/* USER CODE END FD_LOCAL_FUNCTIONS*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
