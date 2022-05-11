/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"
#include "dma.h"
#include "iwdg.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MQTTSim800.h"
#include <stdlib.h>
#include <string.h>
#include "SIM800c.h"

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
SIM800_t SIM800;
uint8_t but1 = 0;
uint8_t but2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == UART_SIM800) {
        Sim800_RxCallBack(Size);
    }
}
uint8_t buf[256];
uint8_t* host = "space.arbina.com";
uint8_t* user = "jackson";
uint8_t* pass = "Amymezyry4235";
HAL_StatusTypeDef state ;
uint32_t tickMsWaiting;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
    MX_IWDG_Init();
    if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
    {
        /* IWDGRST flag set: Turn LED1 on */

        /* Clear reset flags */
        __HAL_RCC_CLEAR_RESET_FLAGS();
    }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  HAL_Delay(5000);
    SIM800.sim.apn = "internet.mts.ru";
    SIM800.sim.apn_user = "";
    SIM800.sim.apn_pass = "";
    memcpy(SIM800.mqttServer.host, host, strlen(host) );
    SIM800.mqttServer.port = 1883;
    memcpy(SIM800.mqttClient.username, user, strlen(user) );
    memcpy(SIM800.mqttClient.pass, pass, strlen(pass) );
    SIM800.mqttClient.clientID = "TestSub";
    SIM800.mqttClient.keepAliveInterval = 120;
    MQTT_Init();
    uint8_t sub = 0;

    //Test data
    uint8_t pub_uint8 = 1;
    uint16_t pub_uint16 = 2;
    uint32_t pub_uint32 = 3;
    float pub_float = 1.1;
    double pub_double = 2.2;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 0)
      {
          if ( HAL_GetTick() - SIM800.RI_time_ms > 150)
          {
              if (SIM800.mqttServer.connect == 1)
              {
                  SIM800.mqttServer.connect = 0;
                  SIM800_SendCommand("+++", "OK", 2000);
                  while (SIM800.answer == 0)
                  {

                  }

              }

              SIM800_SendCommand("ATH\r\n", "", 2000);
              SIM800_SendCommand("ATO\r\n", "", 2000);
              SIM800.mqttServer.connect = 1;
          }
      }
      if (SIM800.newSMS == 1)
      {
          if (SIM800.mqttServer.connect == 1)
          {
              SIM800.mqttServer.connect = 0;
              SIM800_SendCommand("+++", "OK", 2000);
              while (SIM800.answer == 0)
              {

              }
          }
          uint8_t oldNUm = SIM800.smsNumber;
          uint8_t num = found_sms();
          if (oldNUm != 0 && (oldNUm != num && num != 0))
          {
              HAL_NVIC_SystemReset();
          }
          SIM800_SendCommand("ATH\r\n", "", 2000);
          SIM800_SendCommand("ATO\r\n", "", 2000);
          SIM800.mqttServer.connect = 1;
          SIM800.newSMS = 0;


      }


      if (SIM800.mqttServer.connect == 0) {
          MQTT_Init();
          sub = 0;
      }
      if (SIM800.mqttServer.connect == 1) {
          if(sub == 0){
              MQTT_Sub("homeassistant/gate2/status");
              MQTT_Sub("homeassistant/gate2/key1");
              MQTT_Sub("homeassistant/gate2/key2");
              HAL_IWDG_Refresh(&hiwdg);
              sub = 1;
          }





          if(SIM800.mqttReceive.newEvent)
          {
              HAL_IWDG_Refresh(&hiwdg);
              uint8_t *payload = SIM800.mqttReceive.payload;
              if (strstr(SIM800.mqttReceive.topic, "key1") != NULL)
              {
                  if (strstr(payload, "ON") != NULL)
                  {
                      HAL_GPIO_WritePin(BUTTON1_GPIO_Port, BUTTON1_Pin, GPIO_PIN_SET);
                      MQTT_Pub("homeassistant/gate2/key1", "push");
                      HAL_Delay(1000);
                      MQTT_Pub("homeassistant/gate2/key1", "unpush");
                      HAL_GPIO_WritePin(BUTTON1_GPIO_Port, BUTTON1_Pin, GPIO_PIN_RESET);
                  }

              }
              else if (strstr(SIM800.mqttReceive.topic, "key2") != NULL)
              {
                  if (strstr(payload, "ON") != NULL)
                  {
                      HAL_GPIO_WritePin(BUTTON2_GPIO_Port, BUTTON2_Pin, GPIO_PIN_SET);
                      MQTT_Pub("homeassistant/gate2/key2", "push");
                      HAL_Delay(1000);
                      HAL_GPIO_WritePin(BUTTON2_GPIO_Port, BUTTON2_Pin, GPIO_PIN_RESET);
                      MQTT_Pub("homeassistant/gate2/key2", "unpush");

                  }

              }
              else if (strstr(SIM800.mqttReceive.topic, "status") != NULL)
              {

              }
              SIM800.mqttReceive.newEvent = 0;
          }
          uint32_t tickMs = HAL_GetTick();
          if (tickMs - tickMsWaiting > 2000)
          {
              tickMsWaiting = tickMs;
              MQTT_Pub("homeassistant/gate2/status", "online");
          }
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

