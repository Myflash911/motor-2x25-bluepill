/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/*
 * author Boris Pavlenko <bor-pavlenko@yandex.ru>
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "crc.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "DriverMonitor.h"
#include "MotorUnit.h"
#include "SabertoothSimplified.h"
#include "XPIDController.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define RELEASE_LIMIT_CYCLES 	5
#define PID_DT					10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

DriverMonitor monitor;

SabertoothSimplified ST(driver_port_write);

XPIDController PID_L, PID_R;

MotorUnit leftMotorUnit(eLeftMotor, &PID_L, &ST, HAL_GetTick);
MotorUnit rightMotorUnit(eRightMotor, &PID_R, &ST, HAL_GetTick);

//uint32_t last_LSW_tick = 0; 		// Last Limit Switch hit tick
uint8_t L_BWD_lmt, L_FWD_lmt, R_BWD_lmt, R_FWD_lmt;
char _lmt_str[] = "LMT1_BWD\r\n\0";
uint32_t prev_tick, now, dt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void RefreshLimitSwitches(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	leftMotorUnit.SetTargetRange(0,  3961);
	rightMotorUnit.SetTargetRange(0, 3961);

	leftMotorUnit.pPosition = &(TIM1->CNT);
	rightMotorUnit.pPosition = &(TIM2->CNT);

//	ST.motor1_inversion = eInvertesionOn;
	monitor.Start(TxBuffer, &ST);
	monitor.pLeftMotorUnit = &leftMotorUnit;
	monitor.pRightMotorUnit = &rightMotorUnit;
	monitor.pPID_L = &PID_L;
	monitor.pPID_R = &PID_R;


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  // На всякий случай при сбросе первым делом посылаем команду СТОП.
  driver_port_write(0);
  driver_port_write(0);
  driver_port_write(0);
  HAL_Delay(50);
  driver_port_write(0);
  HAL_Delay(50);
  driver_port_write(0);

  if (! isLimitSwitchReleased(LMT1_BWD_Pin))
  {
	  leftMotorUnit.HappendEvent(HIT_BWD, OnEvent);
  }
  if (! isLimitSwitchReleased(LMT1_FWD_Pin))
  {
	  leftMotorUnit.HappendEvent(HIT_FWD, OnEvent);
  }
  if (! isLimitSwitchReleased(LMT2_BWD_Pin))
  {
	  rightMotorUnit.HappendEvent(HIT_BWD, OnEvent);
  }
  if (! isLimitSwitchReleased(LMT2_FWD_Pin))
  {
	  rightMotorUnit.HappendEvent(HIT_FWD, OnEvent);
  }

  HAL_Delay(9500);

  SAY("Sybertooth driver %s \r\n", VERSION);
  SAY("Loading settings...\r\n");
  BLINK(); BLINK(); LED_ON();
  if (monitor.Load() != 0)
  {
	  SAY("Loading setting from flash failed!\r\n");
	  monitor.Set_PID_default();
	  SAY("Initilazed defaut settings.\r\n");
  }
  else
  {
	  SAY("Setting loaded successfully.\r\n");
  }

  LED_OFF();

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1 | TIM_CHANNEL_2);

  leftMotorUnit.RequestCalibration();
  rightMotorUnit.RequestCalibration();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  monitor.Parse();
	  monitor.Execute();

	  // calc target
	  if (now != HAL_GetTick())
	  {
		  now = HAL_GetTick();
		  RefreshLimitSwitches();
		  dt = now - prev_tick;
		  if (dt > PID_DT)
		  {
			  leftMotorUnit.CalcPID(dt);
			  rightMotorUnit.CalcPID(dt);
			  prev_tick = now;
		  }
	  }
	  // calc speed
	  leftMotorUnit.GetSpeed();
	  rightMotorUnit.GetSpeed();
	  // speed command
	  leftMotorUnit.Drive();
	  rightMotorUnit.Drive();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * LIMIT SWITCHES CALLBACK
 * Detects if Limit Switch was hitted.
 * Stops motors in normal mode.
 * @param GPIO_Pin
 * Rising EDGE Trigger detection
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == LMT1_BWD_Pin)
	{
		L_BWD_lmt = RELEASE_LIMIT_CYCLES;
		leftMotorUnit.HappendEvent(HIT_BWD, OnEvent);
		_lmt_str[3] = '1';
		_lmt_str[5] = 'B';
	}
	else if (GPIO_Pin == LMT1_FWD_Pin)
	{
		L_FWD_lmt = RELEASE_LIMIT_CYCLES;
		leftMotorUnit.HappendEvent(HIT_FWD, OnEvent);
		_lmt_str[3] = '1';
		_lmt_str[5] = 'F';
	}
	else if (GPIO_Pin == LMT2_BWD_Pin)
	{
		R_BWD_lmt = RELEASE_LIMIT_CYCLES;
		rightMotorUnit.HappendEvent(HIT_BWD, OnEvent);
		_lmt_str[3] = '2';
		_lmt_str[5] = 'B';
	}
	else if (GPIO_Pin == LMT2_FWD_Pin)
	{
		R_FWD_lmt = RELEASE_LIMIT_CYCLES;
		rightMotorUnit.HappendEvent(HIT_FWD, OnEvent);
		_lmt_str[3] = '2';
		_lmt_str[5] = 'F';
	}

	SAY(_lmt_str);
//	monitor.HitLimitSwitch(GPIO_Pin);
}



uint32_t calc_flash_checksum(uint32_t* _data)
{
	return HAL_CRC_Calculate(&hcrc, _data, 255);
}


void RefreshLimitSwitches(void)
{
	if (L_BWD_lmt && isLimitSwitchReleased(LMT1_BWD_Pin))
	{
		L_BWD_lmt--;
		if (L_BWD_lmt == 0)
		{
			leftMotorUnit.HappendEvent(HIT_BWD, OutEvent);
		}
	}
	if (L_FWD_lmt && isLimitSwitchReleased(LMT1_FWD_Pin))
	{
		L_FWD_lmt--;
		if (L_FWD_lmt == 0)
		{
			leftMotorUnit.HappendEvent(HIT_FWD, OutEvent);
		}
	}
	if (R_BWD_lmt && isLimitSwitchReleased(LMT2_BWD_Pin))
	{
		R_BWD_lmt--;
		if (R_BWD_lmt == 0)
		{
			rightMotorUnit.HappendEvent(HIT_BWD, OutEvent);
		}
	}
	if (R_FWD_lmt && isLimitSwitchReleased(LMT2_FWD_Pin))
	{
		R_FWD_lmt--;
		if (R_FWD_lmt == 0)
		{
			rightMotorUnit.HappendEvent(HIT_FWD, OutEvent);
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
