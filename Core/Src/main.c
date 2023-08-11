/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void AngularSpeedToTimerPWMParametrs(uint16_t w); //w - angular speed = 0.01 radian per second 1
/*
 * Limits for w  (10<w<100)
 */
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  //Config via pins see p.116 of TMC5160 datasheet
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 1); //DRV SLEEP 0 for power on, 1 for power off

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET); //SD_MODE ON
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); //SPI_MODE OFF
  HAL_Delay(100); //Sanity delay

  //64 microsteps, MRES=5
  HAL_GPIO_WritePin(CFG0_GPIO_Port, CFG0_Pin, GPIO_PIN_SET); //CFG0
  HAL_GPIO_WritePin(CFG1_GPIO_Port, CFG1_Pin, GPIO_PIN_SET); //CFG1
  /*
  CFG1   CFG0    Microstep Setting
  GND    GND     8 microsteps, MRES=5
  GND    VCC_IO  16 microsteps, MRES=4
  VCC_IO GND     32 microsteps, MRES=3
  VCC_IO VCC_IO  64 microsteps, MRES=2
  */

  //IRUN=16
  HAL_GPIO_WritePin(CFG2_GPIO_Port, CFG2_Pin, GPIO_PIN_RESET); //CFG2
  HAL_GPIO_WritePin(CFG3_GPIO_Port, CFG3_Pin, GPIO_PIN_RESET); //CFG3
  HAL_GPIO_WritePin(CFG4_GPIO_Port, CFG4_Pin, GPIO_PIN_RESET); //CFG4
  /*
   CFG4/CFG3/CFG2: CONFIGURATION OF RUN CURRENT
	CFG4   CFG3     CFG2   IRUN Setting
	GND     GND     GND    IRUN=16
	GND     GND     VCC_IO IRUN=18
	GND     VCC_IO  GND    IRUN=20
	GND     VCC_IO  VCC_IO IRUN=22
	VCC_IO  GND     GND    IRUN=24
	VCC_IO  GND     VCC_IO IRUN=26
	VCC_IO  VCC_IO  GND    IRUN=28
	VCC_IO  VCC_IO  VCC_IO IRUN=31
   */

  //StealthChop operation
  HAL_GPIO_WritePin(CFG5_GPIO_Port, CFG5_Pin, GPIO_PIN_RESET); //CFG5
  /*
   CFG5: SELECTION OF CHOPPER MODE
   CFG5 Chopper Setting
   GND SpreadCycle operation. (TOFF=3)
   VCC_IO StealthChop operation. (GCONF.en_PWM_mode=1)
   */


  //IHOLD Reduction to 50%. IHOLD=1/2 IRUN
  HAL_GPIO_WritePin(CFG6_GPIO_Port, CFG6_Pin, GPIO_PIN_SET); //CFG6
  /*
  CFG6: CONFIGURATION OF HOLD CURRENT REDUCTION
	CFG6 	Chopper Setting
	GND 	No hold current reduction. IHOLD=IRUN
	VCC_IO 	Reduction to 50%. IHOLD=1/2 IRUN
  */



  HAL_Delay(1000); //sanity delay

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, 0); //DIR
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, 0); //DRV Enable 0 for power on, 1 for power off
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //Start timer
  HAL_TIM_Base_Start(&htim1);
  //Start timer PWM
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  //setting the speed of rotation
  uint8_t speed = 100; //  * 0.01 rad / sec
  //timer settings
  AngularSpeedToTimerPWMParametrs(speed);

  while (1)
  {
	  uint8_t str[50];
	  sprintf(str,"Rotating %d * 0.01 rad / sec \r\n\0", speed);
	  HAL_UART_Transmit_IT(&huart2, str, sizeof(str));
	  HAL_Delay(100);
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void AngularSpeedToTimerPWMParametrs(uint16_t w)
{

int step_duration = 0;
step_duration = (int)((100 / w) * 100); //100 total duration of UP + DOWN of STEP
TIM1->PSC = 15;
TIM1->ARR = step_duration;
TIM1->CCR1 = 40; // 40 is min duration of UP of STEP
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
