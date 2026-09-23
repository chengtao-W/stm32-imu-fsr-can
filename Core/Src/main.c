   /* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_fdcan.h"
#include "delay.h"
#include <stdio.h> /* printf */
#include <string.h> /* strlen */
#include "stdbool.h"
#include <math.h>
// 鏈哄櫒浜洪€氫俊鐩稿叧澶存枃浠?
#include "robot_comm.h"
// IMU 鏁版嵁杈撳嚭
#include "imu.h"
// 瓒冲簳鍘嬪姏浼犳劅鍣?
#include "fsr.h"
// 璋冭瘯鎺ュ彛
#include "debug_if.h"
// 鎺у埗灞傞€氫俊瀵规帴 (閬ユ祴缁勮 + 鍛戒护鎵ц)
#include "exo_telemetry.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t imu_tx_flag = 0;  // ISR 缃綅 鈫?涓诲惊鐜彂閫?IMU JSON
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


// Called from TIM ISR (quick, non-blocking) 鈥?灞忚斀(鍙敤LED娴嬭瘯)
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM4) {
		// 灏嗗綋鍓嶇姸鎬侀€氳繃 USB 鎺ㄩ€佸埌涓绘満
		// 銆愪复鏃剁Щ闄ゃ€慠obot_Push_Feedback();

		// 姣?10 娆?TIM4 涓柇璁剧疆涓€娆?IMU 鏁版嵁鍙戦€佹爣蹇?(绾?100Hz)
		// 瀹為檯鍙戦€佹斁鍦ㄤ富寰幆涓紝閬垮厤鍦?ISR 涓樆濉炶皟鐢?HAL_UART_Transmit
		{
			static uint8_t imu_tx_cnt = 0;
			if (++imu_tx_cnt >= 10) {
				imu_tx_cnt = 0;
				imu_tx_flag = 1;
			}
		}
	}
}
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

  /* Initialize all configured peripherals 鈥?鍙繚鐣?GPIO(LED) */
  MX_GPIO_Init();

  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_TIM4_Init();
  MX_FDCAN2_Init();
  MX_UART7_Init();
  MX_USART10_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USB Host is started after the sensor firmware settle delay below. */
  /* USER CODE BEGIN 2 */
	/* USB OTG 鍒濆鍖栧悗 PA10 鍙兘琚?USB 鍗犵敤, 閲嶆柊閰嶄负 UART1 RX */
	{
		GPIO_InitTypeDef g = {0};
		g.Pin = GPIO_PIN_9 | GPIO_PIN_10;
		g.Mode = GPIO_MODE_AF_PP;
		g.Pull = GPIO_NOPULL;
		g.Speed = GPIO_SPEED_FREQ_LOW;
		g.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA, &g);
	}

	bsp_fdcan_set_baud(&hfdcan1, CAN_CLASS, CAN_BR_1M);
	bsp_fdcan_set_baud(&hfdcan2, CAN_CLASS, CAN_BR_500K);
	HAL_Delay(1000);
	bsp_can_init();
	hipnuc_imu_config();
	HAL_Delay(200);
	DebugIf_Init();
	FSR_Init();


	printf("\r\n=== CtrBoard Boot OK ===\r\n");
    {
        uint32_t isr = huart1.Instance->ISR;
        uint32_t cr1 = huart1.Instance->CR1;
        printf("[UART] ISR=0x%08lX CR1=0x%08lX RE=%d\r\n",
               isr, cr1, (cr1 & USART_CR1_RE) ? 1 : 0);
    }
	Robot_Comm_Init();
	ExoTelemetry_Init();

  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */


  /* ESP32-S3 sensor may expose its bootloader before the application CDC interface. */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  printf("[USB] Waiting 5000 ms for sensor firmware...\r\n");
  HAL_Delay(5000U);
  printf("[USB] Starting USB enumeration.\r\n");
  MX_USB_HOST_Init();

  /* Infinite loop 鈥?鐢?LED 闂儊鑺傚鎸囩ず BOOT_ADD0 */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    IMU_Poll_RX();
    FSR_DetectGaitPhase();

    if (imu_tx_flag) {
        imu_tx_flag = 0;
        DetectPosture();
        /* Publish IMU and foot-pressure telemetry over CAN1 at 10 Hz. */
        ExoTelemetry_Push();
    }
    Robot_Flush_TxBuffer();
    MX_USB_HOST_Process();
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
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





