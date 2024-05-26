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
#include "cmsis_os.h"

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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for ledVerde */
osThreadId_t ledVerdeHandle;
const osThreadAttr_t ledVerde_attributes = { .name = "ledVerde", .stack_size =
		128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for ledAmarillo */
osThreadId_t ledAmarilloHandle;
const osThreadAttr_t ledAmarillo_attributes = { .name = "ledAmarillo",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for control_leds */
osThreadId_t control_ledsHandle;
const osThreadAttr_t control_leds_attributes =
		{ .name = "control_leds", .stack_size = 128 * 4, .priority =
				(osPriority_t) osPriorityBelowNormal, };
/* Definitions for respuesta */
osThreadId_t respuestaHandle;
const osThreadAttr_t respuesta_attributes = { .name = "respuesta", .stack_size =
		128 * 4, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for num_pulsa */
osMessageQueueId_t num_pulsaHandle;
const osMessageQueueAttr_t num_pulsa_attributes = { .name = "num_pulsa" };
/* Definitions for semaverde */
osSemaphoreId_t semaverdeHandle;
const osSemaphoreAttr_t semaverde_attributes = { .name = "semaverde" };
/* Definitions for semarillo */
osSemaphoreId_t semarilloHandle;
const osSemaphoreAttr_t semarillo_attributes = { .name = "semarillo" };
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);
void Start_led_verde(void *argument);
void Start_led_amarillo(void *argument);
void Start_control_leds(void *argument);
void StartTask05(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of semaverde */
	semaverdeHandle = osSemaphoreNew(5, 0, &semaverde_attributes);

	/* creation of semarillo */
	semarilloHandle = osSemaphoreNew(5, 0, &semarillo_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of num_pulsa */
	num_pulsaHandle = osMessageQueueNew(3, sizeof(uint16_t),
			&num_pulsa_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of ledVerde */
	ledVerdeHandle = osThreadNew(Start_led_verde, NULL, &ledVerde_attributes);

	/* creation of ledAmarillo */
	ledAmarilloHandle = osThreadNew(Start_led_amarillo, NULL,
			&ledAmarillo_attributes);

	/* creation of control_leds */
	control_ledsHandle = osThreadNew(Start_control_leds, NULL,
			&control_leds_attributes);

	/* creation of respuesta */
	respuestaHandle = osThreadNew(StartTask05, NULL, &respuesta_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the Systick interrupt time
	 */
	__HAL_RCC_PLLI2S_ENABLE();
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, VERDE_Pin | AMARILLO_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : TAMPER_Pin */
	GPIO_InitStruct.Pin = TAMPER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(TAMPER_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : WKUP_Pin */
	GPIO_InitStruct.Pin = WKUP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(WKUP_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : VERDE_Pin AMARILLO_Pin */
	GPIO_InitStruct.Pin = VERDE_Pin | AMARILLO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */

	for (;;) {

		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_led_verde */
/**
 * @brief Function implementing the ledVerde thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_led_verde */
void Start_led_verde(void *argument) {
	/* USER CODE BEGIN Start_led_verde */
	/* Infinite loop */
	for (;;) {
		osSemaphoreAcquire(semaverdeHandle, 0xFFFFFFF);
		HAL_GPIO_WritePin(VERDE_GPIO_Port, VERDE_Pin, GPIO_PIN_SET);
		osDelay(300);
		HAL_GPIO_WritePin(VERDE_GPIO_Port, VERDE_Pin, GPIO_PIN_RESET);
		osDelay(300);
	}
	/* USER CODE END Start_led_verde */
}

/* USER CODE BEGIN Header_Start_led_amarillo */
/**
 * @brief Function implementing the ledAmarillo thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_led_amarillo */
void Start_led_amarillo(void *argument) {
	/* USER CODE BEGIN Start_led_amarillo */
	/* Infinite loop */
	for (;;) {
		osSemaphoreAcquire(semarilloHandle, 0xFFFFFFF);
		HAL_GPIO_WritePin(AMARILLO_GPIO_Port, AMARILLO_Pin, GPIO_PIN_SET);
		osDelay(200);
		HAL_GPIO_WritePin(AMARILLO_GPIO_Port, AMARILLO_Pin, GPIO_PIN_RESET);
		osDelay(200);
	}
	/* USER CODE END Start_led_amarillo */
}

/* USER CODE BEGIN Header_Start_control_leds */
/**
 * @brief Function implementing the control_leds thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Start_control_leds */
void Start_control_leds(void *argument) {
	/* USER CODE BEGIN Start_control_leds */
	/* Infinite loop */

	unsigned short respuesta;
	//mientras no se plse el tamper, esperamos de forma activa
	while (HAL_GPIO_ReadPin(TAMPER_GPIO_Port, TAMPER_Pin) == 0) {
		osDelay(101);
	}
		osMessageQueueReset(num_pulsaHandle);
		for (;;) {
			osMessageQueueGet(num_pulsaHandle, &respuesta, 0, 0xFFFFFFFF);
			if (respuesta % 2 == 0) { // es par
				HAL_GPIO_WritePin(AMARILLO_GPIO_Port, AMARILLO_Pin, 0);
				HAL_GPIO_WritePin(VERDE_GPIO_Port, VERDE_Pin, 1);
			} else {
				HAL_GPIO_WritePin(AMARILLO_GPIO_Port, AMARILLO_Pin, 1);
				HAL_GPIO_WritePin(VERDE_GPIO_Port, VERDE_Pin, 0);
			}

			osDelay(2000);
			HAL_GPIO_WritePin(AMARILLO_GPIO_Port, AMARILLO_Pin, 0);
			HAL_GPIO_WritePin(VERDE_GPIO_Port, VERDE_Pin, 0);
			osDelay(10); // eliminar en la fase 4
		}

	/* USER CODE END Start_control_leds */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
 * @brief Function implementing the respuesta thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument) {
	/* USER CODE BEGIN StartTask05 */
	/* Infinite loop */
	unsigned short conta2;
	for (;;) {
		if (HAL_GPIO_ReadPin(GPIOA, WKUP_Pin) == 1) {
			conta2++;
			while (HAL_GPIO_ReadPin(GPIOA, WKUP_Pin) == 1) {
				//hasta que no se libere no salimos
				osDelay(10);
			}
		}
		// metemos en la cola
		if (HAL_GPIO_ReadPin(GPIOC, TAMPER_Pin) == 0 && conta2 != 0) {
			osMessageQueuePut(num_pulsaHandle, &conta2, 0, 0);
			conta2 = 0;
		}

		osDelay(10);
	}
	/* USER CODE END StartTask05 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
