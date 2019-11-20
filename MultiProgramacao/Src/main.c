/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
uint8_t sensorA[10];
uint8_t sensorB[10];
uint8_t resultadoA[3];
uint8_t resultadoB[3];
int posicao;

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
UART_HandleTypeDef huart2;

osThreadId taskLeituraHandle;
osThreadId taskMinHandle;
osThreadId taskMaxHandle;
osThreadId taskMedHandle;
osThreadId taskEscritaHandle;
osMutexId mtxSensoresHandle;
osMutexId mtxResultadoHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartTaskLeitura(void const * argument);
void StartTaskMin(void const * argument);
void StartTask03(void const * argument);
void StartTaskMed(void const * argument);
void StartTaskEscrita(void const * argument);

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
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Create the mutex(es) */
	/* definition and creation of mtxSensores */
	osMutexDef(mtxSensores);
	mtxSensoresHandle = osMutexCreate(osMutex(mtxSensores));

	/* definition and creation of mtxResultado */
	osMutexDef(mtxResultado);
	mtxResultadoHandle = osMutexCreate(osMutex(mtxResultado));

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of taskLeitura */
	osThreadDef(taskLeitura, StartTaskLeitura, osPriorityNormal, 0, 128);
	taskLeituraHandle = osThreadCreate(osThread(taskLeitura), NULL);

	/* definition and creation of taskMin */
	osThreadDef(taskMin, StartTaskMin, osPriorityIdle, 0, 128);
	taskMinHandle = osThreadCreate(osThread(taskMin), NULL);

	/* definition and creation of taskMax */
	osThreadDef(taskMax, StartTask03, osPriorityIdle, 0, 128);
	taskMaxHandle = osThreadCreate(osThread(taskMax), NULL);

	/* definition and creation of taskMed */
	osThreadDef(taskMed, StartTaskMed, osPriorityIdle, 0, 128);
	taskMedHandle = osThreadCreate(osThread(taskMed), NULL);

	/* definition and creation of taskEscrita */
	osThreadDef(taskEscrita, StartTaskEscrita, osPriorityIdle, 0, 128);
	taskEscritaHandle = osThreadCreate(osThread(taskEscrita), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

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

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 38400;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTaskLeitura */
uint8_t buffer_test[256];
/**
 * @brief  Function implementing the taskLeitura thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskLeitura */
void StartTaskLeitura(void const * argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {

		uint8_t senA[10];
		uint8_t senB[10];

		senA[0] = 2;
		senB[0] = 3;

		osMutexWait(mtxSensoresHandle, 1000);
		for(int i = 0; i < 10; i++){
			senA[i] = senA[i+1];
			senB[i] = senB[i+1];

			posicao = i;

			sensorA[posicao] = senA[i];
			sensorB[posicao] = senB[i];
		}
		osMutexRelease(mtxSensoresHandle);
	}

	osDelay(5000);
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTaskMin */
/**
 * @brief Function implementing the taskMin thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskMin */
void StartTaskMin(void const * argument) {
	/* USER CODE BEGIN StartTaskMin */
	uint8_t minA;
	uint8_t minB;
	uint8_t senALocal[10];
	uint8_t senBLocal[10];

	/* Infinite loop */
	for (;;) {

		osMutexWait(mtxSensoresHandle, 1000);
			for (int i = 0; i < 10; i++){
				senALocal[i] = sensorA[posicao];
				senBLocal[i] = sensorB[posicao];
				posicao++;
			}
		osMutexRelease(mtxSensoresHandle);

		for (int i = 0; i < 10; i++) {
			if (senALocal[i] < minA) {
				minA = senALocal[i];
			}
			if (senBLocal[i] < minB) {
				minB = senBLocal[i];
			}
		}

		osMutexWait(mtxResultadoHandle, 1000);
		resultadoA[0] = minA;
		resultadoB[0] = minB;
		osMutexRelease(mtxResultadoHandle);

	}
	/* USER CODE END StartTaskMin */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Function implementing the taskMax thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument) {
	/* USER CODE BEGIN StartTask03 */
	uint8_t maxA;
	uint8_t maxB;
	uint8_t senALocal[10];
	uint8_t senBLocal[10];

	/* Infinite loop */
	for (;;) {
		osMutexWait(mtxSensoresHandle, 1000);
			for (int i = 0; i < 10; i++){
				senALocal[i] = sensorA[posicao];
				senBLocal[i] = sensorB[posicao];
				posicao++;
			}
		osMutexRelease(mtxSensoresHandle);

		for (int i = 0; i < 10; i++) {
			if (senALocal[i] < maxA) {
				maxA = senALocal[i];
			}
			if (senBLocal[i] < maxB) {
				maxB = senBLocal[i];
			}
		}

		osMutexWait(mtxResultadoHandle, 1000);
		resultadoA[1] = maxA;
		resultadoB[1] = maxB;
		osMutexRelease(mtxResultadoHandle);
	}
	/* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTaskMed */
/**
 * @brief Function implementing the taskMed thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskMed */
void StartTaskMed(void const * argument) {
	/* USER CODE BEGIN StartTaskMed */
	uint8_t medA;
	uint8_t medB;
	uint8_t senALocal[10];
	uint8_t senBLocal[10];

	/* Infinite loop */
	for (;;) {
		osMutexWait(mtxSensoresHandle, 1000);
		for (int i = 0; i < 10; i++){
			senALocal[i] = sensorA[posicao];
			senBLocal[i] = sensorB[posicao];
			posicao++;
		}
		osMutexRelease(mtxSensoresHandle);

		for (int i = 0; i < 10; i++) {
			medA = +senALocal[i];
			medB = +senBLocal[i];
		}
		medA = medA / 10;
		medB = medB / 10;

		osMutexWait(mtxResultadoHandle, 1000);
		resultadoA[2] = medA;
		resultadoB[2] = medB;
		osMutexRelease(mtxResultadoHandle);
	}
	/* USER CODE END StartTaskMed */
}

/* USER CODE BEGIN Header_StartTaskEscrita */
/**
 * @brief Function implementing the taskEscrita thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskEscrita */
void StartTaskEscrita(void const * argument) {
	/* USER CODE BEGIN StartTaskEscrita */
	char bufferA[1000];
	char bufferB[1000];
	int LocalA[3];
	int LocalB[3];
	/* Infinite loop */
	for (;;) {

		osMutexWait(mtxResultadoHandle, 1000);
		LocalA[0] = resultadoA[0];
		LocalA[1] = resultadoA[1];
		LocalA[2] = resultadoA[2];

		LocalB[0] = resultadoB[0];
		LocalB[1] = resultadoB[1];
		LocalB[2] = resultadoB[2];
		osMutexRelease(mtxResultadoHandle);

		sprintf(bufferA, "Sensor A:\r\n Min: %d\r\n Max: %d\r\n Med: %d\r\n", LocalA[0], LocalA[1], LocalA[2]);

		HAL_UART_Transmit(&huart2, (uint8_t*)bufferA, 1000, 1000);

		sprintf(bufferB, "Sensor B:\r\n Min: %d\r\n Max: %d\r\n Med: %d\r\n", LocalB[0], LocalB[1], LocalB[2]);

		HAL_UART_Transmit(&huart2, (uint8_t*)bufferB, 1000, 1000);


	}
	/* USER CODE END StartTaskEscrita */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
