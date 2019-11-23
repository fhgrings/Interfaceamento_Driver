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
#include "stm32f3xx_hal.h"
#include "user_5110.h"
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

osThreadId taskLeituraAcelHandle;
osThreadId taskLeituraSensHandle;
osThreadId taskProcessamenHandle;
osThreadId taskAcionamentoHandle;
osThreadId taskEscritaMemoHandle;
osThreadId taskEscritaDispHandle;
osMutexId MtxAceleradorHandle;
osMutexId MtxQntCombustivelHandle;
osMutexId MtxInformacoesHandle;
osMutexId MtxConstantesHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void StartLeituraAcel(void const * argument);
void StartLeituraSens(void const * argument);
void StartProcessamen(void const * argument);
void StartAcionamento(void const * argument);
void StartEscritaMemoria(void const * argument);
void StartEscritaDisplay(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aceleradorGlobal;
uint8_t sensorOxg_TempGlobal[2];
uint8_t qntCombustivelGlobal;
uint8_t informacoesGlobal[2];
uint8_t constantesGlobal[3];



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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of MtxAcelerador */
  osMutexDef(MtxAcelerador);
  MtxAceleradorHandle = osMutexCreate(osMutex(MtxAcelerador));


  /* definition and creation of MtxQntCombustivel */
  osMutexDef(MtxQntCombustivel);
  MtxQntCombustivelHandle = osMutexCreate(osMutex(MtxQntCombustivel));

  /* definition and creation of MtxInformacoes */
  osMutexDef(MtxInformacoes);
  MtxInformacoesHandle = osMutexCreate(osMutex(MtxInformacoes));

  /* definition and creation of MtxConstantes */
  osMutexDef(MtxConstantes);
  MtxConstantesHandle = osMutexCreate(osMutex(MtxConstantes));

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
  /* definition and creation of taskLeituraAcel */
  osThreadDef(taskLeituraAcel, StartLeituraAcel, osPriorityNormal, 0, 128);
  taskLeituraAcelHandle = osThreadCreate(osThread(taskLeituraAcel), NULL);

  /* definition and creation of taskLeituraSens */
  osThreadDef(taskLeituraSens, StartLeituraSens, osPriorityNormal, 0, 128);
  taskLeituraSensHandle = osThreadCreate(osThread(taskLeituraSens), NULL);

  /* definition and creation of taskProcessamen */
  osThreadDef(taskProcessamen, StartProcessamen, osPriorityNormal, 0, 128);
  taskProcessamenHandle = osThreadCreate(osThread(taskProcessamen), NULL);

  /* definition and creation of taskAcionamento */
  osThreadDef(taskAcionamento, StartAcionamento, osPriorityNormal, 0, 128);
  taskAcionamentoHandle = osThreadCreate(osThread(taskAcionamento), NULL);

  /* definition and creation of taskEscritaMemo */
  osThreadDef(taskEscritaMemo, StartEscritaMemoria, osPriorityNormal, 0, 128);
  taskEscritaMemoHandle = osThreadCreate(osThread(taskEscritaMemo), NULL);

  /* definition and creation of taskEscritaDisp */
  osThreadDef(taskEscritaDisp, StartEscritaDisplay, osPriorityNormal, 0, 128);
  taskEscritaDispHandle = osThreadCreate(osThread(taskEscritaDisp), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//  LCD_Init();
//  LCD_Write_String(0, 0, "Teste");
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_OXIGENIO_Pin|EN_ACELERADOR_Pin|EN_MEMORIA_Pin|LCD_CE_Pin 
                          |LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_3_Pin|LED_2_Pin|LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_1_Pin|EN_TEMPERATURA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_OXIGENIO_Pin EN_ACELERADOR_Pin EN_MEMORIA_Pin LCD_CE_Pin 
                           LCD_DC_Pin */
  GPIO_InitStruct.Pin = EN_OXIGENIO_Pin|EN_ACELERADOR_Pin|EN_MEMORIA_Pin|LCD_CE_Pin 
                          |LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_3_Pin LED_2_Pin LCD_RST_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin|LED_2_Pin|LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin EN_TEMPERATURA_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|EN_TEMPERATURA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLeituraAcel */
/**
  * @brief  Function implementing the taskLeituraAcel thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartLeituraAcel */
void StartLeituraAcel(void const * argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t aceleradorLocal = 0;
	uint8_t transmissao[100];
	uint8_t sensorOxg_TempLocal[2];
  /* Infinite loop */
  for(;;)
  {
	  aceleradorLocal = 0;
	  HAL_GPIO_WritePin(EN_ACELERADOR_GPIO_Port, EN_ACELERADOR_Pin, 0);

	  HAL_SPI_Receive(&hspi1, &aceleradorLocal, 1, 1000);

	  HAL_GPIO_WritePin(EN_ACELERADOR_GPIO_Port, EN_ACELERADOR_Pin, 1);



	  if(aceleradorLocal > 128 ) {
		  sprintf(transmissao, "valor Acelerador lido: %d\r\n", aceleradorLocal);
		  HAL_UART_Transmit(&huart2, transmissao, strlen(transmissao), 1000);
	  }
	  // ===============================================


	  sensorOxg_TempLocal[0] = 0;
	  sensorOxg_TempLocal[1] = 0;

	  HAL_GPIO_WritePin(EN_OXIGENIO_GPIO_Port, EN_OXIGENIO_Pin, 0);
	  HAL_SPI_Receive(&hspi1, &sensorOxg_TempLocal[0], 1, 1000);
	  HAL_GPIO_WritePin(EN_OXIGENIO_GPIO_Port, EN_OXIGENIO_Pin, 1);

	  if(sensorOxg_TempLocal[0] > 128){
		  sprintf (transmissao, "valor Oxigenio lido: %d\r\n", sensorOxg_TempLocal[0]);
		  HAL_UART_Transmit(&huart2, transmissao, strlen(transmissao), 1000);
	  }


	  HAL_GPIO_WritePin(EN_TEMPERATURA_GPIO_Port, EN_TEMPERATURA_Pin, 0);
	  HAL_SPI_Receive(&hspi1, &sensorOxg_TempLocal[1], 1, 1000);
	  HAL_GPIO_WritePin(EN_TEMPERATURA_GPIO_Port, EN_TEMPERATURA_Pin, 1);

	  if(sensorOxg_TempLocal[1] > 128) {
		  sprintf (transmissao, "valor Temperatura lido: %d\r\n", sensorOxg_TempLocal[1]);
		  HAL_UART_Transmit(&huart2, transmissao, strlen(transmissao), 1000);
	  }













	  osMutexWait(MtxAceleradorHandle,1000);

	  aceleradorGlobal = aceleradorLocal;
	  sensorOxg_TempGlobal[0] = sensorOxg_TempLocal[0];
	  sensorOxg_TempGlobal[1] = sensorOxg_TempLocal[1];
	  osMutexRelease(MtxAceleradorHandle);

	  HAL_Delay(10);

  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLeituraSens */
/**
* @brief Function implementing the taskLeituraSens thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLeituraSens */
void StartLeituraSens(void const * argument)
{
  /* USER CODE BEGIN StartLeituraSens */

  /* Infinite loop */
  for(;;)
  {
	  HAL_Delay(10);
  }
  /* USER CODE END StartLeituraSens */
}

/* USER CODE BEGIN Header_StartProcessamen */
/**
* @brief Function implementing the taskProcessamen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartProcessamen */
void StartProcessamen(void const * argument)
{
  /* USER CODE BEGIN StartProcessamen */
	uint8_t aceleracaoLocal;
	uint8_t temperaturaLocal;
	uint8_t oxigenioLocal;
	uint8_t qntCombustivelLocal;
	uint8_t constantesLocal[3];

  /* Infinite loop */
  for(;;)
  {
	osMutexWait(MtxAceleradorHandle, 1000);
	aceleracaoLocal = aceleradorGlobal;
	oxigenioLocal = sensorOxg_TempGlobal[0];
	temperaturaLocal = sensorOxg_TempGlobal[1];
	osMutexRelease(MtxAceleradorHandle);





//	osMutexWait(MtxConstantesHandle, 1000);
	constantesLocal[0] = constantesGlobal[0];
	constantesLocal[1] = constantesGlobal[1];
	constantesLocal[2] = constantesGlobal[2];
//	osMutexRelease(MtxConstantesHandle);




	qntCombustivelLocal = aceleracaoLocal * 50 + (30 - temperaturaLocal * 30) + oxigenioLocal * 20;


	osMutexWait(MtxQntCombustivelHandle, 1000);
	qntCombustivelGlobal = qntCombustivelLocal;
	osMutexRelease(MtxQntCombustivelHandle);


	osMutexWait(MtxInformacoesHandle, 1000);
	informacoesGlobal[0] = qntCombustivelGlobal;
	informacoesGlobal[1] = 222;
	osMutexRelease(MtxInformacoesHandle);

	osDelay(10);

  }
  /* USER CODE END StartProcessamen */
}

/* USER CODE BEGIN Header_StartAcionamento */
/**
* @brief Function implementing the taskAcionamento thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAcionamento */
void StartAcionamento(void const * argument)
{
  /* USER CODE BEGIN StartAcionamento */
	uint8_t qntCombustivelLocal;
  /* Infinite loop */
  for(;;)
  {
	osMutexWait(MtxQntCombustivelHandle, 1000);
	qntCombustivelLocal = qntCombustivelGlobal;
	osMutexRelease(MtxQntCombustivelHandle);

	if(qntCombustivelLocal < 33) {
	  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, 1);
	  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, 0);
	  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, 0);
//	  HAL_UART_Transmit(&huart2, "taskAcionamento 33\r\n", 33, 1000);
	}

	if(qntCombustivelLocal < 66 && qntCombustivelLocal >= 33) {
	  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, 0);
	  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, 1);
	  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, 0);
//	  HAL_UART_Transmit(&huart2, "taskAcionamento 55\r\n", 33, 1000);
	}

	if(qntCombustivelLocal >= 66) {
	  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, 0);
	  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, 0);
	  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, 1);
//	  HAL_UART_Transmit(&huart2, "taskAcionamento 66\r\n", 33, 1000);
	}

    osDelay(10);
  }
  /* USER CODE END StartAcionamento */
}

/* USER CODE BEGIN Header_StartEscritaMemoria */
/**
* @brief Function implementing the taskEscritaMemo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEscritaMemoria */
void StartEscritaMemoria(void const * argument)
{
  /* USER CODE BEGIN StartEscritaMemoria */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(10);
  }
  /* USER CODE END StartEscritaMemoria */
}

/* USER CODE BEGIN Header_StartEscritaDisplay */
/**
* @brief Function implementing the taskEscritaDisp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEscritaDisplay */
void StartEscritaDisplay(void const * argument)
{
  /* USER CODE BEGIN StartEscritaDisplay */
	uint8_t informacoesLocal[2];
	uint8_t bufferOxigenio[50];
	uint8_t bufferAcelerador[50];
  /* Infinite loop */
  for(;;)
  {
	  osMutexWait(MtxInformacoesHandle, 1000);
	  informacoesLocal[0] = informacoesGlobal[0];
	  informacoesLocal[1] = informacoesGlobal[1];
	  osMutexRelease(MtxInformacoesHandle);

	  sprintf(bufferAcelerador, "Resultado Injecao: %d\r\n", informacoesLocal[0]);
	  sprintf(bufferOxigenio, "Teste: %d\r\n\r\n", informacoesLocal[1]);


//	  HAL_UART_Transmit(&huart2, bufferAcelerador, 25, 1000);
//	  HAL_UART_Transmit(&huart2, bufferOxigenio, 17, 1000);

//	  LCD_Write_String(0, 0, bufferAcelerador);
//	  LCD_Write_String(0, 1, bufferOxigenio);
	  osDelay(10);
  }
  /* USER CODE END StartEscritaDisplay */
}

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
