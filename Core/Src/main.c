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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SWARM_UART_HANDLER				&huart1
#define SWARM_UART_UART_TX_TIMEOUT		100
#define SWARM_UART_RX_MAX_BUFF_SIZE		100
#define SWARM_UART_TX_MAX_BUFF_SIZE		250
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
char             	swarm_uart_rx_buff[SWARM_UART_RX_MAX_BUFF_SIZE] ;
char             	swarm_uart_tx_buff[SWARM_UART_TX_MAX_BUFF_SIZE] ;
uint8_t				tim14_on ;
uint8_t				swarm_checklist = 0 ;
uint32_t			swarm_dev_id = 0 ;

// SWARM AT Commands
const char*			cs_at					= "$CS" ;
const char*			rt_0_at					= "$RT 0" ;
const char*			rt_q_rate_at			= "$RT ?" ;
const char*			pw_0_at					= "$PW 0\0" ;
const char*			pw_q_rate_at			= "$PW ?\0" ;
const char*			pw_mostrecent_at		= "$PW @\0" ;
const char*			dt_0_at					= "$DT 0\0" ;
const char*			dt_q_rate_at			= "$DT ?\0" ;
const char*			gs_0_at					= "$GS 0\0" ;
const char*			gs_q_rate_at			= "$GS ?\0" ;
const char*			gj_0_at					= "$GJ 0\0" ;
const char*			gj_q_rate_at			= "$GJ ?\0" ;
const char*			gn_0_at					= "$GN 0\0" ;
const char*			gn_q_rate_at			= "$GN ?\0" ;
// SWARM AT Answers
const char*         cs_answer				= "$CS DI=0x" ;
const char*         rt_ok_answer			= "$RT OK*22" ;
const char*         rt_0_answer				= "$RT 0*16" ;
const char*         pw_ok_answer			= "$PW OK*23\0" ;
const char*         pw_0_answer				= "$PW 0*17\0" ;
const char*         pw_mostrecent_answer	= "$PW \0" ;
const char*         dt_ok_answer			= "$DT OK*34\0" ;
const char*         dt_0_answer				= "$DT 0*00\0" ;
const char*         gs_ok_answer			= "$GS OK*30\0" ;
const char*         gs_0_answer				= "$GS 0*04\0" ;
const char*         gj_ok_answer			= "$GJ OK*29\0" ;
const char*         gj_0_answer				= "$GJ 0*1d\0" ;
const char*         gn_ok_answer			= "$GN OK*2d\0" ;
const char*         gn_0_answer				= "$GN 0*19\0" ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
void			m138_init () ;
float 			m138_get_voltage () ;
void			send_at_command_2_swarm ( const char* , const char* , uint16_t ) ;
void			clean_array ( char* , uint16_t ) ;
uint8_t			nmea_checksum ( const char* , size_t ) ;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_CLEAR_IT ( &htim14 , TIM_IT_UPDATE ) ;
  HAL_UARTEx_ReceiveToIdle_DMA ( SWARM_UART_HANDLER , (uint8_t*) swarm_uart_rx_buff , SWARM_UART_RX_MAX_BUFF_SIZE ) ;
  __HAL_DMA_DISABLE_IT ( &hdma_usart1_rx, DMA_IT_HT ) ; //Disable Half Transfer interrupt.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //HAL_Delay ( 15000 ) ;
  m138_init () ;
  m138_get_voltage () ;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //HAL_PWREx_EnterSHUTDOWNMode () ;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 5000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : T_NRST_Pin */
  GPIO_InitStruct.Pin = T_NRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(T_NRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GREEN_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
float m138_get_voltage ()
{
	float m138_voltage = 0.0 ;
	char* chunk = malloc ( 30 * sizeof (char) ) ;
	send_at_command_2_swarm ( pw_mostrecent_at , pw_mostrecent_answer , 14 ) ;
	if ( swarm_checklist == 13 )
	{
		chunk = strtok ( (char*) swarm_uart_rx_buff , " " ) ;
		chunk = strtok ( NULL , "," ) ;
		m138_voltage = (float) strtof ( chunk , NULL ) ;
	}
	free ( chunk ) ;
	//clean_array ( swarm_uart_rx_buff , SWARM_UART_RX_MAX_BUFF_SIZE ) ;
	return m138_voltage ;
}

void m138_init ()
{
	char* chunk = malloc ( 20 * sizeof (char) ) ;
	send_at_command_2_swarm ( cs_at , cs_answer , 1 ) ;
	if ( swarm_checklist == 1 )
	{
		chunk = strtok ( (char*) swarm_uart_rx_buff , "=" ) ;
		chunk = strtok ( NULL , "," ) ;
		swarm_dev_id = (uint32_t) strtol ( chunk , NULL , 16 ) ;
		//clean_array ( swarm_uart_rx_buff , SWARM_UART_RX_MAX_BUFF_SIZE ) ;
		send_at_command_2_swarm ( rt_0_at , rt_ok_answer , 2 ) ;
	}
	if ( swarm_checklist == 2 )
		send_at_command_2_swarm ( rt_q_rate_at , rt_0_answer , 3 ) ;
	if ( swarm_checklist == 3 )
		send_at_command_2_swarm ( pw_0_at , pw_ok_answer , 4 ) ;
	if ( swarm_checklist == 4 )
		send_at_command_2_swarm ( pw_q_rate_at , pw_0_answer , 5 ) ;
	if ( swarm_checklist == 5 )
		send_at_command_2_swarm ( dt_0_at , dt_ok_answer , 6 ) ;
	if ( swarm_checklist == 6 )
		send_at_command_2_swarm ( dt_q_rate_at , dt_ok_answer , 7 ) ;
	if ( swarm_checklist == 7 )
		send_at_command_2_swarm ( gs_0_at , gs_ok_answer  , 8 ) ;
	if ( swarm_checklist == 8 )
		send_at_command_2_swarm ( gs_q_rate_at , gs_0_answer , 9 ) ;
	if ( swarm_checklist == 9 )
		send_at_command_2_swarm ( gj_0_at , gj_ok_answer  , 10 ) ;
	if ( swarm_checklist == 10 )
		send_at_command_2_swarm ( gj_q_rate_at , gj_0_answer , 11 ) ;
	if ( swarm_checklist == 11 )
		send_at_command_2_swarm ( gn_0_at , gn_ok_answer  , 12 ) ;
	if ( swarm_checklist == 12 )
		send_at_command_2_swarm ( gn_q_rate_at , gn_0_answer , 13 ) ;
	free ( chunk ) ;
	//clean_array ( swarm_uart_rx_buff , SWARM_UART_RX_MAX_BUFF_SIZE ) ;
}

void send_at_command_2_swarm ( const char* at_command , const char* answer , uint16_t step )
{
	uint8_t cs = nmea_checksum ( at_command , strlen ( at_command ) ) ;

	sprintf ( swarm_uart_tx_buff , "%s*%02x\n" , at_command , cs ) ;

	tim14_on = 1 ;
	HAL_TIM_Base_Start_IT ( &htim14 ) ;
	//Usunąc poniższe bo nie dziala do końca przez to
	//clean_array ( swarm_uart_rx_buff , SWARM_UART_RX_MAX_BUFF_SIZE ) ;
	//swarm_uart_rx_buff[0] = 0 ;
	HAL_UART_Transmit ( SWARM_UART_HANDLER , (uint8_t*) swarm_uart_tx_buff ,  strlen ( (char*) swarm_uart_tx_buff ) , SWARM_UART_UART_TX_TIMEOUT ) ;
	while ( tim14_on )
	{
		if ( swarm_uart_rx_buff[0] != 0 )
		{
			if ( strncmp ( (char*) swarm_uart_rx_buff , answer , strlen ( answer ) ) == 0 )
			{
				swarm_checklist = step ;
				break ;
			}
			//else
			//	swarm_uart_rx_buff[0] = 0 ;
		}
	}
	//clean_array ( swarm_uart_tx_buff , SWARM_UART_TX_MAX_BUFF_SIZE ) ;
}

void clean_array ( char* array , uint16_t array_max_size )
{
	uint16_t i ;
	for ( i = 0 ; i < array_max_size ; i++ )
		array[i] = 0 ;
}

uint8_t nmea_checksum ( const char *message , size_t len )
{
	size_t i = 0 ;
	uint8_t cs ;
	if ( message [0] == '$' )
		i++ ;
	for ( cs = 0 ; ( i < len ) && message [i] ; i++ )
		cs ^= ( (uint8_t) message [i] ) ;
	return cs;
}

void HAL_UARTEx_RxEventCallback ( UART_HandleTypeDef *huart , uint16_t Size )
{
	//const char* z = 0 ;
    if ( huart->Instance == USART1 )
    {
    	//if ( swarm_uart_rx_buff[0] != 0 ) // to avoid doublet because of 2 INTs
    	//	strcat ( (char *) swarm_uart_rx_buff , z ) ; // to avoid debris after '\n' of original message
    	swarm_uart_rx_buff[Size] = 0 ;
    	HAL_UARTEx_ReceiveToIdle_DMA ( SWARM_UART_HANDLER , (uint8_t *) swarm_uart_rx_buff , SWARM_UART_RX_MAX_BUFF_SIZE ) ;
		__HAL_DMA_DISABLE_IT ( &hdma_usart1_rx, DMA_IT_HT ) ; //Disable Half Transfer interrupt.
    }
}
void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef *htim )
{
	if ( htim->Instance == TIM14 )
	{
		tim14_on = 0 ;
		HAL_TIM_Base_Stop_IT ( &htim14 ) ;
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
