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
#define SWARM_UART_TX_TIMEOUT			100
#define SWARM_UART_RX_MAX_BUFF_SIZE		100
#define SWARM_ANSWER_MAX_BUFF_SIZE		100
#define SWARM_UART_TX_MAX_BUFF_SIZE		250
#define SWARM_TD_PAYLOAD_MAX_BUFF_SIZE	192
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
char             	swarm_uart_rx_buff[SWARM_UART_RX_MAX_BUFF_SIZE] ;
char             	swarm_uart_tx_buff[SWARM_UART_TX_MAX_BUFF_SIZE] ;
uint8_t				tim14_on ;
uint8_t				swarm_checklist = 0 ;
uint32_t			m138_dev_id = 0 ;
//unsigned int		m138_dev_id = 0 ;
float				m138_voltage = 0 ;
char				m138_fix[50] ;
uint8_t				answer_from_swarm_come = 0 ;

// SWARM AT Commands
const char*			cs_at					= "$CS" ;
const char*			rt_0_at					= "$RT 0" ;
const char*			rt_q_rate_at			= "$RT ?" ;
const char*			pw_0_at					= "$PW 0" ;
const char*			pw_q_rate_at			= "$PW ?" ;
const char*			pw_mostrecent_at		= "$PW @" ;
const char*			dt_0_at					= "$DT 0" ;
const char*			dt_q_rate_at			= "$DT ?" ;
const char*			gs_0_at					= "$GS 0" ;
const char*			gs_q_rate_at			= "$GS ?" ;
const char*			gj_0_at					= "$GJ 0" ;
const char*			gj_q_rate_at			= "$GJ ?" ;
const char*			gn_0_at					= "$GN 0" ;
const char*			gn_q_rate_at			= "$GN ?" ;
const char*			gn_mostrecent_at		= "$GN @" ;
const char*			sl_60s_at_comm			= "$SL S=60" ;
// SWARM AT Answers
const char*         cs_answer				= "$CS DI=0x" ;
const char*         rt_ok_answer			= "$RT OK*22" ;
const char*         rt_0_answer				= "$RT 0*16" ;
const char*         pw_ok_answer			= "$PW OK*23" ;
const char*         pw_0_answer				= "$PW 0*17" ;
const char*         pw_mostrecent_answer	= "$PW " ;
const char*         dt_ok_answer			= "$DT OK*34" ;
const char*         dt_0_answer				= "$DT 0*00" ;
const char*         gs_ok_answer			= "$GS OK*30" ;
const char*         gs_0_answer				= "$GS 0*04" ;
const char*         gj_ok_answer			= "$GJ OK*29" ;
const char*         gj_0_answer				= "$GJ 0*1d" ;
const char*         gn_ok_answer			= "$GN OK*2d" ;
const char*         gn_0_answer				= "$GN 0*19" ;
const char*         gn_mostrecent_answer	= "$GN " ;
const char*			td_ok_answer			= "$TD OK," ;
const char*         sl_ok_answer			= "$SL OK*3b\0" ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void				m138_init 				() ;
uint8_t 			store_m138_dev_id 		( uint32_t* , char* ) ;
uint8_t 			store_m138_voltage 		( float* , char* ) ;
uint8_t 			store_m138_fix 			( char* , char* ) ;
uint8_t 			send_m138_message		() ;
HAL_StatusTypeDef 	send_string_2_uart 		( char* ) ;
uint8_t				uart_comm 				( const char* , const char* , uint16_t ) ;
void				receive_dma_uart 		() ;
void				clean_array 			( char* , uint16_t ) ;
uint8_t				nmea_checksum 			( const char* , size_t ) ;
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
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_CLEAR_IT ( &htim14 , TIM_IT_UPDATE ) ;
  //HAL_UARTEx_ReceiveToIdle_DMA ( SWARM_UART_HANDLER , (uint8_t*) swarm_uart_rx_buff , SWARM_UART_RX_MAX_BUFF_SIZE ) ;
  //__HAL_DMA_DISABLE_IT ( &hdma_usart1_rx, DMA_IT_HT ) ; //Disable Half Transfer interrupt.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_Delay ( 1800 ) ;
  m138_init () ;

  while (1)
  {
	  uart_comm ( pw_mostrecent_at , pw_mostrecent_answer , 14 ) ;
	  if ( swarm_checklist == 14 )
	  {
		  store_m138_voltage ( &m138_voltage , swarm_uart_rx_buff ) ;
		  uart_comm ( gn_mostrecent_at , gn_mostrecent_answer , 15 ) ;
		  if ( swarm_checklist == 15 )
		  {
			  store_m138_fix ( m138_fix , swarm_uart_rx_buff ) ;
		  	  uart_comm ( gn_mostrecent_at , gn_mostrecent_answer , 16 ) ;
		  }
		  if ( swarm_checklist == 16 )
		  {
			  send_m138_message () ;
			  /*
			  HAL_Delay ( 60000 ) ;
			  if ( swarm_checklist == 17 )
			  {
				  uart_comm ( sl_60s_at_comm , sl_ok_answer , 18 ) ;
			  }
			  */
		  }
	  }
	  HAL_Delay ( 3000 ) ;
	  //HAL_PWR_EnterSTOPMode ( PWR_LOWPOWERREGULATOR_ON , PWR_STOPENTRY_WFI ) ;
	  //HAL_PWR_EnterSLEEPMode ( PWR_MAINREGULATOR_ON , PWR_SLEEPENTRY_WFI ) ;
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 60, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
uint8_t send_m138_message ()
{
	snprintf ( swarm_uart_tx_buff , SWARM_TD_PAYLOAD_MAX_BUFF_SIZE , "$TD HD=60,\"%u;%f;%s\"" , (unsigned int) m138_dev_id , m138_voltage , m138_fix ) ;
	uart_comm ( swarm_uart_tx_buff , td_ok_answer , 17 ) ;
	if ( swarm_checklist == 17 )
		return 1 ;
	else
		return 0 ;
}
uint8_t store_m138_fix ( char* d , char* s )
{
	if ( ! strstr ( s , "$GN " ) )
		return 0 ;
	s = strtok ( (char*) s , " " ) ;
	s = strtok ( NULL , "*" ) ;
	size_t l =  strlen ( s ) ;
	memcpy ( d , s , l ) ;
	d[l] = '\0' ;
	return 1 ;
}

uint8_t store_m138_voltage ( float* d , char* s )
{
	if ( ! strstr ( s , "$PW " ) )
		return 0 ;
	s = strtok ( (char*) s , " " ) ;
	s = strtok ( NULL , "," ) ;
	*d = (float) strtof ( s , NULL ) ;
	return 1 ;
	//clean_array ( swarm_uart_rx_buff , SWARM_UART_RX_MAX_BUFF_SIZE ) ;
}

uint8_t store_m138_dev_id ( uint32_t* dev_id , char* s )
{
	if ( ! strstr ( s , "DI=0x" ) )
		return 0 ;
	s = strtok ( (char*) s , "=" ) ;
	s = strtok ( NULL , "," ) ;
	*dev_id = (uint32_t) strtol ( s , NULL , 16 ) ;
	return 1 ;
	//clean_array ( swarm_uart_rx_buff , SWARM_UART_RX_MAX_BUFF_SIZE ) ;
}

void m138_init ()
{
	uart_comm ( cs_at , cs_answer , 1 ) ;
	if ( swarm_checklist == 1 )
	{
		store_m138_dev_id ( &m138_dev_id , swarm_uart_rx_buff ) ;
		uart_comm ( rt_0_at , rt_ok_answer , 2 ) ;
	}
	if ( swarm_checklist == 2 )
		uart_comm ( rt_q_rate_at , rt_0_answer , 3 ) ;
	if ( swarm_checklist == 3 )
		uart_comm ( pw_0_at , pw_ok_answer , 4 ) ;
	if ( swarm_checklist == 4 )
		uart_comm ( pw_q_rate_at , pw_0_answer , 5 ) ;
	if ( swarm_checklist == 5 )
		uart_comm ( dt_0_at , dt_ok_answer , 6 ) ;
	if ( swarm_checklist == 6 )
		uart_comm ( dt_q_rate_at , dt_0_answer , 7 ) ;
	if ( swarm_checklist == 7 )
		uart_comm ( gs_0_at , gs_ok_answer  , 8 ) ;
	if ( swarm_checklist == 8 )
		uart_comm ( gs_q_rate_at , gs_0_answer , 9 ) ;
	if ( swarm_checklist == 9 )
		uart_comm ( gj_0_at , gj_ok_answer  , 10 ) ;
	if ( swarm_checklist == 10 )
		uart_comm ( gj_q_rate_at , gj_0_answer , 11 ) ;
	if ( swarm_checklist == 11 )
		uart_comm ( gn_0_at , gn_ok_answer  , 12 ) ;
	if ( swarm_checklist == 12 )
		uart_comm ( gn_q_rate_at , gn_0_answer , 13 ) ;
}

uint8_t uart_comm ( const char* at_command , const char* answer , uint16_t step )
{
	uint8_t t ;
	uint8_t cs = nmea_checksum ( at_command , strlen ( at_command ) ) ;
	sprintf ( swarm_uart_tx_buff , "%s*%02x\n" , at_command , cs ) ;
	receive_dma_uart () ;
	for ( t = 0 ; t < 5 ; t++ )
	{
		tim14_on = 1 ;
		HAL_TIM_Base_Start_IT ( &htim14 ) ;
		//Usunąc poniższe bo nie dziala do końca przez to
		answer_from_swarm_come = 0 ;
		send_string_2_uart ( swarm_uart_tx_buff ) ;
		while ( tim14_on )
		{
			if ( answer_from_swarm_come == 1 )
			{
				if ( strncmp ( swarm_uart_rx_buff , answer , strlen ( answer ) ) == 0 )
					swarm_checklist = step ;
				break ;
				receive_dma_uart () ;
			}
		}
		if ( swarm_checklist == step )
			break ;
	}
	if ( swarm_checklist == step )
		return 1 ;
	else
		return 0 ;
}

void clean_array ( char* array , uint16_t array_max_size )
{
	uint16_t i ;
	for ( i = 0 ; i < array_max_size ; i++ )
		array[i] = 0 ;
}

HAL_StatusTypeDef send_string_2_uart ( char* s )
{
	return HAL_UART_Transmit ( SWARM_UART_HANDLER , (uint8_t *) s , strlen ( s ) , SWARM_UART_TX_TIMEOUT ) ;
}

void receive_dma_uart ()
{
	HAL_UARTEx_ReceiveToIdle_DMA ( SWARM_UART_HANDLER , (uint8_t*) swarm_uart_rx_buff , SWARM_UART_RX_MAX_BUFF_SIZE ) ;
	__HAL_DMA_DISABLE_IT ( &hdma_usart1_rx, DMA_IT_HT ) ; //Disable Half Transfer interrupt.
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
    if ( huart->Instance == USART1 )
    {
    	swarm_uart_rx_buff[Size] = '\0' ;
    	answer_from_swarm_come = 1 ;
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
