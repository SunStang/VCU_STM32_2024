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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

	uint16_t num;
	char buf[64]; //change to the thing you want to print
	uint32_t wait;
	uint32_t tickFreq = (uint32_t)HAL_GetTickFreq;
	uint32_t currTick;
	uint32_t prevTickOp = 0;
	uint32_t prevTickCAN = 0;
	uint32_t prevTickDrive = 0;
	uint32_t prevTickPower = 0;
	uint8_t CANWait = 150; //150ms
	uint8_t DrivePeriod = 50; //50ms
	uint8_t PowerPeriod = 80; //80ms
	int datacheck = 0;

	volatile uint16_t	adcResults[2];
	const int			adcChannelCount = sizeof(adcResults) / sizeof(adcResults[0]);
	volatile int		adcConversionComplete = 0; // set by a callback function

	uint16_t acel = 0;
	uint16_t brake = 0;

	CAN_TxHeaderTypeDef TxHeaderData;
	CAN_TxHeaderTypeDef TxHeaderRequest;
	CAN_RxHeaderTypeDef RxHeader;

	uint8_t TxRequest[8];
  	uint8_t RxData[8];

  	union Data {
  		int i;
  		float f;
  		uint8_t byte[8];
  	};

  	union Data upperByte; 	//aid in converting upper bytes to float

  	union Data TxData;
  	union Data Current; 	//motor current percent of max current
  	union Data Velocity; 	//motor velocity desired output
  	union Data Power; 		//bus current max percent of absolute current
  	union Data RPM; 		//motor current rpm

  	uint32_t TxMailbox;
  	uint32_t DriveMB;
  	uint32_t PowerMB;

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
	/*Need to change datacheck to bits and then assign bits per instruction
	* bit mask back in the code for what the flag says it needs to do
	* Reason: msgs can come in quicker than code execution
	* Don't want overwrite of actions for only 1 action
	*/
	//Will also need to add in switch case for the RxHeader.StdId
	if(RxHeader.StdId == 0x403){
	//new msg about speed
		datacheck = 1;
	}
}

float ByteToFloat(uint8_t byte0, uint8_t byte1, uint8_t byte2, uint8_t byte3){
	uint8_t Abyte[4] = {byte0, byte1, byte2, byte3};
	for(int i = 0; i <= 3; i++){
		upperByte.byte[i] = Abyte[i];
	}
	return upperByte.f;
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

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  HAL_CAN_Start(&hcan);

  TxHeaderData.DLC = 8;
  TxHeaderData.IDE = CAN_ID_STD;
  TxHeaderData.RTR = CAN_RTR_DATA;
  TxHeaderData.StdId = 0x500; //set the ID of the VCU

  TxHeaderRequest.DLC = 8;
  TxHeaderRequest.IDE = CAN_ID_STD;
  TxHeaderRequest.RTR = CAN_RTR_REMOTE;
  TxHeaderRequest.StdId = 0x500; //Set the ID of VCU

  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
  	  /* USER CODE END WHILE */
  	  //Set the current time
  	  currTick = HAL_GetTick();

  	  /* -- Debugging
  	  //print out some information to the console via UART
  	  //sprintf(buf, "%u: %lu: %lu \r\n", num, tickFreq, currTick);
  	  //HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);
  	   */


  	  //ADC readings with DMA help
  	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcResults, adcChannelCount);
  	  while(adcConversionComplete == 0){
  	  } //waiting for conversion to complete
  	  adcConversionComplete = 0;

  	  /*
  	  //Record the ADC value from channel 1 to num
  	  HAL_ADC_Start(&hadc1);
  	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  	  num = HAL_ADC_GetValue(&hadc1);
  	  */
  	  //set wait
  	  wait = 250;
  	  acel = adcResults[0];
  	  brake = adcResults[1];

  	  //toggle a GPIO pin with the time delay from the potentiometer
  	  if(currTick - prevTickOp > wait) {

  		  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);

  		  // -- Debugging
  		  sprintf(buf, "Acel: %u  Brake: %u \r\n", adcResults[0], adcResults[1]);
  		  HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);


  		  prevTickOp = currTick;
  	  }

  	  //create a delay to send CAN messages THIS IS TEST
  	  if(currTick - prevTickCAN > CANWait){
  		  //set the CAN message to send
  		  //uint8_t Tx[] = {50, 0xAA};
  		  TxData.f = wait;

  		  //Must Assemble the bytes in the union to get an array,
  		  uint8_t Tx[] = {TxData.byte[0], TxData.byte[1], TxData.byte[2], TxData.byte[3], 0, 0, 0, 0};

  		  //Alter address to send to
  		  TxHeaderData.StdId = 0x800;

  		  //send a message through CAN as data
  		  HAL_CAN_AddTxMessage(&hcan, &TxHeaderData, Tx, &TxMailbox);

  		  prevTickCAN = currTick;
  	  }

  	  //send the motor drive command frame, 50ms delay
  	  if(currTick - prevTickDrive > DrivePeriod){

  		  //If we are not braking
  		  if(brake == 0){
  			  //And not accelerating
			  if(acel == 0){
				  //Set to coast
				  Velocity.f = 0;
				  Current.f = 0;
			  }
			  //And accelerating
			  else{
				  //Set high velocity, and scale the current
				  Velocity.f = 1000;
		  		  //convert the int to float for the current percent
		  		  //Current.f = (float)acel/4096;
				  Current.f = 0.00000006*(pow((float)acel, 2));
			  }
		  //If we are braking
  		  } else {
  			  //Set velocity to 0
  			  Velocity.f = 0;
  			  //And if we are not pressing accelerator
  			  if(acel == 0){
  				  //Scale re-gen current with brake until rpm is below a set point
  				  if(RPM.f > 120){
  					  //Current.f = (float)brake/4096;
  					  Current.f = 0.00000002*(pow((float)brake, 2));
  				  }
  			  //If we are pressing both pedals at same time
  			  } else {
  				  //in addition to velocity set to 0, current set to 0
  				  Current.f = 0;
  			  }
  		  }

  		  //Assemble frame
  		  uint8_t DriveTx[] = {Velocity.byte[0], Velocity.byte[1], Velocity.byte[2], Velocity.byte[3], Current.byte[0], Current.byte[1], Current.byte[2], Current.byte[3]};

  		  //Alter address to send to
  		  TxHeaderData.StdId = 0x501;

  		  //Send frame
  		  HAL_CAN_AddTxMessage(&hcan, &TxHeaderData, DriveTx, &DriveMB);

  		  //reset Time
  		  prevTickDrive = currTick;
  	  }

  	  //send the motor drive command frame, 80ms delay
  	  if(currTick - prevTickPower > PowerPeriod){

  		  //Set percent to 100
  		  Power.f = 1.0;

  		  //Assemble frame
  		  uint8_t PowerTx[] = {0, 0, 0, 0, Power.byte[0], Power.byte[1], Power.byte[2], Power.byte[3]};

  		  //Alter address to send to
  		  TxHeaderData.StdId = 0x502;

  		  //Send frame
  		  HAL_CAN_AddTxMessage(&hcan, &TxHeaderData, PowerTx, &PowerMB);

  		  //reset Time
  		  prevTickPower = currTick;
  	  }

  	  //if the data flag is set, do something then reset the flag
  	  if(datacheck){
  		   // -- Debugging
  		  /*
  		  sprintf(buf, "%d: %u%u%u%u%u%u%u%u \r\n", RxHeader.StdId, RxData[7],RxData[6], RxData[5], RxData[4],
  											   RxData[3], RxData[2], RxData[1], RxData[0]);
  		  HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);
  		  */
  		  /*sprintf(buf, "In receive \r\n");
  		  HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);
  		  */
  		  for(char i = 0; i <= 8; i++){
  			  RPM.byte[i] = RxData[i];
  		  }

  		  /*sprintf(buf, "%x: Velocity: %e m/s RPM: %e \r\n", RxHeader.StdId, ByteToFloat(RPM.byte[4], RPM.byte[5], RPM.byte[6], RPM.byte[7]),
  				ByteToFloat(RPM.byte[3], RPM.byte[2], RPM.byte[1], RPM.byte[0]));
  		  HAL_UART_Transmit(&huart2, buf, strlen(buf), HAL_MAX_DELAY);
		  */
  		  datacheck = 0; //reset
  	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  	//Set a up a filter
    //Allow all messages to pass through from any ID
    CAN_FilterTypeDef cf1;
    cf1.FilterActivation = CAN_FILTER_ENABLE;
    cf1.FilterBank = 0;
    cf1.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    cf1.FilterMode = CAN_FILTERMODE_IDMASK;
    cf1.FilterScale = CAN_FILTERSCALE_32BIT;
    cf1.FilterIdLow = 0x0;
    cf1.FilterIdHigh = 0x0<<5;
    cf1.FilterMaskIdLow = 0x0;
    cf1.FilterMaskIdHigh = 0x0<<5;

    HAL_CAN_ConfigFilter(&hcan, &cf1);


  /* USER CODE END CAN_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	adcConversionComplete = 1;
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
