/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADC_BUF_LEN 10
#define P 0
#define I 0.05
#define D 0
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

DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t PowerLevel = 500;
volatile uint16_t adc_buf[ADC_BUF_LEN];

volatile uint16_t adc_buf_high[ADC_BUF_LEN];


uint8_t  flagIn = 0;
uint8_t  flagOut = 1;

double power_val = 2000;
double power_vals[50] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double power_vals_old[50] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double power_mean = 0;
int16_t error = 0;
int32_t errorSum = 0;
uint8_t sample_no = 10;
uint8_t startUp = 0;
uint8_t startUp_2 = 0;
uint16_t Cur_Base_Val = 600;
uint16_t Base_Pow = 3300;
char msg[20];
/*------------------original parameters of PWM
sConfigOC.Pulse = 90; /10 duty cycle

*/
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
  MX_TIM2_Init();
  MX_TIM1_Init();

  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
  //LL_ADC_Enable(ADC1);

    //HAL_DMA_Start_IT (hdma_adc1, uint32_t SrcAddress, adc_buf, ADC_BUF_LEN);

  HAL_ADC_Start(&hadc1);
  HAL_Delay(100);
  HAL_ADC_Start_DMA(&hadc1,(uint16_t*)(adc_buf), ADC_BUF_LEN);
  HAL_Delay(1000);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R,Cur_Base_Val);


    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);

    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    HAL_Delay(5000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*if(flagIn == 1 && startUp < 0){
		  startUp++;
		  HAL_Delay(100);
		  flagIn = 0;
	  }*/


	  if(flagIn == 1 ){
		  	  HAL_Delay(100);
		  	  //HAL_ADC_Stop_DMA(&hadc1);
		  	  	  	  //memset(msg, 0, sizeof(msg));
		  	 		//strncpy(msg, "Mean:\r\n", sizeof(msg));
		  	 	if(startUp < 5){startUp++;}
		  	 	else if(startUp >= 5){
		  	 	for(int l = 0; l < 75; l++ ){
		  	 	  power_val = 0;
		  	  	  for(int i = 2; i < 9; i++){
		  	  		  power_val += adc_buf_high[i];
		  	  	  }
		  	  	  power_val = power_val/(8);
		  	  	  power_mean = power_mean + power_val;
		  	  	  HAL_Delay(10);
		  	  	  flagIn = 0;
		  	  	  while(flagIn == 0){}

		  	  		/*for(int i = 0; i < 49; i++){
		  	  			power_vals[i] = power_vals_old[i+1];

		  	  		}
		  	  	power_vals[49] = power_val;
		  	  		for(int i = 0; i < 50; i++){
		  	  		  	power_vals_old[i] = power_vals[i];

		  	  		 }
		  	  	power_mean = 0;
		  	  	for(int i = 0; i < 50; i++){
		  	  			power_mean += power_vals[i];
		  	  	}*/
		  	 	}
		  	  power_mean = power_mean/75;// power_mean/50;
		  	  if(startUp_2 < 50){startUp_2++;}
		  	  else if(startUp_2 >= 50){
		  		memset(msg, 0, sizeof(msg));
		  		//strncpy(msg, "Mean:\r\n", sizeof(msg));
		  		//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		  		//HAL_Delay(10);
		  	 	//sprintf(msg, "%hu\r\n", (uint16_t)(power_mean));
		  	 	//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);


		  	 	//HAL_Delay(10);


		  		error = Base_Pow - power_mean;
		  		errorSum += error;

		  		//sprintf(msg, "%hu\r\n", (uint16_t)(I*errorSum));
		  		//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);


		  		if(errorSum > (81900 - Cur_Base_Val)){
		  			errorSum = (81900 - Cur_Base_Val);

		  		}
		  		else if(errorSum < (Cur_Base_Val - 81900)){
		  			errorSum = Cur_Base_Val - 81900;

		  		}

		  		if((Cur_Base_Val + P*error + I*errorSum) < 4095 && ((Cur_Base_Val + P*error + I*errorSum) > 0)){
		  	 		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R,(Cur_Base_Val + P*error + I*errorSum));
		  	 		//sprintf(msg, "%hu\r\n", (uint16_t)(Cur_Base_Val + P*error + I*errorSum));
		  	 		//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		  	 	}
		  	 	else if ((Cur_Base_Val + P*error + I*errorSum) < 0){
		  	 		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R,0);
		  	 		//sprintf(msg, "%hu\r\n", (uint16_t)(0));
		  	 		//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		  	 	}
		  	 	else{
		  	 		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1,DAC_ALIGN_12B_R,(4095));
		  	 		//sprintf(msg, "%hu\r\n", (uint16_t)(4095));
		  	 		//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		  	 	}


		  	 	//strncpy(msg, "DAC VAL:\r\n", sizeof(msg));
		  	 	//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

		  	 	/*HAL_Delay(10);
		  	 	strncpy(msg, "Sum:\r\n", sizeof(msg));
		  	 	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		  	 	//HAL_Delay(10);
		  	 	sprintf(msg, "%hu\r\n", (int32_t)(errorSum));
		  	 	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

		  	 	HAL_Delay(10);*/

		  	  	//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		  	  	memset(msg, 0, sizeof(msg));
		  	  	/*strncpy(msg, "Samples:\r\n", sizeof(msg));
	 		  	HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	 		  	HAL_Delay(10);
	 		   for(int i = 0; i <ADC_BUF_LEN; i++){
	 			   	 sprintf(msg, "%hu\r\n", adc_buf_high[i]);
	 		  	 	 HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	 		  	 	 HAL_Delay(10);
	 		   	   }
	 		  	HAL_Delay(1000);*/
		  	  }
		  	 	}
		  	 	flagIn = 0;
	 	  		/*HAL_Delay(5000);
	 	  		memset(msg, 0, sizeof(msg));
	 	  		strncpy(msg, "Sampleshalf:\r\n", sizeof(msg));
	 	  		for(int i = 0; i <ADC_BUF_LEN/2; i++){
	 	  			 sprintf(msg, "%hu\r\n", adc_buf_work_half[i]);
	 	 		  	  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	 	  	 	 }*/
	 	  		  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
	 	  		  /*HAL_TIM_Base_Stop(&htim1);
	 	  		  HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);*/

	 	  		  //HAL_ADC_Start_DMA(&hadc1,(uint16_t*)(adc_buf), ADC_BUF_LEN);

	 	  		  /*HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);
	 	  		  HAL_ADC_Start(&hadc1);

	 	  	  	  HAL_Delay(1000);
	 	  	  	  HAL_TIM_Base_Start(&htim1);
	 	  	  	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);*/
	 	  	  	  //HAL_Delay(1000);
	 	  	  	  //clear = 1;

	 	  	  	  //HAL_Delay(1000);
	 	  	  	  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);


	 	  	  }
	  /*if(clear == 1){
		  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_LEN);

	  }*/


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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */
  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 13;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 11;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 9;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_TRIGGER;
  sSlaveConfig.InputTrigger = TIM_TS_ITR1;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 89;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC2REF;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 70;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 50;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*void ADC_DMA_TC_Callback(void){
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_9);

}*/
/*void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){

	if(flagIn == 0){
			//memcpy(&adc_buf_Active[0] , &adc_buf[0], (ADC_BUF_LEN / 2) * sizeof(adc_buf_Active[0]));
		//for (int i = ADC_BUF_LEN / 2; i < ADC_BUF_LEN; ++i) {

		//	adc_buf_high[i] = adc_buf[i];
		 // }
			//adc_buf_Active = adc_buf_high;
			//memcpy(adc_buf_work_half, adc_buf, sizeof(adc_buf_work_half));
			//hcplt = 1;
			//flagIn = 1;


}
}*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_9);

	if(flagIn == 0 /*&& hcplt == 1*/){
		//memcpy(&adc_buf_Active[ADC_BUF_LEN/2] , &adc_buf[ADC_BUF_LEN/2], (ADC_BUF_LEN / 2) * sizeof(adc_buf_Active[0]));
		//adc_buf_Active = adc_buf_low;
		for (int i = 0; i < ADC_BUF_LEN; ++i) {
					adc_buf_high[i] = adc_buf[i];
				  }

		//hcplt = 0;
		flagIn = 1;

	}
	/*uint32_t PowerLevel_temp = 0;
	uint32_t mean = 0;
	uint32_t NoHighSamp = 0;
	uint32_t temp_Val;

	for(int i = 0; i < ADC_BUF_LEN; i++){
		mean += adc_buf[i];
		//sprintf(msg, "%hu\r\n", adc_buf[i]);
		//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	}
	mean = mean/ADC_BUF_LEN;
	for(int i = 0; i < ADC_BUF_LEN; i++){
		temp_Val = adc_buf[i];
			if(temp_Val > mean){

				PowerLevel_temp += temp_Val;

			}
		}
	PowerLevel_temp = PowerLevel_temp/NoHighSamp;
	//HAL_ADC_Stop_DMA(&hadc);
	PowerLevel = PowerLevel_temp;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);*/

	//HAL_ADC_Stop(&hadc1);
}


void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc){
	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_9);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
