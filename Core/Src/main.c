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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BSP_Step.h"
#include "BTS7960_driver.h"
#include "led5050.h"
#include "Sensor.h"
#include "BSP_UART.h"
#include "stdbool.h"
#include "stdio.h"
#define SETP_TOCDO_COGIAMTOC 14*15
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
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t step = 0;
int countBottle = 0;
int countCan = 0;
uint32_t Timer_Count = 0, Step7_Timer_Count = 0;
uint32_t Prev_Timer_Count = 0, Step7_Prev_Timer_Count = 0;
int ErrorCount = 0;
uint32_t DC_Driver_CurTime = 0;
uint32_t DC_Driver_PrevTime = 0;
bool DC_Driver_Go_Middle = false;
bool UART_Available = false;
bool CanOrBottle = false;
bool HanhTrinhStep = false;
bool DC_OPERATING = false;
float checkADC =0, checkADC2=0;
bool test0 = false, test1 = false;
uint8_t data_rx[22];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart1.Instance) {
		HAL_UART_Receive_IT(&huart1, data_rx, 22);
	}
}

uint8_t checkLength(void) {
	if(data_rx[6] == 'P' && data_rx[7] == 'L') {
		return 8; 
	}
	else {
		return 22; 
	}
}

void resetDataRx(void) {
	for(int i = 0; i < 22; i++) {
		data_rx[i] = 0; 
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
	Timer_Count = HAL_GetTick();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	STEP_GPIO_Init();
	STEP_PUL_Init(&htim3, 20);

	set_StepRotateAngle(90);
//	set_StepSpeed(&htim3, 10);
	change_StepDirect(CW);
	
	DC_Driver_PWM_Init();
	DC_Driver_SetDir(CLOCKWISE);
	DC_Driver_SetSpeed(80);
	HAL_UART_Receive_IT(&huart1, data_rx, 22);
	STEP_Enable();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    switch(step) {
				case 0:
						DC_Driver_Stop();
						DC_Driver_SetDir(CLOCKWISE);
						STEP_Disable();
				
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
				
						//Renew information
						ErrorCount = 0;
						countBottle = 0;
						countCan = 0;
						Timer_Count = 0;
						
						DC_Driver_Go_Middle = false;
						HanhTrinhStep = false;
						
						Led_All_Off();
						HAL_Delay(200);
						
				
//						if(test0 == false) {
//								step = 1;
//						}
						if (data_rx[0] == 'T' && data_rx[1] == 'D' && data_rx[6] == 'P' && data_rx[7] == 'L') {
							if (data_rx[2] == '0' && data_rx[3] == '1' && data_rx[4] == '1' && data_rx[5] == '1' ) {
								step = 1;
							}
						}
						break;

				case 1:
					//STEP
					STEP_Enable();
					//DC Motor
					for(uint8_t i = 0; i < 8 ; i++) {
									data_rx[i] = 0;
					}
						change_StepDirect(CW);
						goHomeStep(&htim3);
					
					DC_Driver_Go_Middle = false;							
					DC_Driver_Stop();
					DC_Driver_SetDir(CLOCKWISE);
//					if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0) 
//					{
//						goHomeStep(&htim3);
//					}
					//UART
					UART_Available = false;
				
				  //LED
//					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);//BAT_DEN_VONG_CAMERA;
					Led_Red_Off();
					Led_Yellow_Off();
					Led_Green_On();

					while(UART_Available == false) {
						Timer_Count = HAL_GetTick();
						checkADC=Read_ADC_Value(2);
						checkADC2=Read_ADC_Value(1);
						if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8) == 1){
							stop_Step(&htim3);
						}
							
						if(Read_ADC_Value(2) >= 1.7) {
							HAL_Delay(1000);
							if(Read_ADC_Value(2) >= 1.7) {
								step = 2;
								UART_Available = true;
							}
						}
						else if((Timer_Count - Prev_Timer_Count > 30000) && (countBottle != 0 || countCan != 0)) {
							Prev_Timer_Count = Timer_Count;
							step = 6;
//							for(uint8_t i = 0; i < 8 ; i++) {
//								data_rx[i] = 0;
//							}
							UART_Available = true;
						}
						else if ((Timer_Count - Prev_Timer_Count > 30000) && countCan == 0 && countBottle == 0) {
							Prev_Timer_Count = Timer_Count;
							step = 0;
//							for(uint8_t i = 0; i < 8 ; i++) {
//								data_rx[i] = 0;
//							}
							UART_Available = true;
						}
						else if(data_rx[0] == 'T' && data_rx[1] == 'D' && data_rx[6] == 'P' && data_rx[7] == 'L') {
							if (data_rx[2] == '1' && data_rx[3] == '2' && data_rx[4] == '1' && data_rx[5] == '1' ) {
								step = 6;
//								for(uint8_t i = 0; i < 8 ; i++) {
//									data_rx[i] = 0;
//								}
								UART_Available = true;
							}
						}
					}
					break;
				
				case 2:
						UART_Available = false;
						Led_Green_Off();
						Led_Yellow_On();
//						UART_sendTakePictureRequest(&huart1);
						if(Read_Sensor(3)) { // sensor capacitor
							HAL_Delay(50);
							if(Read_Sensor(3)) {
							step = 3;
							countCan++;
							CanOrBottle = true;
							}
							} else {
							HAL_Delay(50);
							step = 3;
							countBottle++;
							CanOrBottle = false;
						}

						//if() {
						//  step = 7;//error
						//}
						break;

				case 3:
						DC_Driver_Run();
						DC_OPERATING = true;
						DC_Driver_CurTime = HAL_GetTick();
						DC_Driver_PrevTime = DC_Driver_CurTime;
						while(DC_OPERATING == true) {
							DC_Driver_CurTime = HAL_GetTick();
							while(DC_Driver_Go_Middle == false){
								checkADC=Read_ADC_Value(2);
								checkADC2=Read_ADC_Value(1);
								if(Read_ADC_Value(2) < 1.7) {
									HAL_Delay(50);
									if(Read_ADC_Value(2) < 1.7) {
										HAL_Delay(750);
										DC_Driver_Go_Middle = true;
									}
								}
							}

							if(Read_ADC_Value(1) >= 1.7) {
								HAL_Delay(50);
								if(Read_ADC_Value(1) >= 1.7) {
										step = 4;
										DC_Driver_PrevTime = DC_Driver_CurTime;
										DC_OPERATING = false;
								}
							}

							if(Read_ADC_Value(2) >= 1.7) {
								HAL_Delay(2);
								if(Read_ADC_Value(2) >= 1.7) {
									step = 7;
									DC_OPERATING = false;
								}
							}

							if(DC_Driver_CurTime - DC_Driver_PrevTime >= DC_TIMEOUT) {
								step = 7;
								DC_Driver_PrevTime = DC_Driver_CurTime;
								DC_OPERATING = false;
							}
						}
						break;

				case 4:
						HAL_Delay(150);	
						DC_Driver_Stop();
						if(CanOrBottle)
						{
							change_StepDirect(CW);
							set_StepRotateAngle(90);
//							do{
//								DC_Driver_Run();
//							}while(Read_ADC_Value(1) >= 1.7);
							run_Step(&htim3);
							HAL_Delay(500);
//							DC_Driver_Stop();
							step = 5;
						}
						else
						{
							change_StepDirect(CW);
							set_StepRotateAngle(90);
							do{
								DC_Driver_Run();
							}while(Read_ADC_Value(1) >= 1.7);
							HAL_Delay(2000);
							DC_Driver_Stop();
							//run_Step(&htim3);
							HAL_Delay(1000);
							step = 5;
						}
						break;

				case 5:
						//UART_sendCompleteStatus(&huart1, countBottle, countCan, ErrorCount); //GUI_TIN_HIEU_UPDATE_GIAO_DIEN;
						if(CanOrBottle) {
							HAL_UART_Transmit(&huart1,(uint8_t *)"TD3111PL" , 8, 100);
						}
						else if (CanOrBottle == false) {
							HAL_UART_Transmit(&huart1, (uint8_t *)"TD3000PL", 8, 100);
						}
						Prev_Timer_Count = HAL_GetTick();
						HAL_Delay(1000);
						Led_All_Off();
						step = 1;
						break;

				case 6:
						Led_All_Off();
						step = 0;
						break;

				case 7:
						Led_Yellow_Off(); 
						Led_Red_On();	
						UART_sendErrorInfor(&huart1); //GUI_TIN_HIEU_CANH_BAO_VE_MAY_TINH;
						HAL_UART_Transmit(&huart1, (uint8_t *)"TD7111PL", 8, 100);
						DC_Driver_SetDir(COUNTERCLOCKWISE);
						DC_Driver_Run();
				
						//Update timer timeout
						Step7_Prev_Timer_Count = HAL_GetTick();
						Step7_Timer_Count = Step7_Prev_Timer_Count;
					
						while(UART_Available == false) {
							Step7_Timer_Count = HAL_GetTick();
							if(data_rx[0] == 'T' && data_rx[1] == 'D' && data_rx[6] == 'P' && data_rx[7] == 'L') {
								if(data_rx[2] == '0' && data_rx[3] == '1' && data_rx[4] == '1' && data_rx[5] == '1' ) {
									step = 1;
									ErrorCount++;
									UART_Available = true;
								}
							}
							
							else if(data_rx[0] == 'T' && data_rx[1] == 'D' && data_rx[6] == 'P' && data_rx[7] == 'L') {
								if(data_rx[2] == '1' && data_rx[3] == '2' && data_rx[4] == '1' && data_rx[5] == '1' ) {
									step = 0;
									UART_Available = true;
								}
							}
							
							else if (Step7_Timer_Count - Step7_Prev_Timer_Count > 15000) {
								step = 1;
								UART_Available = true;
							}
						}
					break;

				case 8:
					break;

				default:
					break;

       }
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB12
                           PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB7
                           PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
