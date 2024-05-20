#include "BTS7960_driver.h"


volatile uint8_t speed = 80; 


void DC_Driver_PWM_Init(void) {
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    TIM1 -> CCR4 = 0; 
}

void DC_Driver_SetDir(MotorDirection direction){
    if(direction == CLOCKWISE) {
        HAL_GPIO_WritePin(GPIOA, driver_RPWM_PINA_DC1, GPIO_PIN_SET); 
        HAL_GPIO_WritePin(GPIOA, driver_LPWM_PINA_DC1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, driver_RPWM_PINA_DC2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, driver_LPWM_PINB_DC2, GPIO_PIN_RESET);
    }else if(direction == COUNTERCLOCKWISE) {
        HAL_GPIO_WritePin(GPIOA, driver_RPWM_PINA_DC1, GPIO_PIN_RESET); 
        HAL_GPIO_WritePin(GPIOA, driver_LPWM_PINA_DC1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, driver_RPWM_PINA_DC2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, driver_LPWM_PINB_DC2, GPIO_PIN_SET);
    }
}

void DC_Driver_SetSpeed(uint16_t SPEED_INPUT) {
    //Speed from 0 to 100
    speed = SPEED_INPUT; 
}

void DC_Driver_Run(void) {
    TIM1 -> CCR4 = speed * 655.35; 
}

void DC_Driver_Stop(void) {
    TIM1 -> CCR4 = 0;
} 

void DC_Driver_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_DC_Driver = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    
    GPIO_DC_Driver.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
    GPIO_DC_Driver.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_DC_Driver.Pull = GPIO_NOPULL;
    GPIO_DC_Driver.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_DC_Driver);

    GPIO_DC_Driver.Pin = GPIO_PIN_0;
    GPIO_DC_Driver.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_DC_Driver.Pull = GPIO_NOPULL;
    GPIO_DC_Driver.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_DC_Driver);
		
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
		HAL_TIM_PWM_Init(&htim1);

		sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
		sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

		sConfigOC.OCMode = TIM_OCMODE_PWM1;
		sConfigOC.Pulse = 0;
		sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
		HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);
		sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
		sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
		sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
		sBreakDeadTimeConfig.DeadTime = 0;
		sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
		sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
		sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
		HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);
		
		/* USER CODE BEGIN TIM1_Init 2 */

		/* USER CODE END TIM1_Init 2 */
		HAL_TIM_MspPostInit(&htim1);
}

uint8_t DC_Driver_State()
{
  uint8_t State = 0;
  if(speed > 0)
  {
    State = 1;
  } else
  {
    State = 0;
  }
  
  return State;
}




