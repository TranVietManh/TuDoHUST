#include "BSP_Step.h"

uint16_t angle = 0;

uint32_t aHalfOfSpeed = 0;

void STEP_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PC13 - ENABLE PIN*/
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 - DIRECT PIN */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	/*Configure GPIO pins : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void STEP_GPIO_Deinit(void) {
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_13);
}

void STEP_Disable(void) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
}

void STEP_Enable(void) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

//PA6 - PUL PIN
//Khoi tao Chan phat xung, don vi speed: vong/s
void STEP_PUL_Init(TIM_HandleTypeDef* htimStep, uint32_t speed) {
	TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htimStep->Instance = TIM3;
  htimStep->Init.Prescaler = 72-1;
  htimStep->Init.CounterMode = TIM_COUNTERMODE_UP;
	/* Dat Period la x => Tan so la 1000000/X => Trong truong hop nay tan so la 1KHz  (voi do phan giai 200 xung/vong)
	Toc do quay = (1000000*60/(Period*200) Don vi vong/phut
	*/
  htimStep->Init.Period = ((1000000*60)/(speed*200))-1;
	aHalfOfSpeed = ((1000000*60)/(speed*200))/2;
  htimStep->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htimStep->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_PWM_Init(htimStep);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(htimStep, &sMasterConfig);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(htimStep, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_MspPostInit(htimStep);
	
	HAL_TIM_PWM_Start_IT(htimStep, TIM_CHANNEL_1);
	//Cau hinh %duty 50%
	__HAL_TIM_SetCompare(htimStep, TIM_CHANNEL_1, 000);
}

uint32_t count = 0;
uint8_t isFinished = 0;
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if (count < angle && isFinished == 0)
	{
		  count++;
	}
	else if (isFinished == 0 && count >= angle)
	{
		isFinished = 1;
		count = 0;
		__HAL_TIM_SetCompare(htim, TIM_CHANNEL_1, 000);
		HAL_TIM_PWM_Init(htim);
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	}
}

/*
* _angle c� don vi la do
* angle co don vi la xung 
* Do phan giai la 200 xung = 360 do
*/
void set_StepRotateAngle(uint32_t _angle){
	angle = _angle*200.0/360.0;
}

uint8_t set_StepSpeed(TIM_HandleTypeDef* htimStep, double speed){
	uint32_t periodValue = (1000000*60)/(speed*200);
	/* Dat Period la x => Tan so la 1000000/X => Trong truong hop nay tan so la 1KHz => 5 vong / 1s (voi do phan giai 200 xung/vong)
	Toc do quay = (1000000*60/(Period*200) Don vi vong/phut
	*/
	//Cap nhat gia tri Half of Speed de su dung duty cycle 50%
	aHalfOfSpeed = ((1000000*60)/(speed*200))/2;
  htimStep->Init.Period = periodValue - 1;
	if (HAL_TIM_PWM_Init(htimStep) != HAL_OK)
  {
    return 0;
  }
	return 1;
}

void change_StepDirect(rotateDir_t dir){
	if (dir == CW)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	}
	else if (dir == CCW)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	}
}

uint8_t run_Step(TIM_HandleTypeDef* htimStep){
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	count = 0;
	isFinished = 0;
	__HAL_TIM_SetCompare(htimStep, TIM_CHANNEL_1, aHalfOfSpeed);
	
//	while(isFinished != 1) {}
		
	if (HAL_TIM_PWM_Init(htimStep) != HAL_OK)
  {
    return 0;
  }
	return 1;
}

uint8_t stop_Step(TIM_HandleTypeDef* htimStep){
	isFinished = 1;
	__HAL_TIM_SetCompare(htimStep, TIM_CHANNEL_1, 0);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	if (HAL_TIM_PWM_Init(htimStep) != HAL_OK)
  {
    return 0;
  }
	return 1;
}

uint32_t checkTimeout = 0;
uint8_t startGoHome = 0;
uint8_t goHomeStep(TIM_HandleTypeDef* htimStep){
	STEP_Enable();
	if(startGoHome == 0 && HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8) == 0){
		checkTimeout = HAL_GetTick();
		set_StepRotateAngle(720);
//		set_StepSpeed(htimStep, 10); //0.25 vong / 1s
		set_StepSpeed(htimStep,  15); //0.25 vong / 1s
		run_Step(htimStep);
		startGoHome = 1;
	}

	while(1) 
	{
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 1 || HAL_GetTick() - checkTimeout >= 10000 || count >= angle)
			break;
	}

	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 1)
	{
		stop_Step(htimStep);
		startGoHome = 0;
	
		return 1;
	}
	else if (HAL_GetTick() - checkTimeout >= 10000)
	{
		stop_Step(htimStep);
		startGoHome = 0;

		return 0;
	}
	//HAL_Delay(500);
	return 0;
}

uint8_t getStatusFinish(void) {
	return isFinished;
}
