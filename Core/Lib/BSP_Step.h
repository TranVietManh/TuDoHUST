#ifndef __BSP_STEP_H
#define __BSP_STEP_H
#include "stm32f1xx.h"                  // Device header
#include "stm32f1xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

typedef enum{
	CW,
	CCW} rotateDir_t;

void STEP_GPIO_Init(void);
void STEP_GPIO_Deinit(void);
void STEP_PUL_Init(TIM_HandleTypeDef* htimStep,uint32_t speed);

void STEP_Enable(void);
void STEP_Disable(void);

void set_StepRotateAngle(uint32_t angle);
uint8_t set_StepSpeed(TIM_HandleTypeDef* htimStep, double speed);
void change_StepDirect(rotateDir_t dir);

uint8_t run_Step(TIM_HandleTypeDef* htimStep);
uint8_t stop_Step(TIM_HandleTypeDef* htimStep);
uint8_t goHomeStep(TIM_HandleTypeDef* htimStep);
uint8_t getStatusFinish(void);


#endif /* __BSP_STEP_H */