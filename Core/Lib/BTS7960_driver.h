#ifndef _BTS_7960_ 
#define _BTS_7960_

#include <stm32f1xx.h>

#define driver_RPWM_PINA_DC1 GPIO_PIN_4 
#define driver_LPWM_PINA_DC1 GPIO_PIN_5 
#define driver_RPWM_PINA_DC2 GPIO_PIN_8 
#define driver_LPWM_PINB_DC2 GPIO_PIN_0 
#define driver_LREN_PINA_DC12 GPIO_PIN_11
#define DC_TIMEOUT 15000
extern TIM_HandleTypeDef htim1; 

typedef enum {
    CLOCKWISE,
    COUNTERCLOCKWISE
} MotorDirection;

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void DC_Driver_PWM_Init(void);  
void DC_Driver_SetSpeed(uint16_t speed);
void DC_Driver_SetDir(MotorDirection direction);  
void DC_Driver_Stop(void);
void DC_Driver_Run(void); 
void DC_Driver_GPIO_Init(void);
uint8_t DC_Driver_State(void);
#endif