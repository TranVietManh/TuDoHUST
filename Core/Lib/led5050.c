/*******************************************************************************
** Include 
******************************************************************************/

#include "Led5050.h"

/*******************************************************************************
** Definitions 
******************************************************************************/

/* Declared GPIO LED 5050 */
#define  Red_Led_Port           GPIOB
#define  Red_Led_Pin            GPIO_PIN_14
#define  Green_Led_Port         GPIOB
#define  Green_Led_Pin          GPIO_PIN_15
#define  Blue_Led_Port          GPIOA
#define  Blue_Led_Pin           GPIO_PIN_7


/*******************************************************************************
** Function
******************************************************************************/

/* initial port and pin LED 5050 */
void Led_Init()
{
  
  GPIO_InitTypeDef Init_GPIO_Led = {0};
  
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  
  /*Configure GPIO pin : red_led_pin */
  Init_GPIO_Led.Pin = Red_Led_Pin;
  Init_GPIO_Led.Mode = GPIO_MODE_OUTPUT_PP;
  Init_GPIO_Led.Pull = GPIO_PULLUP;
  Init_GPIO_Led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Red_Led_Port, &Init_GPIO_Led);
  
  /*Configure GPIO pin : green_leb_pin */
  Init_GPIO_Led.Pin = Blue_Led_Pin;
  Init_GPIO_Led.Mode = GPIO_MODE_OUTPUT_PP;
  Init_GPIO_Led.Pull = GPIO_PULLUP;
  Init_GPIO_Led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Blue_Led_Port, &Init_GPIO_Led);
  
  /*Configure GPIO pin : blue_led_pin */
  Init_GPIO_Led.Pin = Green_Led_Pin;
  Init_GPIO_Led.Mode = GPIO_MODE_OUTPUT_PP;
  Init_GPIO_Led.Pull = GPIO_PULLUP;
  Init_GPIO_Led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Green_Led_Port, &Init_GPIO_Led);
  
}

/* Led all off */
void Led_All_Off()
{
  HAL_GPIO_WritePin(Red_Led_Port,    Red_Led_Pin,    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Blue_Led_Port,   Blue_Led_Pin,   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Green_Led_Port,  Green_Led_Pin,  GPIO_PIN_RESET);
}

/* Led red on */
void Led_Red_On()
{
  HAL_GPIO_WritePin(Red_Led_Port,    Red_Led_Pin,    GPIO_PIN_SET);
  HAL_GPIO_WritePin(Blue_Led_Port,   Blue_Led_Pin,   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Green_Led_Port,  Green_Led_Pin,  GPIO_PIN_RESET);
}

/* Led red off */
void Led_Red_Off()
{
  HAL_GPIO_WritePin(Red_Led_Port,    Red_Led_Pin,    GPIO_PIN_RESET);
}

/* Led yellow on */
void Led_Yellow_On()
{
  HAL_GPIO_WritePin(Red_Led_Port,    Red_Led_Pin,    GPIO_PIN_SET);
  HAL_GPIO_WritePin(Blue_Led_Port,   Blue_Led_Pin,   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Green_Led_Port,  Green_Led_Pin,  GPIO_PIN_SET);
}

/* Led yellow off */
void Led_Yellow_Off()
{
  HAL_GPIO_WritePin(Red_Led_Port,    Red_Led_Pin,    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Blue_Led_Port,   Green_Led_Pin,   GPIO_PIN_RESET);
}

/* Led green on */
void Led_Green_On()
{
  HAL_GPIO_WritePin(Red_Led_Port,    Red_Led_Pin,    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Blue_Led_Port,   Blue_Led_Pin,   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Green_Led_Port,  Green_Led_Pin,  GPIO_PIN_SET);
}

/* Led green off */
void Led_Green_Off()
{
  HAL_GPIO_WritePin(Green_Led_Port,  Green_Led_Pin,  GPIO_PIN_RESET);
}

