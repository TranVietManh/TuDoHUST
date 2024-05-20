#ifndef _LED5050_H_
#define _LED5050_H_

/*******************************************************************************
** Include 
*******************************************************************************/

#include "stm32f1xx_hal.h"

/*******************************************************************************
* Port
*******************************************************************************/

/* 
-------------------------------------------------------------------------------

red_led_pin       |   B14
green_leb_pin     |   B15
blue_led_pin      |   A7

Examble: led_init(GPIOB, GPIO_PIN_14, GPIOB, GPIO_PIN_15, GPIOA, GPIO_PIN_7);
--------------------------------------------------------------------------------
*/

/*******************************************************************************
* Prototypes
*******************************************************************************/
/**
  * @brief GPIO Initialization Function For Led 5050
  * @param red_led_port           port for Red
  * @param red_led_pin            pin for Red
  * @param green_led_port         port for Green
  * @param green_led_pin          pin for green
  * @param blue_led_port          port for Blue
  * @param blue_led_pin           pin for Blue
  * @retval None
  */
void Led_Init();

/**
  * @brief Led All Off
  * @param None
  * @retval None
  */
void Led_All_Off();

/**
  * @brief Led Red On
  * @param None
  * @retval None
  */
void Led_Red_On(void);

/**
  * @brief Led Red Off
  * @param None
  * @retval None
  */
void Led_Red_Off(void);

/**
  * @brief Led Yellow On
  * @param None
  * @retval None
  */
void Led_Yellow_On(void);

/**
  * @brief Led Yellow Off
  * @param None
  * @retval None
  */
void Led_Yellow_Off(void);

/**
  * @brief Led Green On
  * @param None
  * @retval None
  */
void Led_Green_On(void);

/**
  * @brief Led Green Off
  * @param None
  * @retval None
  */
void Led_Green_Off(void);

#endif /* _LED5050_H_ */