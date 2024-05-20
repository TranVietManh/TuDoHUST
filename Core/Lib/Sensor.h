#ifndef __SENSOR_H
#define __SENSOR_H

/*******************************************************************************
** Include 
*******************************************************************************/

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

/*******************************************************************************
** Prototype 
*******************************************************************************/

/**
  * @brief    Init GPIO for senser
  * @param    None
  * @retval   None
  */
	
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
	
void Init_Sensor();

/**
  * @brief    DeInit GPIO for senser
  * @param    None
  * @retval   None
  */
void Deinit_Sensor();

/**
  * @brief    Read value of sensor to detect obstacles
  * @param    channel         1 || 2 || 3 || 4 for 4 sensor
  * @retval   1 - Have obstacles, 0 - Haven't obstacles
  */
uint8_t Read_Sensor(uint8_t channel);

float Read_ADC_Value(uint8_t CHANNEL); 

#endif /* __SENSOR_H */

