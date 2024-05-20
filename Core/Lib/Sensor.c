/*******************************************************************************
** Include 
*******************************************************************************/

#include "Sensor.h"

/*******************************************************************************
** Function 
*******************************************************************************/

uint16_t adc_value = 0;
float Vin = 0;

void Deinit_Sensor()
{
HAL_GPIO_DeInit(GPIOA,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
//HAL_ADC_DeInit(&hadc1);
//HAL_DMA_DeInit(&hdma_adc1);
}

float Read_ADC_Value(uint8_t CHANNEL) {
	uint8_t numberOfConversion = 0;
	float accumulatedSum = 0;
	if (CHANNEL == 1) {
		while(numberOfConversion <= 20){
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 300);
			adc_value = HAL_ADC_GetValue(&hadc1);
			Vin = (float)(adc_value/4095.00)*3.3;
			accumulatedSum += Vin;
			HAL_ADC_Stop(&hadc1);
			numberOfConversion++;
		}
	}
	if (CHANNEL == 2) {
		while(numberOfConversion <= 20) {
			HAL_ADC_Start(&hadc2);
			HAL_ADC_PollForConversion(&hadc2, 300);
			adc_value = HAL_ADC_GetValue(&hadc2);
			Vin = (float)(adc_value/4095.00)*3.3;
			HAL_ADC_Stop(&hadc2);
			accumulatedSum += Vin;
			numberOfConversion++;
		}
	}
	
//	if(accumulatedSum/numberOfConversion < 1) {
//		Vin = 0;
//	} else if(accumulatedSum/numberOfConversion >= 1 && accumulatedSum/numberOfConversion < 1.5){
//		Vin = 1;
//	} else if(accumulatedSum/numberOfConversion >= 1.5 && accumulatedSum/numberOfConversion < 2) {
//		Vin = 1.5;
//	} else if (accumulatedSum/numberOfConversion >= 2 && accumulatedSum/numberOfConversion < 2.5) {
//		Vin = 2;
//	} else if (accumulatedSum/numberOfConversion >= 2.5 && accumulatedSum/numberOfConversion < 3) {
//		Vin = 2.5;
//	} else if (accumulatedSum/numberOfConversion >= 3 && accumulatedSum/numberOfConversion < 3.5) {
//		Vin = 3;
//	}
	return accumulatedSum/numberOfConversion;
} 


uint8_t Read_Sensor(uint8_t channel)
{
uint8_t stt_ss;	
	switch(channel)
	{
		case 1:
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3) == GPIO_PIN_SET)
			{
			stt_ss = 1;
			}
			else 
			{
			stt_ss = 0;
			}
		break;
		case 2:
			if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2)== GPIO_PIN_SET)
			{
			stt_ss = 1;
			}
			else 
			{
			stt_ss = 0;
			}
		break;
		case 3:
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_7)== GPIO_PIN_SET)
			{
			stt_ss = 1;
			}
			else 
			{
			stt_ss = 0;
			}
			break;
		case 4:
			if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8) == GPIO_PIN_SET)
			{
			stt_ss = 1;
			}
			else 
			{
			stt_ss = 0;
			}
			break;
	}
//uint8_t val = 0;
//	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)Adc_r, 4); 
return stt_ss;
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void Init_Sensor(void)
{
GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */

}


