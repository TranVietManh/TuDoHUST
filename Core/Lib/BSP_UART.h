#ifndef __BSP_UART_H_
#define __BSP_UART_H_

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_uart.h"



void UART_Init(UART_HandleTypeDef* uart, DMA_HandleTypeDef* dma);

uint8_t UART_waitForStartSignal(UART_HandleTypeDef* uart);
uint8_t UART_waitForStopSignal(UART_HandleTypeDef* uart);
int8_t UART_waitForClassification(UART_HandleTypeDef* uart);
void UART_sendTakePictureRequest(UART_HandleTypeDef* uart);
void UART_sendCompleteStatus(UART_HandleTypeDef* uart, uint8_t soChaiNhua, uint8_t soLon, uint8_t soLanLoi);
void UART_sendErrorInfor(UART_HandleTypeDef* uart);

void UART_getValue8BytesRxBuffer(uint8_t* __rxBuffer);

#endif //__BSP_UART_H_
