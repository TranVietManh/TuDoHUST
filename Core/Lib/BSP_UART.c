#include "BSP_UART.h"

uint8_t rx_buffer[8];

UART_HandleTypeDef huart_BSP;
DMA_HandleTypeDef hdma_usart1_rx_BSP;

static void ClearTxBuffer(UART_HandleTypeDef* uart);

/*
Frame truyen tieu chuan kieu char: {T} {D} {Data1} {Data2} {Data3} {Data4} {P} {L} (Gom 8 Bytes trong co co 4 Bytes chua du lieu)
*/
void UART_Init(UART_HandleTypeDef* uart, DMA_HandleTypeDef* dma) {
 
/* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
	__HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	__HAL_RCC_USART1_CLK_ENABLE();

  uart->Instance = USART1;
  uart->Init.BaudRate = 115200;
  uart->Init.WordLength = UART_WORDLENGTH_8B;
  uart->Init.StopBits = UART_STOPBITS_1;
  uart->Init.Parity = UART_PARITY_NONE;
  uart->Init.Mode = UART_MODE_TX_RX;
  uart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart->Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(uart);
	
	__HAL_RCC_DMA1_CLK_ENABLE();

	dma->Instance = DMA1_Channel5;
	dma->Init.Direction = DMA_PERIPH_TO_MEMORY;
	dma->Init.PeriphInc = DMA_PINC_DISABLE;
	dma->Init.MemInc = DMA_MINC_ENABLE;
	dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	dma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	dma->Init.Mode = DMA_CIRCULAR;
	dma->Init.Priority = DMA_PRIORITY_LOW;

	HAL_DMA_Init(dma);

	__HAL_LINKDMA(uart, hdmarx, *dma);
	
	HAL_UART_Receive_DMA(uart, (uint8_t*)rx_buffer, sizeof(rx_buffer));
	
}

void UART_getValue8BytesRxBuffer(uint8_t* __rxBuffer) {
	__rxBuffer[0] = rx_buffer[0];
	__rxBuffer[1] = rx_buffer[1];
	__rxBuffer[2] = rx_buffer[2];
	__rxBuffer[3] = rx_buffer[3];
	__rxBuffer[4] = rx_buffer[4];
	__rxBuffer[5] = rx_buffer[5];
	__rxBuffer[6] = rx_buffer[6];
	__rxBuffer[7] = rx_buffer[7];
}

uint8_t UART_waitForStartSignal(UART_HandleTypeDef* uart) {
	//Kiem tra dau va cuoi cua frame truyen
	if(rx_buffer[0] == 'T' && rx_buffer[1] == 'D' && rx_buffer[6] == 'P' && rx_buffer[7] == 'L') {
		//Kiem tra co dung la du kieu bat dau khong
		if(rx_buffer[2] == '0' && rx_buffer[3] == '1' && rx_buffer[4] == '1' && rx_buffer[5] == '1') {
			return 1;
		}
		else
			//Bao loi gui sai du lieu
			UART_sendErrorInfor(uart);
			return 0;
	}
	else
		return 0;
}

uint8_t UART_waitForStopSignal(UART_HandleTypeDef* uart) {
	//Kiem tra dau va cuoi cua frame truyen
	if(rx_buffer[0] == 'T' && rx_buffer[1] == 'D' && rx_buffer[6] == 'P' && rx_buffer[7] == 'L') {
		//Kiem tra co dung la du kieu ket thuc khong
		if(rx_buffer[2] == '3' && rx_buffer[3] == 'S' && rx_buffer[4] == '1' && rx_buffer[5] == '1') {
			return 1;
		}
		else
			//Bao loi gui sai du lieu
			UART_sendErrorInfor(uart);
			return 0;
	}
	else
		return 0;
}

int8_t UART_waitForClassification(UART_HandleTypeDef* uart) {
	//Kiem tra dau va cuoi cua frame truyen
	if(rx_buffer[0] == 'T' && rx_buffer[1] == 'D' && rx_buffer[6] == 'P' && rx_buffer[7] == 'L') {
		//Kiem tra co dung la du kieu bat dau khong
		if(rx_buffer[2] == '2' && rx_buffer[3] == '0' && rx_buffer[4] == '0' ) {		
			return rx_buffer[5];
		}
		else
			//Bao loi gui sai du lieu
			UART_sendErrorInfor(uart);
			return -1;
	}
	else
		return -1;
}
void UART_sendTakePictureRequest(UART_HandleTypeDef* uart) {
	uint8_t tx_Buffer[8];
	tx_Buffer[0] = 'T';
	tx_Buffer[1] = 'D';
	tx_Buffer[2] = '2';
	tx_Buffer[3] = '1';
	tx_Buffer[4] = '1';
	tx_Buffer[5] = '1';
	tx_Buffer[6] = 'P';
	tx_Buffer[7] = 'L';
	
	HAL_UART_Transmit(uart, tx_Buffer, 8, 20);
}
void UART_sendCompleteStatus(UART_HandleTypeDef* uart, uint8_t soChaiNhua, uint8_t soLon, uint8_t soLanLoi) {
	uint8_t tx_Buffer[8];
	uint8_t charValue;
	tx_Buffer[0] = 'T';
	tx_Buffer[1] = 'D';
	tx_Buffer[2] = '3';
	charValue = soChaiNhua + 30 ; //Chuyen thanh gia tri Ky tu
	tx_Buffer[3] = charValue;
	charValue = soLon + 30 ; //Chuyen thanh gia tri Ky tu
	tx_Buffer[4] = charValue;
	charValue = soLanLoi + 30 ; //Chuyen thanh gia tri Ky tu
	tx_Buffer[5] = charValue;
	tx_Buffer[6] = 'P';
	tx_Buffer[7] = 'L';

	HAL_UART_Transmit(uart, tx_Buffer, 8, 20);
}
void UART_sendErrorInfor(UART_HandleTypeDef* uart) {
	uint8_t tx_Buffer[8];

	tx_Buffer[0] = 'T';
	tx_Buffer[1] = 'D';
	tx_Buffer[2] = '7';
	tx_Buffer[3] = '1';
	tx_Buffer[4] = '1';
	tx_Buffer[5] = '1';
	tx_Buffer[6] = 'P';
	tx_Buffer[7] = 'L';

	HAL_UART_Transmit(uart, tx_Buffer, 8, 20);
}

static void ClearTxBuffer(UART_HandleTypeDef* uart) {
  // Dung truyen và dat lai tr?ng thái cua UART
  HAL_UART_AbortTransmit(uart);

  // Chi cho trang thái truyen tro ve HAL_UART_STATE_READY
  while (HAL_UART_GetState(uart) != HAL_UART_STATE_READY) {
  }
}
