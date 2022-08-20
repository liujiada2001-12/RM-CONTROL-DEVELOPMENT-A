#include "bsp_rc.h"
#include "main.h"
#include "usart.h"
#include "dma.h"
#include "remote_control.h"
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];
void RC_init(void)
{
  HAL_UART_Receive_DMA(&huart1,sbus_rx_buf,18);//开启串口DMA接收中断
}






