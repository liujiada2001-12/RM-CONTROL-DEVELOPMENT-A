/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "remote_control.h"

#include "main.h"


extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_rx_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_rx_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */
static void sbus_to_rc(volatile const uint8_t *sbus_rx_buf, RC_ctrl_t *rc_ctrl);

//remote control data 
//遥控器控制变量
RC_ctrl_t rc_ctrl;

//receive data, 18 bytes one frame
//接收原始数据，为18个字节
uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];


/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}


//串口接收中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if(huart->Instance==USART1)
	{
   rc_ctrl.rc.ch[0] = (sbus_rx_buf[0] | (sbus_rx_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl.rc.ch[1] = ((sbus_rx_buf[1] >> 3) | (sbus_rx_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl.rc.ch[2] = ((sbus_rx_buf[2] >> 6) | (sbus_rx_buf[3] << 2) |(sbus_rx_buf[4] << 10)) &0x07ff;  //!< Channel 2
    rc_ctrl.rc.ch[3] = ((sbus_rx_buf[4] >> 1) | (sbus_rx_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl.rc.s[0] = ((sbus_rx_buf[5] >> 4) & 0x0003);                  //!< Switch left
    rc_ctrl.rc.s[1] = ((sbus_rx_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
    rc_ctrl.mouse.x = sbus_rx_buf[6] | (sbus_rx_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl.mouse.y = sbus_rx_buf[8] | (sbus_rx_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl.mouse.z = sbus_rx_buf[10] | (sbus_rx_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl.mouse.press_l = sbus_rx_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl.mouse.press_r = sbus_rx_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl.key.v = sbus_rx_buf[14] | (sbus_rx_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl.rc.ch[4] = sbus_rx_buf[16] | (sbus_rx_buf[17] << 8);                 //NULL

    rc_ctrl.rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl.rc.ch[4] -= RC_CH_VALUE_OFFSET;
  }
}
/**
  * @brief          remote control protocol resolution
  * @param[in]      sbus_rx_buf: raw data point
  * @param[out]     rc_ctrl: remote control data struct point
  * @retval         none
  */
/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_rx_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指
  * @retval         none
  */


