 
#include "motor.h"
#include "main.h"

motor_measure_t motor_chassis[7];
uint8_t chassis_can_send_data[8];	
CAN_TxHeaderTypeDef chassis_tx_message;

//motor data read
#define get_motor_measure(ptr,data)\
{\
	(ptr)->last_ecd=(ptr)->ecd;\
	(ptr)->ecd=(uint16_t)((data)[0]<<8|(data)[1]);\
	(ptr)->speed_rpm=(uint16_t)((data)[2]<<8|(data)[3]);\
	(ptr)->ecd=(uint16_t)((data)[4]<<8|(data)[5]);\
	(ptr)->ecd=(data)[6];\
}
 
 /**
  * @brief          ¡¤¦Ì??¦Ì¡Á?¨¬¦Ì??¨² 3508¦Ì??¨²¨ºy?Y????
  * @param[in]      i: ¦Ì??¨²¡À¨¤o?,¡¤??¡ì[0,3]
  * @retval         ¦Ì??¨²¨ºy?Y????
  */
motor_measure_t* get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
void CAN_cmd_chassis(int16_t M1, int16_t M2, int16_t M3, int16_t M4)
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId=CAN_CHASSIS_ALL_ID;
	chassis_tx_message.IDE=CAN_ID_STD;
	chassis_tx_message.RTR=CAN_RTR_DATA;
	chassis_tx_message.DLC=0x08;
	chassis_can_send_data[0]=M1>>8;
	chassis_can_send_data[1]=M1;
	chassis_can_send_data[2]=M2>>8;
	chassis_can_send_data[3]=M2;
	chassis_can_send_data[4]=M3>>8;
	chassis_can_send_data[5]=M3;
	chassis_can_send_data[6]=M4>>8;
	chassis_can_send_data[7]=M4;
	
	HAL_CAN_AddTxMessage(&hcan1,&chassis_tx_message,chassis_can_send_data,&send_mail_box);
}

/**
  * @brief          send control current of motor
  * @param[in]      motor5: 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor6: 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
void CAN_cmd_auxiliary(int16_t M5, int16_t M6)
{
	uint32_t send_mail_box;
	chassis_tx_message.StdId=CAN_AUXILIARY_ALL_ID;
	chassis_tx_message.IDE=CAN_ID_STD;
	chassis_tx_message.RTR=CAN_RTR_DATA;
	chassis_tx_message.DLC=0x08;
	chassis_can_send_data[0]=M5>>8;
	chassis_can_send_data[1]=M5;
	chassis_can_send_data[2]=M6>>8;
	chassis_can_send_data[3]=M6;
	
	HAL_CAN_AddTxMessage(&hcan1,&chassis_tx_message,chassis_can_send_data,&send_mail_box);
}

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	
	HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&rx_header,rx_data);
	
	switch(rx_header.StdId)
	{
		case motor1:
		case motor2:
		case motor3:
		case motor4:
		case motor6:
		{
			static uint8_t i = 0;
			i = rx_header.StdId - motor1;
			get_motor_measure(&motor_chassis[i],rx_data);
			break;
		}
		
		default:
		{
			break;
		}
	}
}

/**
  * @brief          filter function
  * @retval         none
  */
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;

    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
}

