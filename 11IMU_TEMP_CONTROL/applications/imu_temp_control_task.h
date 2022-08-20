#ifndef IMU_TEMP_TASK_H
#define IMU_TEMP_TASK_H


#include "struct_typedef.h"



/**
  * @brief          bmi088 temperature control 
  * @param[in]      argument: NULL
  * @retval         none
  */
/**
  * @brief          bmi088ÎÂ¶È¿ØÖÆ
  * @param[in]      argument: NULL
  * @retval         none
  */
extern void imu_init(void);
extern void imu_read(void);
extern void imu_temp_control(void);
#endif



