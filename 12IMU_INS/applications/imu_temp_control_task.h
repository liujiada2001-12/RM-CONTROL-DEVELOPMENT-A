#ifndef IMU_TEMP_TASK_H
#define IMU_TEMP_TASK_H


#include "struct_typedef.h"

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2


	
extern void imu_init(void);
extern void imu_read(void);
extern void imu_temp_control(void);
extern void update(fp32 x,fp32 y,fp32 z,fp32 mag_by_filter[3]);//地磁计的中值滤波
extern float AC_Azimuth(fp32 ax,fp32 ay, fp32 az, fp32 mx, fp32 my, fp32 mz);
#endif



