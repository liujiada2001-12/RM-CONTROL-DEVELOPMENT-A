
#include "imu_temp_control_task.h"
#include "main.h"
#include "pid.h"
#include "bsp_imu_pwm.h"
#include "struct_typedef.h"
#include "ist8310driver.h"
#include "ist8310driver_middleware.h"
#include "mpu6500driver.h"
#include "mpu6500driver_middleware.h"
#include "mpu6500reg.h"
#include "tim.h"
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm¸ø¶¨

#define TEMPERATURE_PID_KP 1600.0f //kp of temperature control PID 
#define TEMPERATURE_PID_KI 0.2f    //ki of temperature control PID 
#define TEMPERATURE_PID_KD 0.0f    //kd of temperature control PID 

#define TEMPERATURE_PID_MAX_OUT 4500.0f  //max out of temperature control PID 
#define TEMPERATURE_PID_MAX_IOUT 4400.0f //max iout of temperature control PID 


extern SPI_HandleTypeDef hspi5;

volatile uint8_t imu_start_flag = 0;

fp32 gyro[3], accel[3],mag[3], temp;

//kp, ki,kd three params
const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
//pid struct 
pid_type_def imu_temp_pid;

void imu_init(void)
{
	HAL_TIM_Base_Start(&htim10);
HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
	while(mpu6500_init())
	{
		;
	}
	while(ist8310_init())
	{
		;
	}
	PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
	imu_start_flag = 1;
}
void imu_read(void)
{
	mpu6500_read_gyro(gyro);
	mpu6500_read_accel(accel);
	mpu6500_read_temp(&temp);
	ist8310_read_mag(mag);
}
void imu_temp_control(void)
{
	uint16_t tempPWM;
        //pid calculate. PID¼ÆËã
        PID_calc(&imu_temp_pid, temp, 40.0f);
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
}


