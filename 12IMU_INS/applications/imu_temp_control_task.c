
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
#include "AHRS.h"
#include "AHRS_MiddleWare.h"
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm给定

#define TEMPERATURE_PID_KP 1600.0f //kp of temperature control PID 
#define TEMPERATURE_PID_KI 0.2f    //ki of temperature control PID 
#define TEMPERATURE_PID_KD 0.0f    //kd of temperature control PID 

#define TEMPERATURE_PID_MAX_OUT 4500.0f  //max out of temperature control PID 
#define TEMPERATURE_PID_MAX_IOUT 4400.0f //max iout of temperature control PID 


extern SPI_HandleTypeDef hspi5;

volatile uint8_t imu_start_flag = 0;

fp32 gyro[3]= {1.0f, 0.0f, 0.0f}, accel[3]= {0.01f, 0.02f, 9.8f},mag[3]= {0.0f, 0.0f, 1.0f}, temp;
fp32 mag_by_filter[3]={0},gyro_by_filter[3],accel_by_filter[3];
fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.欧拉角 单位 rad
//加速度计低通滤波
fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
//kp, ki,kd three params
const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
//pid struct 
pid_type_def imu_temp_pid;

//陀螺仪校准算法，让陀螺仪静止，求各个的平均值
//加速度校准算法，正放算平均，反放算平均，二者相加取平均
//磁力计校准，各轴转动，取最大最小求平均
fp32 arrg[3];
fp32 gdate[3][1000]={0};
fp32 m_min[3]={0},m_max[3]={0};
fp32 max(fp32 a,fp32 b) {if(a>b) return a;else return b;}
fp32 min(fp32 a,fp32 b) {if(a<b) return a;else return b;}
void dealgdate(fp32 a[3],fp32 arr[3])
{
	fp32 sum=0;
	uint16_t i = 0;
	for(i=1;i<1000;i++)
	{
		gdate[0][i-1]=gdate[0][i];
		gdate[1][i-1]=gdate[1][i];
		gdate[2][i-1]=gdate[2][i];
	}
	gdate[0][999]= a[0];//将新的数据放置到 数据的最后面
	gdate[1][999]= a[1];
	gdate[2][999]= a[2];
	
	for(i=0;i<1000;i++)//求当前数组的合，再取平均值
	{	
		 sum+=gdate[0][i];
	}
	arr[0]=sum/1000;
	sum=0;
	for(i=0;i<1000;i++)//求当前数组的合，再取平均值
	{	
		 sum+=gdate[1][i];
	}
	arr[1]=sum/1000;
	sum=0;
	for(i=0;i<1000;i++)//求当前数组的合，再取平均值
	{	
		 sum+=gdate[2][i];
	}
	arr[2]=sum/1000;
}















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
	AHRS_init(INS_quat, accel, mag);
  get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);

	accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = accel[0];
	accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = accel[1];
	accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = accel[2];
	imu_start_flag = 1;
}
float yaw;

void imu_read(void)
{
	
	mpu6500_read_gyro(gyro);
	mpu6500_read_accel(accel);
	mpu6500_read_temp(&temp);
//	mpu6500_read(gyro,accel,&temp);

////	ist8310_read_mag(mag);
	//校准程序
	gyro[0]=(gyro[0]-(-0.0665913522f));
	gyro[1]=(gyro[1]-(0.00611032126f));
	gyro[2]=(gyro[2]-(-0.0305260234f));
//	dealgdate(gyro,arrg);
	accel[0]=accel[0]-0.5f*(-0.420f+0.102f)+0.248f;
	accel[1]=accel[1]-0.5f*(0.377f+0.128f)+0.121f;
	accel[2]=accel[2]-0.5f*(-9.53f+10.04f)+0.03f;
//	dealgdate(accel,arrg);
//	if(m_max[0])
//	{
//		m_max[0]=max(m_max[0],mag[0]);
//		m_min[0]=min(m_min[0],mag[0]);
//		m_max[1]=max(m_max[1],mag[1]);
//		m_min[1]=min(m_min[1],mag[1]);
//		m_max[2]=max(m_max[0],mag[2]);
//		m_min[2]=min(m_min[0],mag[2]);
//	}
//	else
//	{
//		m_max[0]=mag[0];
//		m_min[0]=mag[0];
//		m_max[1]=mag[1];
//		m_min[1]=mag[1];
//		m_max[2]=mag[2];
//		m_min[2]=mag[2];
//	}
//	
//	mag[0]=mag[0]-0.5f*(9753.90039f-9830.40039f);
//	mag[1]=mag[1]-0.5f*(9830.10059f-9753.90039f);
//	mag[2]=mag[2]-0.5f*(9753.90039f-9830.40039f);
//	update(mag[0],mag[1],mag[2],mag_by_filter);
	accel[2]=accel[2]-9.8f;
	//加速度计低通滤波
  accel_fliter_1[0] = accel_fliter_2[0];
  accel_fliter_2[0] = accel_fliter_3[0];

  accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + accel[0] * fliter_num[2];

  accel_fliter_1[1] = accel_fliter_2[1];
  accel_fliter_2[1] = accel_fliter_3[1];

  accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + accel[1] * fliter_num[2];

  accel_fliter_1[2] = accel_fliter_2[2];
  accel_fliter_2[2] = accel_fliter_3[2];

  accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + accel[2] * fliter_num[2];
	
	AHRS_update(INS_quat, 0.001f, gyro, accel_fliter_3,0);
  get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
//	ist8310_read_mag(mag);
//	update(mag[0],mag[1],mag[2],mag_by_filter);
//	yaw=AC_Azimuth(accel_fliter_3[0], accel_fliter_3[1], accel_fliter_3[2], mag_by_filter[0],mag_by_filter[1],mag_by_filter[2]);		
}
void imu_temp_control(void)
{
	uint16_t tempPWM;
        //pid calculate. PID计算
				if(temp<40.0f)
				{
					imu_temp_pid.out=imu_temp_pid.max_out;
				}
				else
				{
					PID_calc(&imu_temp_pid, temp, 40.0f);
				}
        if (imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
}

//六轴获取水平方位角


//计算旋转角
static float getYaw(fp32 accVals[3], fp32 magVals[3]) 
{
	float roll = (float)atan2(accVals[0],accVals[2]);
	float pitch = -(float)atan(accVals[1]/(accVals[0]*sin(roll)+accVals[2]*cos(roll)));
	float yaw = (float)atan2(magVals[0]*sin(roll)*sin(pitch)+magVals[2]*cos(roll)*sin(pitch)+magVals[1]*cos(pitch),
		   magVals[0]*cos(roll)-magVals[2]*sin(roll));
	return yaw;
}
 
 
/*************************************************************************************************************************
* 函数			:	float AC_Azimuth(s16 ax,s16 ay, s16 az, s16 mx, s16 my, s16 mz)
* 功能			:	通过加速度传感器与磁力计计算方位角
* 参数			:	ax,ay,az:xyz方向加速度原始值;mx,my,mz:xyz方向的磁场强度原始值
* 返回			:	扩大10倍（正北为0）
* 依赖			:	底层读写函数
* 作者			:	cp1300@139.com
* 时间			:	2020-04-28
* 最后修改时间 	: 	2020-04-28
* 说明			: 	
*************************************************************************************************************************/ 
float AC_Azimuth(fp32 ax,fp32 ay, fp32 az, fp32 mx, fp32 my, fp32 mz)
{
	float accVals[3], magVals[3];
	float ftemp;
	
	accVals[0] = ax;
	accVals[1] = az;
	accVals[2] = ay;
	magVals[0] = mx;
	magVals[1] = mz;
	magVals[2] = my;
			
	ftemp = getYaw(accVals, magVals) * 180.0f / 3.141593f;
	if(ftemp > 0) ftemp = -180.0f + (ftemp - 180.0f) ;
	ftemp = 0.0f - ftemp;
	ftemp += 90.0f;
	ftemp += -2.0f;	//补偿磁偏角，不同地区会不一样
	if(ftemp > 360.0f) ftemp -= 360.0f;
	return ftemp;	
}


fp32 IST[3][11]={0};//[0]-[9]为最近10次数据 [10]为10次数据的平均值 注：磁传感器的采样频率慢，所以单独列出
typedef __packed struct
{
	fp32 MaxMagX;
	fp32 MaxMagY;
	fp32 MaxMagZ;
	fp32 MinMagX;
	fp32 MinMagY;
	fp32 MinMagZ;
}MagMaxMinData_t;
MagMaxMinData_t MagMaxMinData;

void update(fp32 x,fp32 y,fp32 z,fp32 mag_by_filter[3])//地磁计的中值滤波
{
		uint8_t i = 0;
	fp32 sum=0;

	for(i=1;i<10;i++)
	{
		IST[0][i-1]=IST[0][i];
		IST[1][i-1]=IST[1][i];
		IST[2][i-1]=IST[2][i];
	}
	IST[0][9]= x;//将新的数据放置到 数据的最后面
	IST[1][9]= y;
	IST[2][9]= z;
	
	for(i=0;i<10;i++)//求当前数组的合，再取平均值
	{	
		 sum+=IST[0][i];
	}
	IST[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=IST[1][i];
	}
	IST[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=IST[2][i];
	}
	IST[2][10]=sum/10;
	mag_by_filter[0]=IST[0][10];
	mag_by_filter[1]=IST[1][10];
	mag_by_filter[2]=IST[2][10];
	//printf (" %d  %d  %d ",arrx,arry,arrz);
	//以上全部为未校准数据
	/************用于偏差校正，见官方开源，暂时无用的函数*******************/
	if(MagMaxMinData.MinMagX>IST[0][10])
	{
		MagMaxMinData.MinMagX=(int16_t)IST[0][10];
	}
	if(MagMaxMinData.MinMagY>IST[1][10])
	{
		MagMaxMinData.MinMagY=(int16_t)IST[1][10];
	}
	if(MagMaxMinData.MinMagZ>IST[2][10])
	{
		MagMaxMinData.MinMagZ=(int16_t)IST[2][10];
	}

	if(MagMaxMinData.MaxMagX<IST[0][10])
	{
		MagMaxMinData.MaxMagX=(int16_t)IST[0][10];		
	}
	if(MagMaxMinData.MaxMagY<IST[1][10])
	{
		MagMaxMinData.MaxMagY = IST[1][10];
	}
	if(MagMaxMinData.MaxMagZ<IST[2][10])
	{
		MagMaxMinData.MaxMagZ=(int16_t)IST[2][10];
	}
	
	/************无用的函数*******************/
 //printf (" %d  %d  %d      %d  %d  %d\n",MagMaxMinData.MinMagX,MagMaxMinData.MinMagY,MagMaxMinData.MinMagZ   ,MagMaxMinData.MaxMagX,MagMaxMinData.MaxMagY,MagMaxMinData.MaxMagZ);	
}
