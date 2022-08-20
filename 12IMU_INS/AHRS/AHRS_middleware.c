/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       AHRS_MiddleWare.c/h
  * @brief      ��̬�����м�㣬Ϊ��̬�����ṩ��غ���
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "MahonyAHRS.h"
#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "main.h"
#include "math.h"
/**
  * @brief          ���ڻ�ȡ��ǰ�߶�
  * @author         RM
  * @param[in]      �߶ȵ�ָ�룬fp32
  * @retval         ���ؿ�
  */

void AHRS_get_height(fp32 *high)
{
    if (high != NULL)
    {
        *high = 0.0f;
    }
}

/**
  * @brief          ���ڻ�ȡ��ǰγ��
  * @author         RM
  * @param[in]      γ�ȵ�ָ�룬fp32
  * @retval         ���ؿ�
  */

void AHRS_get_latitude(fp32 *latitude)
{
    if (latitude != NULL)
    {
        *latitude = Latitude;
    }
}

/**
  * @brief          ���ٿ���������
  * @author         RM
  * @param[in]      ������Ҫ�����ĸ�������fp32
  * @retval         ����1/sqrt ������ĵ���
  */

fp32 AHRS_invSqrt(fp32 num)
{
    fp32 halfnum = 0.5f * num;
    fp32 y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(fp32 *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
  * @brief          sin����
  * @author         RM
  * @param[in]      �Ƕ� ��λ rad
  * @retval         ���ض�Ӧ�Ƕȵ�sinֵ
  */

fp32 AHRS_sinf(fp32 angle)
{
    return sin(angle);
}
/**
  * @brief          cos����
  * @author         RM
  * @param[in]      �Ƕ� ��λ rad
  * @retval         ���ض�Ӧ�Ƕȵ�cosֵ
  */

fp32 AHRS_cosf(fp32 angle)
{
    return cos(angle);
}

/**
  * @brief          tan����
  * @author         RM
  * @param[in]      �Ƕ� ��λ rad
  * @retval         ���ض�Ӧ�Ƕȵ�tanֵ
  */

fp32 AHRS_tanf(fp32 angle)
{
    return tanf(angle);
}
/**
  * @brief          ����32λ�������ķ����Ǻ��� asin����
  * @author         RM
  * @param[in]      ����sinֵ�����1.0f����С-1.0f
  * @retval         ���ؽǶ� ��λ����
  */

fp32 AHRS_asinf(fp32 sin)
{

    return asinf(sin);
}

/**
  * @brief          �����Ǻ���acos����
  * @author         RM
  * @param[in]      ����cosֵ�����1.0f����С-1.0f
  * @retval         ���ض�Ӧ�ĽǶ� ��λ����
  */

fp32 AHRS_acosf(fp32 cos)
{

    return acosf(cos);
}

/**
  * @brief          �����Ǻ���atan����
  * @author         RM
  * @param[in]      ����tanֵ�е�yֵ ����������С������
  * @param[in]      ����tanֵ�е�xֵ ����������С������
  * @retval         ���ض�Ӧ�ĽǶ� ��λ����
  */

fp32 AHRS_atan2f(fp32 y, fp32 x)
{
    return atan2f(y, x);
}

void AHRS_init(fp32 quat[4], const fp32 accel[3], const fp32 mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;

}

void AHRS_update(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}
void get_angle(const fp32 q[4], fp32 *yaw, fp32 *pitch, fp32 *roll)
{
//    *yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 2.0f*(q[0]*q[0]+q[1]*q[1])-1.0f) *RAD_TO_ANGLE;
//    *pitch = asinf(-2.0f*(q[1]*q[3]-q[0]*q[2])) *RAD_TO_ANGLE;
//    *roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]),2.0f*(q[0]*q[0]+q[3]*q[3])-1.0f) *RAD_TO_ANGLE;
		*pitch = asin(-2 * q[1] * q[3] + 2 * q[0]* q[2])* 57.3;	// pitch
		*roll  = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2]* q[2] + 1)* 57.3;	// roll
		*yaw   = atan2(2*(q[1]*q[2] + q[0]*q[3]),q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]) * 57.3;	//yaw
}



