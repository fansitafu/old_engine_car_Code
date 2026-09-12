/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_extra.cpp
  * Version			: v1.0
  * Author			: PanJiajun
  * Date				: 2025-11-9
  * Description	:	IFR 额外库文件，提供半圈判断、线速度角速度互相转换、rpm和rad/s相互转换、变量限制函数等额外功能
	*
  *********************************************************************
  */
#include "ifr_extra.h"
/*******************************************************************************
 * @概述  处理角度跳变，将目标角度归一化到当前角度附近
 * @参数1 target_angle: 目标角度
 * @参数2 current_angle: 当前角度  
 * @参数3 full_circle: 完整一圈的角度值（如360.0f或2*M_PI）
 * @返回值 归一化后的目标角度，与当前角度差值在±半圈范围内
 *******************************************************************************/
float NormalizeAngleDifference(float target_angle, float current_angle, float full_circle)
{
    float half_circle = full_circle / 2.0f;
    float difference = target_angle - current_angle;
    // 如果差值超过半圈，调整到当前角度附近
    if (difference > half_circle) 
        target_angle -= full_circle;
    else if (difference < -half_circle)
        target_angle += full_circle;
    return target_angle;
}
/*******************************************************************************
 * @概述  角速度转轮子实际转速（线速度）
 * @参数1 angular_speed: 角速度（单位：rad/s）
 * @参数2 radius: 半径（单位：m） 
 * @返回值 轮子转动的线速度（m/s）
 *******************************************************************************/
float angular_to_linear(float angular_speed, float radius)
{
	return angular_speed * radius;
}
/*******************************************************************************
 * @概述  轮子实际转速（线速度）转角速度
 * @参数1 linear_speed: 轮子转动的线速度（m/s）
 * @参数2 radius: 半径（单位：m） 
 * @返回值 角速度（单位：rad/s）
 *******************************************************************************/
float linear_to_angular(float linear_speed, float radius)
{
	return linear_speed / radius;
}
/*******************************************************************************
* @功能     		: 陀螺仪限位计算函数（用于将电机角度的相对坐标系转换为陀螺仪数据的绝对坐标系，如上坡时的pitch轴限位，由于机械限位基于电机角度，但是控制值基于陀螺仪值）
* @参数1        : 电机当前角度
* @参数2        : 陀螺仪当前角度
* @参数3        : 陀螺仪角度低限位
* @参数4        : 陀螺仪角度高限位
* @参数5        : 电机角度低限位
* @参数6        : 电机角度高限位
* @参数7        : 电机角度偏移量（若过0点），若不过0点就填0
* @参数8        : 电机转一圈的角度变化量
* @返回值 			: void
* @概述  				:	用于计算陀螺仪限制，若角度变化会自动计算限位,例如若电机转动角度为7200-8192，0-2000，则偏移量应当写1000，角度低限位7200，高限位2000，一圈8192
*******************************************************************************/
void demarcate(float motor_now_angle, float imu_now_angle, float &IMU_low_limit, float &IMU_high_limit, 
							float motor_low_limit, float motor_high_limit, float motor_angle_offset, float motor_angle_circle)
{
	if (motor_angle_offset != 0)	//若活动范围跨越了0点，需要先进行偏移，然后转换回来，从而让角度值不跨越0点
	{
		motor_now_angle = offest_angle(motor_now_angle, motor_angle_offset, motor_angle_circle);
		motor_low_limit = offest_angle(motor_low_limit, motor_angle_offset, motor_angle_circle);
		motor_high_limit = offest_angle(motor_high_limit, motor_angle_offset, motor_angle_circle);
	}
	IMU_low_limit = imu_now_angle - (motor_now_angle - motor_low_limit) / motor_angle_circle * 360.0f;
	IMU_high_limit = imu_now_angle - (motor_now_angle - motor_high_limit) / motor_angle_circle * 360.0f;
}
/*******************************************************************************
* @功能     		: 角度偏移计算函数
* @参数1        : 电机当前角度
* @参数2        : 电机角度偏移量（若过0点），若不过0点就填0
* @参数3        : 电机转一圈的角度变化量
* @返回值 			: 修改过的角度值
* @概述  				:	此函数用于当电机活动空间中过0点时出现阶跃的问题，例如若电机转动角度为7200-8192，0-2000，则偏移量应当写1000，然后电机角度值范围就会变为8-3000，一圈8192
*******************************************************************************/
float offest_angle(float &now, float offset, float angle_circle)	
{
	now += offset;
	if (now > angle_circle) 
		now -= angle_circle;
	return now;
}
