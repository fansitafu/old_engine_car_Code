#ifndef __IFR_EXTRA_H_
#define __IFR_EXTRA_H_
/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_extra.h
  * Version			: v1.0
  * Author			: PanJiajun
  * Date				: 2025-11-9
  * Description	:	IFR 额外库文件，提供半圈判断、变量限制函数等额外功能
	*
  *********************************************************************
  */
#include "main.h"
#ifdef __cplusplus
 extern "C" {
#endif
#ifdef __cplusplus
 }
#endif
#include "ifr_can.h"
#include "ifr_fdcan.h"
#include "math.h"
#include "ifr_key.h" 
/*******************************************************************************
* @功能     		: 限制变量数值函数（模版函数）（适用于所有的变量类型，但传入的三个变量必须是同一种，有类型错误的保护）
* @参数1        : 当前值
* @参数2        : 最小值
* @参数3        : 最大值
* @返回值 			: 输入的数据类型
* @概述  				:	若小于最小值则输出最小值，若大于最大值则输出最大值
*******************************************************************************/
template < typename T >
const T& clamp_num(const T& value, const T& min, const T& max) 
{
	if (value < min) 
			return min;
	else if (value > max) 
			return max;
	return value;
} 
/*******************************************************************************
* @功能     		: 限制变量数值函数（宏定义函数）（适用于所有的变量类型，但没有类型错误的保护操作，且不能使用一个正在被++的数传入）
* @参数1        : 当前值
* @参数2        : 最小值
* @参数3        : 最大值
* @返回值 			: 输入的数据类型
* @概述  				:	若小于最小值则输出最小值，若大于最大值则输出最大值
*******************************************************************************/
#define CLAMP_NUM(value, min, max) \
    (((value) < (min)) ? (min) : (((value) > (max)) ? (max) : (value)))
/*******************************************************************************
* @功能     		: rpm转rad/s（宏定义函数）
* @参数1        : rpm单位下的速度
* @返回值 			: 输入的数据类型
* @概述  				:	输出rad/s单位下的速度 1 rpm = 0.10471 rad/s
*******************************************************************************/
#define RPM_TO_RADS(rpm) ((rpm) * IFR_PI * 2.0f / 60.0f)
/*******************************************************************************
* @功能     		: rad/s转rpm（宏定义函数）
* @参数1        : rad/s单位下的速度
* @返回值 			: 输入的数据类型
* @概述  				:	输出rpm单位下的速度，1 rad/s = 9.5492965 rpm
*******************************************************************************/
#define RADS_TO_RPM(rads) ((rads) * 60.0f / (2.0f * IFR_PI))
void demarcate(float motor_now_angle, float imu_now_angle, float &IMU_low_limit, float &IMU_high_limit, 
					float motor_low_limit, float motor_high_limit, float motor_angle_offset, float motor_angle_circle);
float offest_angle(float &now, float offset, float angle_circle);
float NormalizeAngleDifference(float target_angle, float current_angle, float full_circle);
float angular_to_linear(float angular_speed, float radius);
float linear_to_angular(float linear_speed, float radius);
#endif
