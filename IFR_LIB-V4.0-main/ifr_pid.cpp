/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_pid.cpp
  * Version		: v3.0
  * Author		: LiuHao Lijiawei Albert panjiajun
  * Date			: 2025-10-30
  * Description	: IFR库中的PID算法组件层 提供带微分先行、不完全微分、输出限制、积分分离算法的 位置式PID和增量式PID
  *********************************************************************
  */
/* Includes ---------------------------------------------------------*/
#include "ifr_pid.h"
/* Private variables -------------------------------------------------*/
/**
  * @概述	PID参数初始化。
  * @参数1	Kp
  * @参数2	Ki
  * @参数3	Kd
  * @参数4	输出最大值
  * @参数5	误差最大值
  * @参数6	积分最大值
  * @参数7	死区
  * @参数8	积分分离阈值（误差超过这个积分就不再累加）
  * @参数9	不完全微分系数（建议值在0.1-0.3之间），用于抑制电机反馈值的高频噪声，如若此系数为0.1，则意味着微分的频率（累加速度）将会是十分之一，若为0.33，则为三分之一，若为1，则无滤波
  * @返回值 void
  */
void IFR_PID::PID_Init(float kp, float ki, float kd, float output_max, float error_max, float integral_max, float dead_zone, float integral_Separation_Threshold, float diff_alpha)
{
	Kp = kp; Ki = ki; Kd = kd;
	Output_Max = output_max;
	Error_Max = error_max;
	Integral_Max = integral_max;
	Dead_Zone = dead_zone;
	Integral_Separation_Threshold = integral_Separation_Threshold;
	Diff_alpha = diff_alpha;
}
/**
  * @概述	位置式PID计算函数，带有积分分离、不完全微分等额外算法
  * @参数1	目标值
  * @参数2	现在值
  * @返回值 PID计算的输出
  */
float IFR_PID::Positional_PID(float Target_Value, float Actual_Value)
{
    // 误差计算与限幅
    Error = Target_Value - Actual_Value;
    Error = IFR_CLAMP(Error, -Error_Max, Error_Max);
    if (IFR_ABS(Error) <= Dead_Zone) Error = 0;
    // 微分计算
		Diffrential = Error - Last_Error;
		//不完全微分：微分项低通滤波（抑制高频噪声）
    Diff_filtered = Diff_alpha * Diffrential + (1 - Diff_alpha) * Diff_filtered;
    // 积分分离策略
    if (IFR_ABS(Error) >= Integral_Separation_Threshold) 
		{
        // 大误差区：仅用PD控制
        Output = Kp * Error + Kd * Diff_filtered;
        Integral = 0; // 大误差时清零积分，避免累积过大
    } 
		else 
		{
        // 小误差区：启用变积分控制
        // 变积分系数计算：误差越小，积分系数越大（0~1之间）
        float ki_adjust = 1.0f - (IFR_ABS(Error) / Integral_Separation_Threshold);
        ki_adjust = IFR_CLAMP(ki_adjust, 0.0f, 1.0f); // 确保系数在有效范围
        // 积分累积（乘以变积分系数）
        Integral += Error * ki_adjust;
        Integral = IFR_CLAMP(Integral, -Integral_Max, Integral_Max);
        // 计算输出（积分项已含变积分系数）
        Output = Kp * Error + Ki * Integral + Kd * Diff_filtered;
    }
    // 输出限幅
    Output = IFR_CLAMP(Output, -Output_Max, Output_Max);
    Last_Error = Error;
    return Output;
}
/**
  * @概述	增量式PID计算函数。
  * @参数1	目标值
  * @参数2	现在值
  * @返回值 PID计算的输出
  */
float IFR_PID::Incremental_PID(float Target_Value, float Actual_Value)
{
    Error = Target_Value - Actual_Value;
    // 误差限幅
    Error = IFR_CLAMP(Error, -Error_Max, Error_Max);
    if (IFR_ABS(Error) <= Dead_Zone) Error = 0;
    // 增量计算（标准形式）
    float delta = Kp * (Error - Last_Error) + 
                 Ki * Error + 
                 Kd * (Error - 2 * Last_Error + Last_Last_Error);
    // 增量限幅（防止突变）
    const float Delta_Max = Output_Max * 0.3f; // 单步最大变化量
    delta = IFR_CLAMP(delta, -Delta_Max, Delta_Max);
    Output += delta;
    // 总输出限幅
    Output = IFR_CLAMP(Output, -Output_Max, Output_Max);
    // 更新历史误差
    Last_Last_Error = Last_Error;
    Last_Error = Error;
    return Output;
}
/**
  * @概述 	PID最大输出设置
  * @参数1	最大输出设置值
  * @返回值 void
  */
void IFR_PID::OutputMax_Set(float output_max)
{
	Output_Max = output_max;
}
/**
  * @概述		获取输出值
  * @返回值 输出值(float)
  */
float IFR_PID::Output_Get(void)
{
	return Output;
}
