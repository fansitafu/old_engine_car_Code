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
#ifndef __IFR_PID_H_
#define __IFR_PID_H_
/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
#ifdef __cplusplus
 }
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
#define IFR_PI 3.14159265358979323846f
#define IFR_ABS(x) ((x)>0? (x):(-(x)))
#define IFR_CLAMP(val, min, max) \
    ((val) < (min) ? (min) : ((val) > (max) ? (max) : (val)))
class IFR_PID
{
	public:
		void  PID_Init(float kp,float ki,float kd,float output_max,float error_max,float integral_max,float dead_zone, float integral_Separation_Threshold, float diff_alpha);
		float Positional_PID(float Target_Value,float Actual_Value);
		float Incremental_PID(float Target_Value,float Actual_Value);
		void OutputMax_Set(float output_max);
		float Output_Get(void);
		float Kp;
		float Ki;
		float Kd;
	private:
		float Error;
		float Error_Max;
		float Integral;
		float Integral_Max;
		float Diffrential;
		float Dead_Zone;
		float Last_Error;
		float Last_Last_Error;
		float Output;
		float Output_Max;
		float Integral_Separation_Threshold;
		float Diff_filtered;
		float Diff_alpha;
};
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */
/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */
#endif
