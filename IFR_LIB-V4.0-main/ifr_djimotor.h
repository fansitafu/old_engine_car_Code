/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_djimotor.h
  * Version			: v3.0
  * Author			: PanJiajun 
  * Date				: 2025-10-10
  * Description	:	DJI系列电机控制库，包含电机初始化、CAN消息解析、扭矩/速度/位置控制等核心功能，IFR CAN库中的 设备层，负责具体电机控制
	*
  *********************************************************************
  */
#ifndef __IFR_DJIMOTOR_H_
#define __IFR_DJIMOTOR_H_
/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
#ifdef __cplusplus
 }
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ifr_pid.h"
#include "ifr_basic_motor.h"
#include "ifr_can_msg_queue.h"
 /**
 * @brief   DJI电机型号枚举（若需要添加新的电机则在这里添加）
 */
typedef enum
{
	DJI_2006,
	DJI_3508,
	DJI_6020,
} DJI_Motor_Type;	// 电机型号

/**
 * @brief   DJI电机错误码枚举（数值越小，错误级别越高，多错误时优先反馈小数值）
 */
typedef enum
{
	DJI_MOTOR_NO_ERROR = 0,                // 0：无异常
	DJI_MOTOR_UNABLE_ACCESS,               // 1：无法访问电机中的存储芯片（仅开机自检）
	DJI_MOTOR_VOLTAGE_HIGH,                // 2：电调供电电压过高（仅开机自检）
	DJI_MOTOR_THREE_PHASE_UNCONNECTED,     // 3：电机三相线未接入
	DJI_MOTOR_POS_SENSOR_DATA_LOST,        // 4：与电机相连的数据线中位置传感器数据丢失
	DJI_MOTOR_TEMP_ABNORMAL_HIGH,          // 5：电机温度异常或过高 (≥180℃)
	DJI_MOTOR_CALIBRATION_FAILED = 7,      // 7：电机校准失败（注：无错误码6）
	DJI_MOTOR_TEMP_OVERHEAT = 8            // 8：电机过热 (≥125℃)
} DJI_Motor_Error_Code;                    // 电机报错码

// 电机电流环基类
class IFR_DJI_Motor : public IFR_Basic_Motor
{
	public:
		IFR_DJI_Motor(uint16_t Num, DJI_Motor_Type m_DJI_Motor_type, CanMsgQueue* MsgQueue);
		IFR_DJI_Motor(float (*Offset_MotoFunc)(float Motor_Angle), uint16_t Num, DJI_Motor_Type m_DJI_Motor_type, CanMsgQueue* MsgQueue);
		float Get_DJIMotor_Electrict(void) {return tar_electric;}
		void set_electric(float set_output);
		void set_electric_offset(float _set_electric);
		DJI_Motor_Error_Code get_error_code() { return motor_error_code; }
		virtual void Enable() { return; }
		virtual void Disable(uint8_t clear_error) { return; }
	protected:
		virtual void parse_can_msg(const CanMsg_t msg);
		virtual void generate_can_msg(CanMsg_t& msg);
		void motor_init(uint16_t Num, DJI_Motor_Type m_DJI_Motor_type, CanMsgQueue* MsgQueue);
		void set_motor_error_code(const uint8_t error_data);
		float tar_electric;				// 目标电流
		CanMsgQueue* msgQueue;			// 消息队列指针
		float int16_to_float;			// 不同电机int16下 1对应的数值不同，此为比例参数
		float electric_max;				// 最大电流限制（6020:3， 2006:10， 3508:20）
		float pos_offest;					// 速度前馈系数（由目标电机位置和上一刻的差产生）
		DJI_Motor_Error_Code motor_error_code;	// 电机反馈的错误码
};
 // 电机速度环类
class IFR_DJI_Speed_Motor : public IFR_DJI_Motor
{
	public:
		IFR_DJI_Speed_Motor(uint16_t Num, DJI_Motor_Type m_DJI_Motor_type, CanMsgQueue* MsgQueue) : IFR_DJI_Motor(Num, m_DJI_Motor_type, MsgQueue){}
		IFR_DJI_Speed_Motor(float (*Offset_MotoFunc)(float Motor_Angle), uint16_t Num, DJI_Motor_Type m_DJI_Motor_type, CanMsgQueue* MsgQueue): IFR_DJI_Motor(Offset_MotoFunc, Num, m_DJI_Motor_type, MsgQueue){}
		void Motor_Speed_Set(float Speed_Tar);
		IFR_PID Speed_PID;			
	protected:
		float tar_speed;		// 目标速度
};
// 电机角度环类
class IFR_DJI_Pos_Motor: public IFR_DJI_Speed_Motor
{
	public:
		IFR_DJI_Pos_Motor(uint16_t Num, DJI_Motor_Type m_DJI_Motor_type, CanMsgQueue* MsgQueue) : IFR_DJI_Speed_Motor(Num, m_DJI_Motor_type, MsgQueue){}
		IFR_DJI_Pos_Motor(float (*Offset_MotoFunc)(float Motor_Angle), uint16_t Num, DJI_Motor_Type m_DJI_Motor_type, CanMsgQueue* MsgQueue): IFR_DJI_Speed_Motor(Offset_MotoFunc, Num, m_DJI_Motor_type, MsgQueue){}
		void Motor_AbsPos_Set(float AbsPos_Tar);
		void Motor_Pos_Set(float Pos_Tar);
		IFR_PID Pos_PID;
		void motor_set_pos_offest(float pos_offest);
	protected:
		float tar_pos;																		// 目标电机位置
		float last_tar_pos;																// 上一刻的目标位置 
};
// 电机自定义控制值类
class IFR_DJI_Custom_Motor: public IFR_DJI_Pos_Motor
{
	public:
		IFR_DJI_Custom_Motor(uint16_t Num, DJI_Motor_Type m_DJI_Motor_type, CanMsgQueue* MsgQueue) : IFR_DJI_Pos_Motor(Num, m_DJI_Motor_type, MsgQueue){}
		IFR_DJI_Custom_Motor(float (*Offset_MotoFunc)(float Motor_Angle), uint16_t Num, DJI_Motor_Type m_DJI_Motor_type, CanMsgQueue* MsgQueue): IFR_DJI_Pos_Motor(Offset_MotoFunc, Num, m_DJI_Motor_type, MsgQueue){}
		void Motor_Custom_Get(float Custom_Speed, float custom_Angle);
		void Motor_Custom_Angle_Set(float Custom_Tar);
		void Motor_Custom_Speed_Set(float Custom_Speed_Tar);
		float Get_Tar_Custom_Speed(void) { return custom_speed; }
		float Get_Tar_Custom_Pos(void) { return custom_angle; }
		IFR_PID Angle_PID;
	protected:
		float tar_custom;																// 自定义目标值
		float last_tar_custom;													// 上一刻的目标位置 
		float custom_speed;
		float custom_angle;
};

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */
/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */
#endif
