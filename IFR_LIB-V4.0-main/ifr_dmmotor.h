#ifndef __IFR_DMMOTOR_H_
#define __IFR_DMMOTOR_H_
/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_dmmotor.h
  * Version			: v2.0
  * Author			: Albert PanJiajun 
  * Date				: 2025-10-10
  * Description	:	DM系列电机控制库，包含电机初始化、CAN消息解析、扭矩/速度/位置控制等核心功能，IFR CAN库中的 设备层，负责具体电机控制
	*
  *********************************************************************
  */
#include "main.h"
#include "ifr_pid.h"
#include "ifr_basic_motor.h"
#include "ifr_can_msg_queue.h"
/*******************************/
#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif

#define MasterCanID 0x00	// 主机ID
typedef enum
{
	DMState_Disable = 0x00,
	DMState_Enable = 0x01,
	DMError_overV = 0x08,
	DMError_qianya = 0x09,
	DMError_overI = 0x0A,
	DMError_MosHot = 0x0B,
	DMError_MotorHot = 0x0C,
	DMError_loss	= 0x0D,
	DMError_overF = 0x0E,
} ErrCodeTypedef;
typedef enum
{
	DM_Enable = 0xFC,		//使能(上电后800ms)
	DM_Disable = 0xFD,	//失能
	DM_SaveZero = 0xFE,	//保存零点
	DM_ClearError = 0xFB,
} DM_Command_t;
typedef struct
{
	float P_MIN;	// 角度最小值
	float P_MAX;	// 角度最大值
	float V_MIN;	// 速度最小值
	float V_MAX;	// 速度最大值
	float KP_MIN;	// KP最小值
	float KP_MAX;	// KP最大值
	float KD_MIN;	// KD最小值
	float KD_MAX;	// KD最大值
	float T_MIN;	// 扭矩最小值
	float T_MAX;	// 扭矩最大值
	void set(float P_MIN_, float P_MAX_, float V_MIN_, float V_MAX_, float KP_MIN_, float KP_MAX_, float KD_MIN_, float KD_MAX_, float T_MIN_, float T_MAX_)
	{
		P_MIN = P_MIN_;	P_MAX = P_MAX_; 
		V_MIN = V_MIN_; V_MAX = V_MAX_; 
		KP_MIN = KP_MIN_; KP_MAX = KP_MAX_; 
		KD_MIN = KD_MIN_; KD_MAX = KD_MAX_; 
		T_MIN = T_MIN_; T_MAX = T_MAX_;
	}
} DM_Motor_Prame;	// 电机参数

typedef enum
{
	DM_4310 = 0,
	DM_3507,
	DM_8009,
} DM_Motor_Type;	// 达妙电机型号
class IFR_DM_Motor : public IFR_Basic_Motor
{
	public:
		inline float uint_to_float(int x_int, float x_max, int bits);
		inline int float_to_uint(float x, float x_max, int bits);
		inline float uint_to_float(int x_int, float x_min, float x_max, int bits);
		inline int float_to_uint(float x, float x_min, float x_max, int bits);
		IFR_DM_Motor(uint8_t Motor_id, DM_Motor_Type DM_Motor_type, CanMsgQueue* MsgQueue);
		IFR_DM_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t Motor_id, DM_Motor_Type DM_Motor_type, CanMsgQueue* MsgQueue);
		IFR_DM_Motor(uint8_t Motor_id, DM_Motor_Type DM_Motor_type, float T_Max, CanMsgQueue* MsgQueue);
		IFR_DM_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t Motor_id, DM_Motor_Type DM_Motor_type, float T_Max, CanMsgQueue* MsgQueue);
		void set_torque(float set_torque);
		void set_torque_offset(float _set_torque);
		float Get_DMMotor_Torque(void) {return tar_torque;}
		void DM_Motor_Command(DM_Command_t Command);
		virtual void generate_can_msg(CanMsg_t& msg); 	 // 生成CAN控制消息（将目标转为CAN帧，供入队）
		virtual void parse_can_msg(const CanMsg_t msg); // 解析CAN反馈消息（根据收到的CAN帧更新电机状态）
		virtual void Enable();
		virtual void Disable(uint8_t clear_error);
		void (*DM_Error_Callback)(void);
	protected:
		CanMsgQueue* msgQueue;		// 消息队列指针
		DM_Motor_Prame DM_motor_prame;	// 电机数据最大最小值
		float _t_max;						// 最大扭矩	
		float tar_torque;					// 目标扭矩
		ErrCodeTypedef DM_State;	// 电机状态
		uint8_t T_MOS;				// 表示驱动上MOS 的平均温度，单位℃
		uint8_t T_Rotor;			// 表示电机内部线圈的平均温度，单位℃
		float pos_offest;			// 速度前馈系数
		void motor_init(uint8_t Motor_id, CanMsgQueue* MsgQueue, DM_Motor_Type DM_Motor_type);
};
// 电机速度环类
class IFR_DM_Speed_Motor : public IFR_DM_Motor
{
	public:
		IFR_DM_Speed_Motor(uint8_t Motor_id, DM_Motor_Type DM_Motor_type, CanMsgQueue* MsgQueue) : IFR_DM_Motor(Motor_id, DM_Motor_type, MsgQueue){}
		IFR_DM_Speed_Motor(float (*Offset_MotoFunc)(float Motor_Angle), uint8_t Motor_id, DM_Motor_Type DM_Motor_type, CanMsgQueue* MsgQueue): IFR_DM_Motor(Offset_MotoFunc, Motor_id, DM_Motor_type, MsgQueue){}
		IFR_DM_Speed_Motor(uint8_t Motor_id, DM_Motor_Type DM_Motor_type, float T_Max, CanMsgQueue* MsgQueue) : IFR_DM_Motor(Motor_id, DM_Motor_type, T_Max, MsgQueue){}
		IFR_DM_Speed_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t Motor_id, DM_Motor_Type DM_Motor_type, float T_Max, CanMsgQueue* MsgQueue): IFR_DM_Motor(Offset_MotoFunc, Motor_id, DM_Motor_type, T_Max, MsgQueue){}
		void Motor_Speed_Set(float Speed_Tar);
		IFR_PID Speed_PID;			
	protected:
		float tar_speed;		// 目标速度
};
// 电机角度环类
class IFR_DM_Pos_Motor: public IFR_DM_Speed_Motor
{
	public:
		IFR_DM_Pos_Motor(uint8_t Motor_id, DM_Motor_Type DM_Motor_type, CanMsgQueue* MsgQueue) : IFR_DM_Speed_Motor(Motor_id, DM_Motor_type, MsgQueue){}
		IFR_DM_Pos_Motor(float (*Offset_MotoFunc)(float Motor_Angle), uint8_t Motor_id, DM_Motor_Type DM_Motor_type, CanMsgQueue* MsgQueue): IFR_DM_Speed_Motor(Offset_MotoFunc, Motor_id, DM_Motor_type, MsgQueue){}
		IFR_DM_Pos_Motor(uint8_t Motor_id, DM_Motor_Type DM_Motor_type, float T_Max, CanMsgQueue* MsgQueue) : IFR_DM_Speed_Motor(Motor_id, DM_Motor_type, T_Max, MsgQueue){}
		IFR_DM_Pos_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t Motor_id, DM_Motor_Type DM_Motor_type, float T_Max, CanMsgQueue* MsgQueue): IFR_DM_Speed_Motor(Offset_MotoFunc, Motor_id, DM_Motor_type, T_Max, MsgQueue){}
		void Motor_AbsPos_Set(float AbsPos_Tar);
		void Motor_Pos_Set(float Pos_Tar);
		void motor_set_pos_offest(float pos_offest);
		IFR_PID Pos_PID;
	protected:
		float tar_pos;																		// 目标电机位置
		float last_tar_pos;																// 上一刻的目标位置 
};
// 电机自定义控制值类
class IFR_DM_Custom_Motor: public IFR_DM_Pos_Motor
{
	public:
		IFR_DM_Custom_Motor(uint8_t Motor_id, DM_Motor_Type DM_Motor_type, CanMsgQueue* MsgQueue) : IFR_DM_Pos_Motor(Motor_id, DM_Motor_type, MsgQueue){}
		IFR_DM_Custom_Motor(float (*Offset_MotoFunc)(float Motor_Angle), uint8_t Motor_id, DM_Motor_Type DM_Motor_type, CanMsgQueue* MsgQueue): IFR_DM_Pos_Motor(Offset_MotoFunc, Motor_id, DM_Motor_type, MsgQueue){}
		IFR_DM_Custom_Motor(uint8_t Motor_id, DM_Motor_Type DM_Motor_type, float T_Max, CanMsgQueue* MsgQueue) : IFR_DM_Pos_Motor(Motor_id, DM_Motor_type, T_Max, MsgQueue){}
		IFR_DM_Custom_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t Motor_id, DM_Motor_Type DM_Motor_type, float T_Max, CanMsgQueue* MsgQueue): IFR_DM_Pos_Motor(Offset_MotoFunc, Motor_id, DM_Motor_type, T_Max, MsgQueue){}
		void Motor_Custom_Get(float Custom_Speed, float custom_Angle);
		void Motor_Custom_Angle_Set(float Custom_Tar);
		void Motor_Custom_Speed_Set(float Custom_Speed_Tar);
		float Get_Tar_Custom_Speed(void) { return custom_speed; }
		float Get_Tar_Custom_Pos(void) { return custom_angle; }
		IFR_PID Angle_PID;
	protected:
		float tar_custom;																		// 自定义目标值
		float last_tar_custom;															// 上一刻的目标位置 
		float custom_speed;
		float custom_angle;
};
#endif
