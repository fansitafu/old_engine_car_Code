#ifndef __IFR_RSMOTOR_H_
#define __IFR_RSMOTOR_H_
/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_robstride.h
  * Version			: v2.0
  * Author			: PanJiajun zhaozilin
  * Date				: 2025-10-9
  * Description	:	灵足（RobStride）系列电机控制库（兼容小米电机），支持多型号电机初始化、CAN消息解析、多模式控制，IFR CAN库中的 设备层，负责具体电机控制
	*
  *********************************************************************
  */
#include "ifr_pid.h"
#include "ifr_basic_motor.h"
#include "ifr_can_msg_queue.h"
#ifdef __cplusplus
 extern "C" {
#endif
#ifdef __cplusplus
 }
#endif
typedef enum
{
	CyberGear,
	RobStride_00,
	RobStride_01,
	RobStride_02,
	RobStride_03,
	RobStride_04,
	RobStride_05,
	RobStride_06,
} RS_Motor_type;	// 电机型号结构体
typedef struct
{
	float P_MIN;
	float P_MAX;
	float V_MIN;
	float V_MAX;
	float KP_MIN;
	float KP_MAX;
	float KD_MIN;
	float KD_MAX;
	float T_MIN;
	float T_MAX;
	void set(float P_MIN_, float P_MAX_, float V_MIN_, float V_MAX_, float KP_MIN_, float KP_MAX_, float KD_MIN_, float KD_MAX_, float T_MIN_, float T_MAX_)
	{
		P_MIN = P_MIN_;	P_MAX = P_MAX_; 
		V_MIN = V_MIN_; V_MAX = V_MAX_; 
		KP_MIN = KP_MIN_; KP_MAX = KP_MAX_; 
		KD_MIN = KD_MIN_; KD_MAX = KD_MAX_; 
		T_MIN = T_MIN_; T_MAX = T_MAX_;
	}
} RS_Motor_Prame;	// 电机参数结构体

// 通信协议类型说明
#define Communication_Type_Get_ID 0x00			   			// 获取设备ID (通信类型0)获取设备的ID和64位MCU唯一标识符
#define Communication_Type_MotionControl 0x01	   		// 运控模式电机控制指令 （通信类型1）用来向电机发送控制指令
#define Communication_Type_MotorRequest 0x02	   		// 电机反馈数据 （通信类型2） 用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03		   		// 电机使能运行 （通信类型3）
#define Communication_Type_MotorStop 0x04		   			// 电机停止运行 （通信类型4）
#define Communication_Type_SetPosZero 0x06		   		// 设置电机机械零位（通信类型6）会把当前电机位置设为机械零位（掉电丢失）
#define Communication_Type_Can_ID 0x07			   		  // 设置电机CAN_ID（通信类型7）更改当前电机CAN_ID , 立即生效。
#define Communication_Type_Control_Mode 0x12	   	  // 设置电机模式
#define Communication_Type_GetSingleParameter 0x11  // 单个参数读取（通信类型17）
#define Communication_Type_SetSingleParameter 0x12  // 单个参数写入（通信类型18） （掉电丢失）
#define Communication_Type_ErrorFeedback 0x15	   	  // 故障反馈帧（通信类型21）

// 这是通信17所读取的index参数
const uint16_t Index_List[24] = {0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701E, 0x701F, 0x7020, 0x7021, 0x7022, 0x7024, 0x7025, 0x7026, 0x7028, 0x7029};
// 通讯类型17所读取到的信息
typedef struct
{
	uint8_t run_mode;		// 0运控模式 1位置模式（PP） 2速度模式 3电流模式 5位置模式（CSP）
	float iq_ref;				// 电流模式的iq指令
	float spd_ref;			// 转速模式转速指令
	float limit_torque; // 扭矩限制
	float cur_kp;				// 电流的kp
	float cur_ki;				// 电流的ki
	float curfilt_gain; // 电流滤波系数filt_gain
	float loc_ref;			// 位置模式角度指令
	float limit_spd;		// 位置模式速度限制
	float limit_cur;		// 速度位置模式电流限制
	float mechPos;			// 负载端计圈机械角度
	float iqf;					// iq滤波值
	float mechVel;			// 负载端转速
	float VBUS;					// 母线电压
	float loc_kp;				// 位置的kp
	float spd_kp;				// 速度的kp
	float spd_ki;				// 速度的ki
	float spd_filt_gain;// 速度滤波值
	float acc_rad;			// 速度模式加速度 
  float vel_max;			// 位置模式（PP）速度
	float acc_set;			// 位置模式（PP）加速度
	uint32_t EPScan_time;  // 上报时间设置，1代表10ms，加1递增5ms
	uint32_t canTimeout;   // can超时阀值，20000代表1s
	uint8_t zero_sta;			// 零点标志位，0代表02π,1代表π-π
} Read_RS_Message_From_17;

// 电机回传的报错信息查询：对应Motordata中的error_code
#define error_motoroverheat 0       	// bit0：电机过温故障（默认135度）
#define error_driverchip 		1      		// bit1：驱动芯片故障
#define error_undervoltage  2       	// bit2：欠压故障
#define error_overvoltage   3       	// bit3：过压故障
#define error_encoderuncalibratedt 7  // bit7：编码器未标定
#define error_blockedoroverload 14 		// bit14：堵转或过载故障

// 灵足电机基类
class IFR_RS_Motor: public IFR_Basic_Motor
{
	public:
		IFR_RS_Motor(uint8_t CAN_Id, RS_Motor_type m_RS_Motor_type, CanMsgQueue* MsgQueue);
		IFR_RS_Motor(float (*Offset_MotoFunc)(float Motor_Angle), uint8_t CAN_Id, RS_Motor_type m_RS_Motor_type, CanMsgQueue* MsgQueue);
		void set_torque(float set_torque);
		void set_torque_offset(float set_torque);
		virtual void Enable();
		virtual void Disable(uint8_t clear_error);
		void set_zero_pos();
		void set_can_id(uint8_t set_can_id);
		void get_motor_parameter(uint16_t Index);
		void set_motor_parameter(uint16_t Index, uint32_t Value);
		void get_motor_id_mcu();
		float Get_RSMotor_Torque(void) {return tar_torque;}
		virtual void parse_can_msg(const CanMsg_t msg);
		virtual void generate_can_msg(CanMsg_t& msg);
	protected:
		float tar_torque;																		// 目标扭矩
		CanMsgQueue* msgQueue;															// 消息队列指针
		RS_Motor_Prame RS_Motor_prame;											// 电机最大最小值参数
		Read_RS_Message_From_17 RS_Message_From_17; 				// 用来查电机问询信息(通信类型17)
		uint8_t CAN_MASTER_ID;														  // 主机ID （默认0x1F）
		float pos_offest;																	// 速度前馈系数（由目标电机位置和上一刻的差产生）
		void Motor_Init(uint8_t CAN_Id, RS_Motor_type m_LZ_Motor_type, CanMsgQueue* MsgQueue);	// 电机初始化函数
};
// 电机速度环类
class IFR_RS_Speed_Motor: public IFR_RS_Motor
{
	public:
		IFR_RS_Speed_Motor(uint8_t CAN_Id, RS_Motor_type m_RS_Motor_type, CanMsgQueue* MsgQueue) : IFR_RS_Motor(CAN_Id, m_RS_Motor_type, MsgQueue){}
		IFR_RS_Speed_Motor(float (*Offset_MotoFunc)(float Motor_Angle), uint8_t CAN_Id, RS_Motor_type m_RS_Motor_type, CanMsgQueue* MsgQueue): IFR_RS_Motor(Offset_MotoFunc, CAN_Id, m_RS_Motor_type, MsgQueue){}
		void Motor_Speed_Set(float Speed_Tar);
		IFR_PID Speed_PID;
	protected:
		float tar_speed;																		// 目标速度
};
// 电机角度环类
class IFR_RS_Pos_Motor: public IFR_RS_Speed_Motor
{
	public:
		IFR_RS_Pos_Motor(uint8_t CAN_Id, RS_Motor_type m_RS_Motor_type, CanMsgQueue* MsgQueue): IFR_RS_Speed_Motor(CAN_Id, m_RS_Motor_type, MsgQueue){}
		IFR_RS_Pos_Motor(float (*Offset_MotoFunc)(float Motor_Angle), uint8_t CAN_Id, RS_Motor_type m_RS_Motor_type, CanMsgQueue* MsgQueue): IFR_RS_Speed_Motor(Offset_MotoFunc, CAN_Id, m_RS_Motor_type, MsgQueue){}
		void Motor_AbsPos_Set(float AbsPos_Tar);
		void Motor_Pos_Set(float Pos_Tar);
		IFR_PID Pos_PID;
		void motor_set_pos_offest(float pos_offest);
	protected:
		float tar_pos;																		// 目标电机位置
		float last_tar_pos;																// 上一刻的目标位置 
};
// 电机自定义控制值类
class IFR_RS_Custom_Motor: public IFR_RS_Pos_Motor
{
	public:
		IFR_RS_Custom_Motor(uint8_t CAN_Id, RS_Motor_type m_RS_Motor_type, CanMsgQueue* MsgQueue): IFR_RS_Pos_Motor(CAN_Id, m_RS_Motor_type, MsgQueue){}
		IFR_RS_Custom_Motor(float (*Offset_MotoFunc)(float Motor_Angle), uint8_t CAN_Id, RS_Motor_type m_RS_Motor_type, CanMsgQueue* MsgQueue): IFR_RS_Pos_Motor(Offset_MotoFunc, CAN_Id, m_RS_Motor_type, MsgQueue){}
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
