#ifndef IFR_BASIC_MOTOR_H
#define IFR_BASIC_MOTOR_H
/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_basic_motor.h
  * Version			: v1.0
  * Author			: PanJiajun 
  * Date				: 2025-10-10
  * Description	:	IFR CAN库中的 设备抽象层，负责统一电机设备接口定义
	*	
  *********************************************************************
  */
#include "main.h"
#include "string.h"
#include "ifr_pid.h"
#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif
// CAN帧类型枚举
typedef enum 
{
	CAN_STD = 0,  // 标准帧（11位ID）
	CAN_EXT   		// 扩展帧（29位ID）
} CAN_identifier_type_e;

// 统一CAN消息格式（适配标准帧/扩展帧）
typedef struct 
{
	uint32_t id;                        			// CAN ID
	uint8_t data[8];                    			// 数据（最长8字节）
	CAN_identifier_type_e can_identifier_type;  // CAN ID类型（标准/扩展）
} CanMsg_t;

// 电机状态枚举
typedef enum Motor_State
{
	MOTOR_DISABLE = 0, // 未使能
	MOTOR_ENABLE,  // 使能
} Motor_State_e;

// 电机报错信息
typedef enum 
{
	MOTOR_NO_ERROR = 0, 	// 无错误
	MOTOR_PRAME_ERROR,		// 函数传入的参数错误
	MOTOR_MSGQUEUE_ERROR,	// 队列为空或者满
} ErrorCode_t;

// 电机状态数据结构（所有电机共有的反馈信息）
typedef struct MotorData
{
	float Angle;       			// 角度（单位：rad）
	float Speed;       			// 速度（单位：rad/s）
	float Electric;     		// 电流（单位：A）
	float Torque;				// 扭矩（单位：Nm）
	float Temperature; 			// 温度（单位：℃）
	float abs_angle;   			// 多圈计数角度（单位：rad）
	float last_angle;  			// 上一刻角度值（单位：rad）
	uint16_t error_code;		// 电机报错信息
	Motor_State motor_state;	// 电机状态（是否使能）
	uint8_t Updata;				// 判断电机是否接到消息的标志位
	uint32_t last_update_time;	// 上次接到消息的时间戳（单位：ms）
} MotorData_t;
// 抽象基类：所有电机必须继承并实现这些接口
class IFR_Basic_Motor 
{
	public:
    // 纯虚析构函数必须声明为=0且提供实现
    virtual ~IFR_Basic_Motor() = 0;
    // 生成CAN控制消息（将目标转为CAN帧，供入队）
    virtual void generate_can_msg(CanMsg_t& msg) = 0;
    // 解析CAN反馈消息（根据收到的CAN帧更新电机状态）
    virtual void parse_can_msg(const CanMsg_t msg) = 0;
	// 使能电机
	virtual void Enable() = 0;
	// 失能电机
	virtual void Disable(uint8_t clear_error) = 0;
	// 返回电机回传值
	MotorData_t get_motordata() {return Motordata;}
	// 设定输出值按倍数降低
	void Set_Motor_decrease(float set_decrease){ this->set_decrease = set_decrease;} // 注意！若有前馈函数，这个参数也会把前馈同比例降低
	// 注册前馈补偿函数
	void Register_Motor_Offset_MotoFunc(float (*Offset_MotoFunc)(float Motor_Angle)) {Motor_Offset_MotoFunc = Offset_MotoFunc;}	
	// 设定电机减速比
	void set_motor_reduction_ratio(float set_reduction_ratio) { this->set_reduction_ratio = set_reduction_ratio; }
	// 返回电机CANID
	uint16_t get_motor_id() { return motor_id; }
	// 返回电机是否接到消息的标志位
	uint8_t get_updata() { return Motordata.Updata; }	
	protected:
		uint16_t motor_id;				// 电机ID号
		MotorData_t Motordata;     		// 电机数据
		ErrorCode_t Errorcode;		 	// 此库内置的报错系统
		float set_decrease;			 	// 电机电流按比例降低的值
		float set_reduction_ratio;// 电机减速箱减速比
		float (*Motor_Offset_MotoFunc)(float Motor_Angle);  // 前馈补偿函数
};
// 纯虚析构函数必须提供实现（否则链接错误）
inline IFR_Basic_Motor::~IFR_Basic_Motor() {}
#endif // IFR_BASIC_MOTOR_H
