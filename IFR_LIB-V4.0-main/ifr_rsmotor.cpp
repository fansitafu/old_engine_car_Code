/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_robstride.cpp
  * Version			: v2.0
  * Author			: PanJiajun zhaozilin
  * Date			: 2025-10-9
  * Description	:	灵足（RobStride）系列电机控制库（兼容小米电机），支持多型号电机初始化、CAN消息解析、多模式控制，IFR CAN库中的 设备层，负责具体电机控制
	*
  *********************************************************************
  */
/* Includes ---------------------------------------------------------*/
#include "ifr_rsmotor.h"
static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits);
static int float_to_uint(float x, float x_min, float x_max, int bits);
static float Byte_to_float(const uint8_t *bytedata);
/*******************************************************************************
* @功能     	: 灵足电机类无前馈函数的构造函数，调用初始化函数完成电机ID、类型、消息队列配置
* @参数1        : CAN_Id：电机CAN通信ID（用于寻址）
* @参数2        : m_LZ_Motor_type：灵足电机型号（RobStride_00~06，对应不同参数范围）
* @参数3        : MsgQueue：CAN消息队列指针（存储待发送的控制消息）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
IFR_RS_Motor::IFR_RS_Motor(uint8_t CAN_Id, RS_Motor_type m_LZ_Motor_type, CanMsgQueue* MsgQueue)
{
	Motor_Init(CAN_Id, m_LZ_Motor_type, MsgQueue);
}
/*******************************************************************************
* @功能     	: 灵足电机类带前馈函数的构造函数，初始化电机并绑定扭矩前馈补偿函数
* @参数1        : Offset_MotoFunc：扭矩前馈补偿函数指针（输入电机当前角度，输出补偿扭矩）
* @参数2        : CAN_Id：电机CAN通信ID（用于寻址）
* @参数3        : m_LZ_Motor_type：灵足电机型号（RobStride_00~06，对应不同参数范围）
* @参数4        : MsgQueue：CAN消息队列指针（存储待发送的控制消息）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
IFR_RS_Motor::IFR_RS_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id, RS_Motor_type m_LZ_Motor_type, CanMsgQueue* MsgQueue)
{
	Motor_Init(CAN_Id, m_LZ_Motor_type, MsgQueue);
	Register_Motor_Offset_MotoFunc(Offset_MotoFunc);
}
/*******************************************************************************
* @功能     	: 解析灵足电机反馈的扩展CAN消息，提取不同反馈类型的电机状态（角度、速度、扭矩、错误码等）
* @参数1        : msg：CAN消息结构体（包含扩展ID、数据内容等）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Motor::parse_can_msg(const CanMsg_t msg)
{
	// 筛选条件：扩展CAN帧、ID中“电机ID字段”与当前电机CAN_ID匹配
	if ((uint8_t)((msg.id & 0xFF00) >> 8) == get_motor_id() && msg.can_identifier_type == CAN_EXT)
	{
		Motordata.last_update_time = HAL_GetTick(); // 更新上次接到消息的时间戳
		// 提取反馈类型（扩展ID的24~29位，共6位）
		uint16_t feedback_type = uint32_t((msg.id & 0x3F000000) >> 24);
		Motordata.Updata = 1; // 判断电机是否断连标志位 置1
		// 反馈类型0x02/0x12：基础状态（角度、速度、扭矩、温度、使能状态）
		if (feedback_type  == 0x02 || feedback_type == 0x12) 
		{
			// 原始整数转浮点数（按灵足电机协议，16位数据对应指定范围）
			Motordata.Angle = uint16_to_float(uint16_t(msg.data[0] << 8 | msg.data[1]), RS_Motor_prame.P_MIN, RS_Motor_prame.P_MAX, 16); // 数据0+1：角度
			Motordata.Speed = uint16_to_float(uint16_t(msg.data[2] << 8 | msg.data[3]), RS_Motor_prame.V_MIN, RS_Motor_prame.V_MAX, 16); // 数据2+3：速度
			Motordata.Torque = uint16_to_float(uint16_t(msg.data[4] << 8 | msg.data[5]), RS_Motor_prame.T_MIN, RS_Motor_prame.T_MAX, 16); // 数据4+5：扭矩
			Motordata.Temperature = float(msg.data[6] << 8 | msg.data[7]) / 10.0f; // 数据6+7：温度（除以10得实际温度，单位℃）
			// 提取错误码（扩展ID的16~21位，共6位）
			Motordata.error_code = uint32_t((msg.id & 0x3F0000) >> 16);
			// 提取使能状态（扩展ID的22~23位，为2时表示使能）
			if (uint8_t((msg.id & 0xC00000) >> 22) == 2)
				Motordata.motor_state = MOTOR_ENABLE;
			else 
				Motordata.motor_state = MOTOR_DISABLE;
			if (Motordata.last_angle == 2 * IFR_PI)
				Motordata.last_angle = Motordata.Angle;
			float Error = Motordata.Angle - Motordata.last_angle; // 当前与上一帧角度差
			// 速度大于阈值（过滤噪声）时，计算多圈绝对角度（处理角度溢出）
			if (IFR_ABS(Error) > 0.01f)
			{
				Motordata.abs_angle += Error / this->set_reduction_ratio; // 累加得到绝对角度
				// 处理溢出：差值小于-π（多转一圈）加2π，大于π减2π
				if (Error < IFR_PI) 
					Motordata.abs_angle += IFR_PI * 2.0f / this->set_reduction_ratio;
				else if (Error > IFR_PI)  
					Motordata.abs_angle -= IFR_PI * 2.0f / this->set_reduction_ratio;
			}
			Motordata.last_angle = Motordata.Angle; // 保存当前角度为下一帧的“上一角度”
		}
		// 反馈类型0x15：错误码反馈（数据前三个字节为错误码）
		else if (feedback_type == 0x15)	
		{
			Motordata.error_code = uint32_t(msg.data[2] << 16 | msg.data[1] << 8 | msg.data[0]);
		}
		// 反馈类型0x11：参数信息反馈（根据参数索引解析对应参数值）
		else if (feedback_type == 0x11) 
		{
			uint16_t current_index = (msg.data[1] << 8) | msg.data[0]; // 提取当前参数索引（数据1+0）
			// 遍历参数索引列表，匹配后解析对应参数
			for (int i = 0; i <= 23; i++)
			{
				if (current_index == Index_List[i])
				{
					switch (i)
					{
						case 0: RS_Message_From_17.run_mode = uint8_t(msg.data[4]); break;        // 运行模式
						case 1: RS_Message_From_17.iq_ref = Byte_to_float(msg.data); break;       // 参考电流IQ
						case 2: RS_Message_From_17.spd_ref = Byte_to_float(msg.data); break;      // 参考速度
						case 3: RS_Message_From_17.limit_torque = Byte_to_float(msg.data); break; // 扭矩限制
						case 4: RS_Message_From_17.cur_kp = Byte_to_float(msg.data); break;       // 电流环KP
						case 5: RS_Message_From_17.cur_ki = Byte_to_float(msg.data); break;       // 电流环KI
						case 6: RS_Message_From_17.curfilt_gain = Byte_to_float(msg.data); break;  // 电流滤波增益
						case 7: RS_Message_From_17.loc_ref = Byte_to_float(msg.data); break;       // 参考位置
						case 8: RS_Message_From_17.limit_spd = Byte_to_float(msg.data); break;     // 速度限制
						case 9: RS_Message_From_17.limit_cur = Byte_to_float(msg.data); break;     // 电流限制
						case 10: RS_Message_From_17.mechPos = Byte_to_float(msg.data); break;      // 机械位置
						case 11: RS_Message_From_17.iqf = Byte_to_float(msg.data); break;          // 滤波后电流IQ
						case 12: RS_Message_From_17.mechVel = Byte_to_float(msg.data); break;      // 机械速度
						case 13: RS_Message_From_17.VBUS = Byte_to_float(msg.data); break;         // 母线电压
						case 14: RS_Message_From_17.loc_kp = Byte_to_float(msg.data); break;       // 位置环KP
						case 15: RS_Message_From_17.spd_kp = Byte_to_float(msg.data); break;       // 速度环KP
						case 16: RS_Message_From_17.spd_ki = Byte_to_float(msg.data); break;       // 速度环KI
						case 17: RS_Message_From_17.spd_filt_gain = Byte_to_float(msg.data); break;// 速度滤波增益
						case 18: RS_Message_From_17.acc_rad = Byte_to_float(msg.data); break;      // 角加速度
						case 19: RS_Message_From_17.vel_max = Byte_to_float(msg.data); break;      // 最大速度
						case 20: RS_Message_From_17.acc_set = Byte_to_float(msg.data); break;      // 设置加速度
						case 21: RS_Message_From_17.EPScan_time = uint16_t(msg.data[4] << 8 | msg.data[5]); break; // EPS扫描时间
						case 22: RS_Message_From_17.canTimeout = uint32_t(msg.data[4] << 24 | msg.data[5] << 16 | msg.data[6] << 8 | msg.data[7]); break; // CAN超时时间
						case 23: RS_Message_From_17.zero_sta = uint8_t(msg.data[4]); break;        // 零点状态
					}
				break; // 匹配到索引后退出循环，避免重复解析
				}	
			}
		}
	}
}
/*******************************************************************************
* @功能     	: 灵足电机初始化函数，配置电机CAN ID、消息队列，并根据电机型号设置参数范围
* @参数1        : CAN_Id：电机CAN通信ID（用于寻址）
* @参数2        : m_LZ_Motor_type：灵足电机型号（RobStride_00~06）
* @参数3        : MsgQueue：CAN消息队列指针（存储待发送的控制消息）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Motor::Motor_Init(uint8_t CAN_Id, RS_Motor_type m_LZ_Motor_type, CanMsgQueue* MsgQueue)
{
	this->motor_id = CAN_Id;         // 保存电机CAN ID
	this->CAN_MASTER_ID = 0x1F;      // 设置CAN主ID（灵足电机协议固定值）
	Motordata.last_angle = 2 * IFR_PI;
	this->set_decrease = 1;
	this->Errorcode = MOTOR_NO_ERROR;
	this->pos_offest = 0;
	this->set_reduction_ratio = 1.0f;	// 默认无需处理减速比
	Motordata.last_update_time = 0; // 初始化上次接到消息的时间戳
	// 检查消息队列有效性，无效则标记参数错误
	if (MsgQueue == NULL) 
	{
		Errorcode = MOTOR_PRAME_ERROR;
		return;
   }
	this->msgQueue = MsgQueue; // 绑定CAN消息队列
	// 根据电机型号设置参数范围（P：角度；V：速度；KP/KD：电机内置PID参数；T：扭矩）
	switch (m_LZ_Motor_type)
	{ 
		case CyberGear:
			// 参数顺序：P_MIN, P_MAX, V_MIN, V_MAX, KP_MIN, KP_MAX, KD_MIN, KD_MAX, T_MIN, T_MAX
			RS_Motor_prame.set(-12.5f, 12.5f, -30.0f, 30.0f, 0.0f, 500.0f, 0.0f, 5.0f, -12.0f, 12.0f);
			break;
		case RobStride_00:
			RS_Motor_prame.set(-12.57f, 12.57f, -33.0f, 33.0f, 0.0f, 500.0f, 0.0f, 5.0f, -14.0f, 14.0f);
			break;
		case RobStride_01:
			RS_Motor_prame.set(-12.57f, 12.57f, -44.0f, 44.0f, 0.0f, 500.0f, 0.0f, 5.0f, -17.0f, 17.0f);
			break;
		case RobStride_02:
			RS_Motor_prame.set(-12.57f, 12.57f, -44.0f, 44.0f, 0.0f, 500.0f, 0.0f, 5.0f, -17.0f, 17.0f);
			break;
		case RobStride_03:
			RS_Motor_prame.set(-12.57f, 12.57f, -20.0f, 20.0f, 0.0f, 500.0f, 0.0f, 100.0f, -60.0f, 60.0f);
			break;
		case RobStride_04:
			RS_Motor_prame.set(-12.57f, 12.57f, -15.0f, 15.0f, 0.0f, 5000.0f, 0.0f, 100.0f, -120.0f, 120.0f);
			break;
		case RobStride_05:
			RS_Motor_prame.set(-12.57f, 12.57f, -50.0f, 50.0f, 0.0f, 500.0f, 0.0f, 5.0f, -5.5f, 5.5f);
			break;
		case RobStride_06:
			RS_Motor_prame.set(-12.57f, 12.57f, -50.0f, 50.0f, 0.0f, 5000.0f, 0.0f, 100.0f, -36.0f, 36.0f);
			break;
		default:
			Errorcode = MOTOR_PRAME_ERROR; // 未知电机型号，标记参数错误
			break;
	}
}
/*******************************************************************************
* @功能     	: 灵足电机使能函数，生成使能指令的CAN消息并加入发送队列
* @参数        	: 无
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Motor::Enable()
{
	CanMsg_t msg;
	// 构造扩展CAN ID：反馈类型（使能）+ 主ID + 电机ID
	msg.id =  Communication_Type_MotorEnable << 24 | CAN_MASTER_ID << 8 | motor_id;
	memset(msg.data, 0, 8); 
	generate_can_msg(msg); 
}
/*******************************************************************************
* @功能     	: 灵足电机失能函数，生成失能指令的CAN消息，支持错误清零
* @参数1        : clear_error：错误清零标志（1：失能同时清零错误；0：仅失能）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Motor::Disable(uint8_t clear_error)
{
	CanMsg_t msg;
	// 构造扩展CAN ID：反馈类型（失能）+ 主ID + 电机ID
	msg.id =  Communication_Type_MotorStop << 24 | CAN_MASTER_ID << 8 | motor_id;
	memset(msg.data, 0, 8); 
	msg.data[0] = clear_error; // 数据0存储错误清零标志
	generate_can_msg(msg); 
}
/*******************************************************************************
* @功能     	: 设置灵足电机的CAN ID，需先失能电机再发送配置指令
* @参数1        : set_can_id：电机新的CAN ID（用于后续通信寻址）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Motor::set_can_id(uint8_t set_can_id)
{
	Disable(1); // 先失能电机并清零错误
	CanMsg_t msg;
	// 构造扩展CAN ID：反馈类型（设置CAN ID）+ 新CAN ID + 主ID + 旧CAN ID
	msg.id =  Communication_Type_Can_ID << 24 | set_can_id << 16 | CAN_MASTER_ID << 8 | motor_id;
	memset(msg.data, 0, 8);
	generate_can_msg(msg); 
}
/*******************************************************************************
* @功能     	: 设置灵足电机的机械零点，需先失能再使能以确保零点生效
* @参数        	: 无
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Motor::set_zero_pos()
{
	Disable(1); // 先失能电机并清零错误
	CanMsg_t msg;
	// 构造扩展CAN ID：反馈类型（设置零点）+ 主ID + 电机ID
	msg.id =  Communication_Type_SetPosZero << 24 | CAN_MASTER_ID << 8 | motor_id;
	memset(msg.data, 0, 8); 
	msg.data[0] = 1;
	generate_can_msg(msg); 
	Enable(); // 重新使能电机，使零点设置生效
}
/*******************************************************************************
* @功能     	: 设置灵足电机扭矩输出，自动使能和消除错误
* @参数1        : set_torque：电机目标扭矩值（单位：N·m）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Motor::set_torque(float set_torque)
{
	this->tar_torque = set_torque;
	this->tar_torque *= this->set_decrease;
	this->tar_torque = IFR_CLAMP(this->tar_torque, RS_Motor_prame.T_MIN, RS_Motor_prame.T_MAX);	// 将扭矩值限制在最大扭矩值内，防止计算出负数
	CanMsg_t msg;
	// 扩展CAN ID：反馈类型（运动控制）+ 扭矩整数 + 电机ID
	msg.id =  Communication_Type_MotionControl << 24 | float_to_uint(this->tar_torque, RS_Motor_prame.T_MIN, RS_Motor_prame.T_MAX, 16) << 8 | motor_id;
	memset(msg.data, 0, 8); 
	generate_can_msg(msg); 
}
/*******************************************************************************
* @功能     	: 设置灵足电机扭矩输出，自动处理使能状态和错误，支持前馈补偿
* @参数1        : set_torque：电机目标扭矩值（单位：N·m）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Motor::set_torque_offset(float _set_torque)
{
	this->tar_torque = _set_torque;
// 若前馈补偿函数有效，叠加角度对应的补偿扭矩
	if (NULL != this->Motor_Offset_MotoFunc)
		this->tar_torque += (*this->Motor_Offset_MotoFunc)(this->Motordata.Angle);
	set_torque(this->tar_torque); 
}
/*******************************************************************************
* @功能     	: 读取灵足电机的指定参数，生成参数读取指令的CAN消息
* @参数1        : Index：参数索引（对应电机内部参数，如KP、KI等，见Index_List定义）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Motor::get_motor_parameter(uint16_t Index)
{
	CanMsg_t msg;
	// 扩展CAN ID：反馈类型（读取参数）+ 主ID + 电机ID
	msg.id =  Communication_Type_GetSingleParameter << 24 | CAN_MASTER_ID << 8 | motor_id;
	memset(msg.data, 0, 8); 
	msg.data[0] = Index;       // 数据0：参数索引低8位
	msg.data[1] = Index << 8;  // 数据1：参数索引高8位
	generate_can_msg(msg); 
}
/*******************************************************************************
* @功能     	: 写入灵足电机的指定参数，支持不同类型参数（整数/浮点数）
* @参数1        : Index：参数索引（对应电机内部参数，如KP、KI等）
* @参数2        : Value：参数值（整数或浮点数的二进制表示）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Motor::set_motor_parameter(uint16_t Index, uint32_t Value)
{
	CanMsg_t msg;
	// 扩展CAN ID：反馈类型（写入参数）+ 主ID + 电机ID
	msg.id =  Communication_Type_SetSingleParameter << 24 | CAN_MASTER_ID << 8 | motor_id;
	memset(msg.data, 0, 8);
	memcpy(&msg.data[0], &Index, 2); // 数据0+1：参数索引（低8位+高8位）
	if (Index == 0x7005)
		msg.data[4] = Value & 0xff; // 数据4：整数参数低8位
	else
	{
		float fValue = *(float*)&Value; // 将uint32_t转为float
		// 数据4~7：浮点数的4个字节（小端存储）
		msg.data[4] = ((uint8_t*)&fValue)[0];
		msg.data[5] = ((uint8_t*)&fValue)[1];
		msg.data[6] = ((uint8_t*)&fValue)[2];
		msg.data[7] = ((uint8_t*)&fValue)[3];
	}
	generate_can_msg(msg); // 加入CAN发送队列
}
/*******************************************************************************
* @功能     	: 获取灵足电机的设备ID和MCU信息，生成对应的CAN读取指令
* @参数         : 无
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Motor::get_motor_id_mcu()
{
	CanMsg_t msg;
	// 扩展CAN ID：反馈类型（获取ID/MCU）+ 主ID + 电机ID
	msg.id =  Communication_Type_Get_ID << 24 | CAN_MASTER_ID << 8 | motor_id;
	memset(msg.data, 0, 8);
	generate_can_msg(msg);
}
/*******************************************************************************
* @功能     	: 将灵足电机的CAN控制消息加入发送队列，若队列无效或满则标记错误
* @参数1        : msg：待发送的CAN消息结构体（已填充扩展ID、数据等）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Motor::generate_can_msg(CanMsg_t& msg)
{
	msg.can_identifier_type = CAN_EXT; // 标记为扩展CAN帧
	// 队列有效性和空闲状态检查
	if (msgQueue != NULL)
	{
		msgQueue->enqueue(msg);
		Motordata.Updata = 0;
	}
	if (msgQueue == NULL || msgQueue->is_full())
		Errorcode = MOTOR_MSGQUEUE_ERROR; // 队列错误：无效或满
}
/*******************************************************************************
* @功能     	: 灵足速度环电机类的速度设置函数，通过速度PID计算扭矩实现速度控制
* @参数1        : Speed_Tar：电机目标速度（单位：rad/s）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Speed_Motor::Motor_Speed_Set(float Speed_Tar)
{
	tar_speed = Speed_Tar; 
	// 速度PID计算：输入目标速度与当前速度偏差，输出目标扭矩
	tar_torque = Speed_PID.Positional_PID(tar_speed, Motordata.Speed);
	set_torque_offset(tar_torque);
}
/*******************************************************************************
* @功能     	: 灵足位置环电机类的单圈位置设置函数，采用位置-PID+速度-PID双环控制
* @参数1        : Pos_Tar：电机单圈目标位置（单位：rad）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Pos_Motor::Motor_Pos_Set(float Pos_Tar)
{
	tar_pos = Pos_Tar; 
	// 位置PID：目标位置与当前单圈位置偏差→输出目标速度
	tar_speed = Pos_PID.Positional_PID(tar_pos, Motordata.Angle);
	// 速度前馈：直接通过目标位置差计算出期望速度前馈
	tar_speed += pos_offest * (tar_pos - last_tar_pos);			
	last_tar_pos = tar_pos;
	// 速度PID：目标速度与当前速度偏差→输出目标扭矩
	tar_torque = Speed_PID.Positional_PID(tar_speed, Motordata.Speed);
	set_torque_offset(tar_torque);
}
/*******************************************************************************
* @功能     	: 灵足位置环电机类的多圈绝对位置设置函数，支持连续多圈控制
* @参数1        : AbsPos_Tar：电机多圈绝对目标位置（单位：rad）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Pos_Motor::Motor_AbsPos_Set(float AbsPos_Tar)
{
	tar_pos = AbsPos_Tar; 
	// 位置PID：绝对目标位置与当前绝对位置偏差→输出目标速度
	tar_speed = Pos_PID.Positional_PID(tar_pos, Motordata.abs_angle);
	// 速度前馈：直接通过目标位置差计算出期望速度前馈
	tar_speed += pos_offest * (tar_pos - last_tar_pos);			
	last_tar_pos = tar_pos;
	// 速度PID：目标速度与当前速度偏差→输出目标扭矩
	tar_torque = Speed_PID.Positional_PID(tar_speed, Motordata.Speed);
	set_torque_offset(tar_torque); 
}
/*******************************************************************************
* @功能     	: 灵足位置环电机类的速度前馈系数设置函数，用于提高位置环响应速度
* @参数1        : pos_offest：电机速度前馈系数
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Pos_Motor::motor_set_pos_offest(float pos_offest)
{
	this->pos_offest = pos_offest;
}
/*******************************************************************************
* @功能     	: 灵足自定义控制电机类的外部数据接收函数，用外部数据替代电机原生反馈
* @参数1        : Custom_Speed：外部输入的自定义速度（替代Motordata.Speed）
* @参数2        : Custom_Angle：外部输入的自定义角度（替代Motordata.Angle）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Custom_Motor::Motor_Custom_Get(float Custom_Speed, float Custom_Angle)
{
	this->custom_speed = Custom_Speed; // 保存自定义速度
	this->custom_angle = Custom_Angle; // 保存自定义角度
}
/*******************************************************************************
* @功能     	: 灵足自定义控制电机类的位置设置函数，基于外部数据实现位置闭环
* @参数1        : Custom_Tar：自定义位置目标值（与Custom_Angle单位一致）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Custom_Motor::Motor_Custom_Angle_Set(float Custom_Tar)
{
	tar_custom = Custom_Tar; 
	// 角度PID：自定义目标位置与当前自定义角度偏差→输出目标速度
	tar_speed = Angle_PID.Positional_PID(tar_custom, this->custom_angle);
	// 速度前馈：直接通过目标位置差计算出期望速度前馈
	tar_speed += pos_offest * (tar_custom - last_tar_custom);			
	last_tar_custom = tar_custom;
	// 速度PID：目标速度与当前自定义速度偏差→输出目标扭矩
	tar_torque = Speed_PID.Positional_PID(tar_speed, this->custom_speed);
	set_torque_offset(tar_torque); 
}
/*******************************************************************************
* @功能     	: 灵足自定义控制电机类的速度设置函数，基于外部数据实现速度闭环
* @参数1        : Custom_Speed_Tar：自定义速度目标值（与Custom_Speed单位一致）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_RS_Custom_Motor::Motor_Custom_Speed_Set(float Custom_Speed_Tar)
{
	tar_speed = Custom_Speed_Tar;
	// 速度PID：自定义目标速度与当前自定义速度偏差→输出目标扭矩
	tar_torque = Speed_PID.Positional_PID(tar_speed, this->custom_speed);
	set_torque_offset(tar_torque);
}
/*******************************************************************************
 * @功能        : float型转uint16_t型浮点数
 * @参数1       : 需要转换的值
 * @参数2       : x的最小值
 * @参数3       : x的最大值
 * @参数4       : 需要转换的进制数
 * @返回值      : 十进制的uint16_t型浮点数
 * @概述        : 无需调用
 *******************************************************************************/
static int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	if (x > x_max)
		x = x_max;
	else if (x < x_min)
		x = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
/*******************************************************************************
 * @功能        : uint16_t型转float型浮点数
 * @参数1       : 需要转换的值
 * @参数2       : x的最小值
 * @参数3       : x的最大值
 * @参数4       : 需要转换的进制数
 * @返回值      : 十进制的float型浮点数
 * @概述        : 不需要你用
 *******************************************************************************/
static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
	uint32_t span = (1 << bits) - 1;
	float offset = x_max - x_min;
	return offset * x / span + x_min;
}
/*******************************************************************************
 * @功能        : Byte型转float型浮点数
 * @参数1       : 需要转换的值
 * @返回值      : 十进制的float型浮点数
 * @概述        : 不需要你用
 *******************************************************************************/
static float Byte_to_float(const uint8_t *bytedata)
{
	uint32_t data = bytedata[7] << 24 | bytedata[6] << 16 | bytedata[5] << 8 | bytedata[4];
	float data_float = *(float *)(&data);
	return data_float;
}
