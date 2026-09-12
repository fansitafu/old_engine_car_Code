/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_motor.cpp
  * Version			: v3.0
  * Author			: PanJiajun 
  * Date				: 2025-10-10
  * Description	:	DJI系列电机控制库，包含电机初始化、CAN消息解析、扭矩/速度/位置控制等核心功能，IFR CAN库中的 设备层，负责具体电机控制
	*
  *********************************************************************
  */
/* Includes ---------------------------------------------------------*/
#include "ifr_djimotor.h"
/*******************************************************************************
* @功能     	: DJI电机实例化的构造函数，初始化电机ID
* @参数1        : 电机ID
* @参数2        : 大疆电机类型 @arg DJI_2006 DJI_3508 DJI_6020
* @参数3        : 需要绑定的CAN消息队列
* @返回值 		: void
* @概述  		: 用于将设备层、中间层绑定、电机参数初始化
*******************************************************************************/
IFR_DJI_Motor::IFR_DJI_Motor(uint16_t Num, DJI_Motor_Type m_DJI_Motor_type, CanMsgQueue* MsgQueue)
{
	motor_init(Num, m_DJI_Motor_type, MsgQueue);
}
/*******************************************************************************
* @功能     	: DJI电机实例化带前馈补偿的的构造函数，初始化电机ID
* @参数1        : 前馈补偿函数
* @参数2        : 电机ID
* @参数3        : 大疆电机类型 @arg DJI_2006 DJI_3508 DJI_6020
* @参数4        : 需要绑定的CAN消息队列
* @返回值 		: void
* @概述  		: 用于将设备层、中间层绑定、电机参数初始化
*******************************************************************************/
IFR_DJI_Motor::IFR_DJI_Motor(float (*Offset_MotoFunc)(float Motor_Angle), uint16_t Num, DJI_Motor_Type m_DJI_Motor_type, CanMsgQueue* MsgQueue)
{
	Register_Motor_Offset_MotoFunc(Offset_MotoFunc);
	motor_init(Num, m_DJI_Motor_type, MsgQueue);
}
/*******************************************************************************
* @功能     	: DJI电机数据解析函数（不对外开放）
* @参数1        : 需要解析CAN消息
* @返回值 		: void
* @概述  		: 此函数为CAN接收中断自动触发，无需调用
*******************************************************************************/
void IFR_DJI_Motor::parse_can_msg(const CanMsg_t msg)
{
	if(msg.id == motor_id && CAN_STD == msg.can_identifier_type)
	{
		Motordata.Angle = float((uint16_t(msg.data[0] << 8) | msg.data[1])) / 8192.0f * 2.0f * IFR_PI;
		Motordata.Speed = float((int16_t(msg.data[2] << 8) | msg.data[3])) / 60.0f * 2.0f * IFR_PI;
		Motordata.Electric = float((int16_t(msg.data[4] << 8) | msg.data[5]) / 1000.0f); // / 16384.0f * 20.0f;
		Motordata.Temperature = msg.data[6];
		Motordata.motor_state = MOTOR_ENABLE;	// DJI电机只要接到消息，电机都是使能状态
		set_motor_error_code(msg.data[7]);
		if(Motordata.last_angle == 2 * IFR_PI) // 排除第一次赋值，防止第一次直接将abs_angle计算
			Motordata.last_angle = Motordata.Angle;
		if (IFR_ABS(Motordata.Speed) < 0.1f)	// 排除电机噪声
		{
			float Error = Motordata.Angle - Motordata.last_angle;
			Motordata.abs_angle += Error / this->set_reduction_ratio;
			if (Error < -IFR_PI) Motordata.abs_angle += 2 * IFR_PI / this->set_reduction_ratio;
			else if (Error > IFR_PI)  Motordata.abs_angle -= 2 * IFR_PI / this->set_reduction_ratio;
		}
		Motordata.last_angle = Motordata.Angle;
		Motordata.Updata = 1; // 判断电机是否断连标志位 置1
		Motordata.last_update_time = HAL_GetTick(); // 更新上次接到消息的时间戳
	}
}
/*******************************************************************************
* @功能     	: DJI电机设置输出电流函数
* @参数1        : 设置输出电流（单位：A）
* @返回值 		: void
* @概述  		: CANID对应关系：200 -> 201-204		1ff -> 205->208		2ff -> 209、20a、20b、
*******************************************************************************/
void IFR_DJI_Motor::set_electric(float set_electric)
{
	this->tar_electric = set_electric;
	this->tar_electric *= this->set_decrease;
	this->tar_electric = IFR_CLAMP(this->tar_electric, -this->electric_max, this->electric_max);	// 将电流值限制在最大电流值内，防止计算出负数
	int16_t set_int16_output = this->tar_electric / this->int16_to_float; // 将float的电流值转换为对应的uint16值
	CanMsg_t msg;
	if (0x201 <= motor_id && motor_id <= 0x204)
		msg.id =  0x200;
	else if (0x205 <= motor_id && motor_id <= 0x208)
		msg.id =  0x1ff;
	else if (0x209 <= motor_id && motor_id <= 0x20b)
		msg.id =  0x2ff;
	memset(msg.data, 0, 8);	// 将临时变量的数据清零，防止后续使用时临时变量的随机数出现问题
	// 如果队列中已有数据，就提取出来（一条can消息控制四个电机），从当前队列的头开始读取
	// 循环队列的正确遍历方式：按元素个数count_遍历，而非front到rear的索引范围
	uint16_t queue_count = msgQueue->size();  // 获取队列当前元素个数
	uint16_t queue_front = msgQueue->get_front();  // 获取队头索引
	for (int i = 0; i < queue_count; i++)  // 遍历所有有效元素（0 ~ count_-1）
	{
		// 计算第i个元素的实际索引（循环队列核心：对队列容量取模）
		uint16_t elem_index = (queue_front + i) % CanMsgQueue_count;
		// 检查当前元素的CAN ID是否匹配
		if (msgQueue->get_index_id(elem_index) == msg.id)
		{
			// 找到匹配的消息，拷贝数据域
			memcpy(msg.data, msgQueue->get_index_data(elem_index), 8);
			break;  // 找到后退出循环
		}
	}
	// 计算电机在数据域的索引
	uint8_t motor_idx = (motor_id - 0x201) % 4;
	// 拆分int16为高低8位
	msg.data[motor_idx * 2]     = uint8_t(set_int16_output >> 8);   // 高8位
	msg.data[motor_idx * 2 + 1] = uint8_t(set_int16_output);        // 低8位
	generate_can_msg(msg); 
}
/*******************************************************************************
* @功能     		: 设置大疆电机扭矩输出，带支持前馈补偿
* @参数1        : _set_electric：电机目标电流值（单位：A）
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void IFR_DJI_Motor::set_electric_offset(float _set_electric)
{
	this->tar_electric = _set_electric;
// 若前馈补偿函数有效，叠加角度对应的补偿扭矩
	if (Motor_Offset_MotoFunc != NULL)
		this->tar_electric += (*Motor_Offset_MotoFunc)(Motordata.Angle);
	set_electric(this->tar_electric); 
}
/*******************************************************************************
* @功能     	: 将大疆电机的CAN控制消息加入发送队列，若队列无效或满则标记错误（不对外开放）
* @参数1        : msg：待发送的CAN消息结构体（已填充扩展ID、数据等）
* @返回值 		: void
* @概述  		: 无需调用
*******************************************************************************/
void IFR_DJI_Motor::generate_can_msg(CanMsg_t& msg)
{
	msg.can_identifier_type = CAN_STD; // 标记为标准CAN帧
	// 队列有效性和空闲状态检查
	if (msgQueue != NULL)
	{
		msgQueue->enqueue(msg);
		Motordata.Updata = 0; // 判断电机是否断连标志位 置0
	}
	if (msgQueue == NULL || msgQueue->is_full())
		Errorcode = MOTOR_MSGQUEUE_ERROR; // 队列错误：无效或满
}
/*******************************************************************************
* @功能     	: DJI电机初始化函数（不对外开放）
* @参数1        : 电机ID
* @参数2        : 大疆电机类型 @arg DJI_2006 DJI_3508 DJI_6020
* @参数3        : 需要绑定的CAN消息队列
* @返回值 		: void
* @概述  		: 用于将设备层、中间层绑定、电机参数初始化，无需调用
*******************************************************************************/
void IFR_DJI_Motor::motor_init(uint16_t Num, DJI_Motor_Type m_DJI_Motor_type, CanMsgQueue* MsgQueue)
{
	motor_id = Num;
	Motordata.last_angle = 2 * IFR_PI;
	msgQueue = MsgQueue;
	this->set_decrease = 1;
	this->pos_offest = 0;
	Motordata.last_update_time = 0; // 初始化上次接到消息的时间戳
	switch(m_DJI_Motor_type)
	{
		case DJI_2006:
			this->int16_to_float = 0.001f;	// -10000~10000 对应 -10~10A
			this->electric_max = 10.0f;
			this->set_reduction_ratio = 36.0f;	// 原装减速箱减速比为36.0f			
			break;
		case DJI_3508:
			this->int16_to_float = 20.0f / 16384.0f;	// -16384~16384 对应 -20~20A
			this->electric_max = 20.0f;
			this->set_reduction_ratio = 3591.0f / 187.0f;	// 原装减速箱减速比为19.2032f
			break;
		case DJI_6020:
			this->int16_to_float = 0.00012f; // -25000~25000 对应 -3~3A
			this->electric_max = 3.0f;
			this->set_reduction_ratio = 1.0f;	// 原装无减速箱
			break;
	}
}
/*******************************************************************************
* @功能     		: DJI电机根据反馈值设定错误码函数（不对外开放）
* @参数1        : 反馈值（对应错误码数值：0/1/2/3/4/5/7/8）
* @返回值 			: void
* @概述  				: 无需调用，内部根据电机反馈值自动设置错误码
*******************************************************************************/
void IFR_DJI_Motor::set_motor_error_code(const uint8_t error_data)
{
	// 根据反馈值匹配对应的错误码枚举，每个case必须加break避免穿透
	switch (error_data)
	{
		case 0: // 无异常
				motor_error_code = DJI_MOTOR_NO_ERROR;
				break;
		case 1: // 无法访问存储芯片
				motor_error_code = DJI_MOTOR_UNABLE_ACCESS;
				break;
		case 2: // 电调供电电压过高
				motor_error_code = DJI_MOTOR_VOLTAGE_HIGH;
				break;
		case 3: // 电机三相线未接入
				motor_error_code = DJI_MOTOR_THREE_PHASE_UNCONNECTED;
				break;
		case 4: // 位置传感器数据丢失
				motor_error_code = DJI_MOTOR_POS_SENSOR_DATA_LOST;
				break;
		case 5: // 电机温度异常过高(≥180℃)
				motor_error_code = DJI_MOTOR_TEMP_ABNORMAL_HIGH;
				break;
		case 7: // 电机校准失败（无错误码6）
				motor_error_code = DJI_MOTOR_CALIBRATION_FAILED;
				break;
		case 8: // 电机过热(≥125℃)
				motor_error_code = DJI_MOTOR_TEMP_OVERHEAT;
				break;
		default: // 处理未定义的反馈值（如6、9及以上），默认设为无异常
				motor_error_code = DJI_MOTOR_NO_ERROR;
				break;
	}
}
/*******************************************************************************
* @功能     		: 大疆速度环电机类的速度设置函数，通过速度PID计算扭矩实现速度控制
* @参数1        : Speed_Tar：电机目标速度（单位：rad/s）
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void IFR_DJI_Speed_Motor::Motor_Speed_Set(float Speed_Tar)
{
	tar_speed = Speed_Tar; 
	// 速度PID计算：输入目标速度与当前速度偏差，输出目标扭矩
	tar_electric = Speed_PID.Positional_PID(tar_speed, Motordata.Speed);
	set_electric_offset(tar_electric); 
}
/*******************************************************************************
* @功能     	: 大疆位置环电机类的单圈位置设置函数，采用位置-PID+速度-PID双环控制
* @参数1        : Pos_Tar：电机单圈目标位置（单位：rad）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DJI_Pos_Motor::Motor_Pos_Set(float Pos_Tar)
{
	tar_pos = Pos_Tar; 
	// 位置PID：目标位置与当前单圈位置偏差→输出目标速度
	tar_speed = Pos_PID.Positional_PID(tar_pos, Motordata.Angle);
	// 速度前馈：直接通过目标位置差计算出期望速度前馈
	tar_speed += pos_offest * (tar_pos - last_tar_pos);			
	last_tar_pos = tar_pos;
	// 速度PID：目标速度与当前速度偏差→输出目标扭矩
	tar_electric = Speed_PID.Positional_PID(tar_speed, Motordata.Speed);
	set_electric_offset(tar_electric);
}
/*******************************************************************************
* @功能     		: 大疆位置环电机类的多圈绝对位置设置函数，支持连续多圈控制
* @参数1        : AbsPos_Tar：电机多圈绝对目标位置（单位：rad）
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void IFR_DJI_Pos_Motor::Motor_AbsPos_Set(float AbsPos_Tar)
{
	tar_pos = AbsPos_Tar; 
	// 位置PID：绝对目标位置与当前绝对位置偏差→输出目标速度
	tar_speed = Pos_PID.Positional_PID(tar_pos, Motordata.abs_angle);
	// 速度前馈：直接通过目标位置差计算出期望速度前馈
	tar_speed += pos_offest * (tar_pos - last_tar_pos);			
	last_tar_pos = tar_pos;
	// 速度PID：目标速度与当前速度偏差→输出目标扭矩
	tar_electric = Speed_PID.Positional_PID(tar_speed, Motordata.Speed);
	set_electric_offset(tar_electric); 
}
/*******************************************************************************
* @功能     		: 大疆位置环电机类的速度前馈系数设置函数，用于提高位置环响应速度
* @参数1        : pos_offest：电机速度前馈系数
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void IFR_DJI_Pos_Motor::motor_set_pos_offest(float pos_offest)
{
	this->pos_offest = pos_offest;
}
/*******************************************************************************
* @功能     	: 大疆自定义控制电机类的外部数据接收函数，用外部数据替代电机原生反馈
* @参数1        : Custom_Speed：外部输入的自定义速度（替代Motordata.Speed）
* @参数2        : Custom_Angle：外部输入的自定义角度（替代Motordata.Angle）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DJI_Custom_Motor::Motor_Custom_Get(float Custom_Speed, float Custom_Angle)
{
	this->custom_speed = Custom_Speed; // 保存自定义速度
	this->custom_angle = Custom_Angle; // 保存自定义角度
}
/*******************************************************************************
* @功能     	: 大疆自定义控制电机类的位置设置函数，基于外部数据实现位置闭环
* @参数1        : Custom_Tar：自定义位置目标值（与Custom_Angle单位一致）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DJI_Custom_Motor::Motor_Custom_Angle_Set(float Custom_Tar)
{
	tar_custom = Custom_Tar; 
	// 角度PID：自定义目标位置与当前自定义角度偏差→输出目标速度
	tar_speed = Angle_PID.Positional_PID(tar_custom, this->custom_angle);
	// 速度前馈：直接通过目标位置差计算出期望速度前馈
	tar_speed += pos_offest * (tar_custom - last_tar_custom);			
	last_tar_custom = tar_custom;
	// 速度PID：目标速度与当前自定义速度偏差→输出目标扭矩
	tar_electric = Speed_PID.Positional_PID(tar_speed, this->custom_speed);
	set_electric_offset(tar_electric); 
}
/*******************************************************************************
* @功能     	: 大疆自定义控制电机类的速度设置函数，基于外部数据实现速度闭环
* @参数1        : Custom_Speed_Tar：自定义速度目标值（与Custom_Speed单位一致）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DJI_Custom_Motor::Motor_Custom_Speed_Set(float Custom_Speed_Tar)
{
	tar_speed = Custom_Speed_Tar;
	// 速度PID：自定义目标速度与当前自定义速度偏差→输出目标扭矩
	tar_electric = Speed_PID.Positional_PID(tar_speed, this->custom_speed);
	set_electric_offset(tar_electric); 
}
