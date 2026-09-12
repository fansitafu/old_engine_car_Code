/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_dmmotor.cpp
  * Version			: v2.0
  * Author			: Albert PanJiajun 
  * Date				: 2025-10-10
  * Description	:	DM系列电机控制库，包含电机初始化、CAN消息解析、扭矩/速度/位置控制等核心功能，IFR CAN库中的 设备层，负责具体电机控制
	*
  *********************************************************************
  */
/* Includes ---------------------------------------------------------*/
#include "ifr_dmmotor.h"
/*******************************************************************************
* @功能     	: DM电机类无前馈函数、无扭矩上限的构造函数，调用初始化函数完成电机基础配置
* @参数1        : Motor_id：电机ID（用于CAN通信寻址）
* @参数2        : MsgQueue：CAN消息队列指针（用于存储待发送的电机控制消息）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
IFR_DM_Motor::IFR_DM_Motor(uint8_t Motor_id, DM_Motor_Type DM_Motor_type, CanMsgQueue* MsgQueue)
{
	motor_init(Motor_id, MsgQueue, DM_Motor_type);
	this->_t_max = DM_motor_prame.T_MAX;
}
/*******************************************************************************
* @功能     	: DM电机类带前馈函数、无扭矩上限的构造函数，初始化电机并绑定扭矩前馈补偿函数
* @参数1        : Offset_MotoFunc：扭矩前馈补偿函数指针（输入电机当前角度，输出补偿扭矩）
* @参数2        : Motor_id：电机ID（用于CAN通信寻址）
* @参数3        : MsgQueue：CAN消息队列指针（用于存储待发送的电机控制消息）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
IFR_DM_Motor::IFR_DM_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t Motor_id, DM_Motor_Type DM_Motor_type, CanMsgQueue* MsgQueue)
{
	motor_init(Motor_id, MsgQueue, DM_Motor_type);
	Register_Motor_Offset_MotoFunc(Offset_MotoFunc);
	this->_t_max = DM_motor_prame.T_MAX;
}
/*******************************************************************************
* @功能     	: DM电机类无前馈函数、带扭矩上限的构造函数，初始化电机并设置扭矩输出上限
* @参数1        : Motor_id：电机ID（用于CAN通信寻址）
* @参数2        : T_Max：电机最大允许输出扭矩（用于限制扭矩溢出）
* @参数3        : MsgQueue：CAN消息队列指针（用于存储待发送的电机控制消息）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
IFR_DM_Motor::IFR_DM_Motor(uint8_t Motor_id, DM_Motor_Type DM_Motor_type, float T_Max, CanMsgQueue* MsgQueue)
{
	motor_init(Motor_id, MsgQueue, DM_Motor_type);
	this->_t_max = T_Max; 
}
/*******************************************************************************
* @功能     	: DM电机类带前馈函数、带扭矩上限的构造函数，初始化电机、绑定前馈函数并设置扭矩上限
* @参数1        : Offset_MotoFunc：扭矩前馈补偿函数指针（输入电机当前角度，输出补偿扭矩）
* @参数2        : Motor_id：电机ID（用于CAN通信寻址）
* @参数3        : T_Max：电机最大允许输出扭矩（用于限制扭矩溢出）
* @参数4        : MsgQueue：CAN消息队列指针（用于存储待发送的电机控制消息）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
IFR_DM_Motor::IFR_DM_Motor(float (*Offset_MotoFunc)(float Motor_Tar), uint8_t Motor_id, DM_Motor_Type DM_Motor_type, float T_Max, CanMsgQueue* MsgQueue)
{
	motor_init(Motor_id, MsgQueue, DM_Motor_type);
	Register_Motor_Offset_MotoFunc(Offset_MotoFunc);
	this->_t_max = T_Max; 
}
/*******************************************************************************
* @功能     	: DM电机类初始化函数（不对外开放）
* @参数1        : Motor_id：电机ID（用于CAN通信寻址）
* @参数2        : MsgQueue：CAN消息队列指针（用于存储待发送的电机控制消息）
* @参数3        : DM_Motor_type：达妙电机型号
* @参数4        : T_Max：电机最大允许输出扭矩（用于限制扭矩溢出）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DM_Motor::motor_init(uint8_t Motor_id, CanMsgQueue* MsgQueue, DM_Motor_Type DM_Motor_type)
{
	this->motor_id = Motor_id; 
	this->msgQueue = MsgQueue; 
	this->Motordata.last_angle = 2 * IFR_PI; 
	this->set_decrease = 1; 
	this->pos_offest = 0;
	this->set_reduction_ratio = 1.0f;	// 默认无需处理减速比
	Motordata.last_update_time = 0; // 初始化上次接到消息的时间戳
	switch (DM_Motor_type)
	{
		case DM_4310:
			this->DM_motor_prame.set(-12.5f, 12.5f, -30.0f, 30.0f, 0.0f, 500.0f, 0.0f, 5.0f, -10.0f, 10.0f);
			break;
		case DM_3507:
			this->DM_motor_prame.set(-12.566f, 12.566f, -50.0f, 50.0f, 0 ,500.0f, 0, 5.0f, -5.0f, 5.0f);
			break;
		case DM_8009:
			break;
	}
}
/*******************************************************************************
* @功能     	: 解析电机反馈的CAN消息，提取电机状态（角度、速度、扭矩、温度等）并计算多圈绝对角度（不对外开放）
* @参数1        : msg：CAN消息结构体（包含消息ID、数据长度、数据内容等）
* @返回值 		: void
* @概述  		: 此函数为CAN接收中断自动触发，无需外部调用
*******************************************************************************/
float a__;
float ang__;



void IFR_DM_Motor::parse_can_msg(const CanMsg_t msg)
{
	// 筛选条件：标准CAN帧、主ID匹配、数据帧首字节低4位为当前电机ID
	if(CAN_STD == msg.can_identifier_type&& MasterCanID == msg.id && (msg.data[0]&0x0F) == get_motor_id())	
	{


		if(5 == get_motor_id())
		{
			a__ = 1;
			uint16_t p_int;  // 角度原始整数数据（16位）
			uint16_t v_int;  // 速度原始整数数据（12位）
			uint16_t t_int;  // 扭矩原始整数数据（12位）
			// 从CAN数据字节中提取角度、速度、扭矩的原始整数（按电机通信协议拼接字节）
			p_int = (((uint16_t)msg.data[1]<<8)&0xFF00) | (((uint16_t)msg.data[2]) & 0x00FF); // 数据1（高8位）+ 数据2（低8位）
			v_int = (((uint16_t)msg.data[3]<<4)&0x0FF0) | (((uint16_t)msg.data[4]>>4) & 0x000F); // 数据3（高8位左移4）+ 数据4（高4位）
			t_int = (((uint16_t)msg.data[4]<<8)&0x0F00) | (((uint16_t)msg.data[5]) & 0x00FF); // 数据4（低4位左移8）+ 数据5（低8位）
			ang__ = p_int;  // 16位角度整数转浮点数（范围±P_MAX）


			// 将原始整数转换为实际物理量（角度、速度、扭矩）
			Motordata.Angle	= uint_to_float(p_int, DM_motor_prame.P_MAX, 16);   // 16位角度整数转浮点数（范围±P_MAX）
			Motordata.Speed = uint_to_float(v_int, DM_motor_prame.V_MAX, 12);   // 12位速度整数转浮点数（范围±V_MAX）
			Motordata.Torque = uint_to_float(t_int, DM_motor_prame.T_MAX, 12); // 12位扭矩整数转浮点数（范围±T_MAX）

		}











		uint16_t p_int;  // 角度原始整数数据（16位）
		uint16_t v_int;  // 速度原始整数数据（12位）
		uint16_t t_int;  // 扭矩原始整数数据（12位）
		// 从CAN数据字节中提取角度、速度、扭矩的原始整数（按电机通信协议拼接字节）
		p_int = (((uint16_t)msg.data[1]<<8)&0xFF00) | (((uint16_t)msg.data[2]) & 0x00FF); // 数据1（高8位）+ 数据2（低8位）
		v_int = (((uint16_t)msg.data[3]<<4)&0x0FF0) | (((uint16_t)msg.data[4]>>4) & 0x000F); // 数据3（高8位左移4）+ 数据4（高4位）
		t_int = (((uint16_t)msg.data[4]<<8)&0x0F00) | (((uint16_t)msg.data[5]) & 0x00FF); // 数据4（低4位左移8）+ 数据5（低8位）
		// 将原始整数转换为实际物理量（角度、速度、扭矩）
		Motordata.Angle	= uint_to_float(p_int, DM_motor_prame.P_MAX, 16);   // 16位角度整数转浮点数（范围±P_MAX）
		Motordata.Speed = uint_to_float(v_int, DM_motor_prame.V_MAX, 12);   // 12位速度整数转浮点数（范围±V_MAX）
		Motordata.Torque = uint_to_float(t_int, DM_motor_prame.T_MAX, 12); // 12位扭矩整数转浮点数（范围±T_MAX）
		// 提取电机错误状态（数据0高4位），若存在错误且回调函数有效则触发错误回调
		DM_State = (ErrCodeTypedef)((msg.data[0]>>4)&0x0F);
		if (DM_State < 8 && DM_Error_Callback != NULL) 
			DM_Error_Callback();
		// 更新电机使能状态（根据错误状态判断：仅当状态为使能时标记为MODE_ENABLE）
		if (DM_State == DMState_Enable)
			Motordata.motor_state = MOTOR_ENABLE;
		else
			Motordata.motor_state = MOTOR_DISABLE;
		// 提取电机温度信息（数据6：MOS管温度；数据7：线圈温度）
		Motordata.Temperature = T_MOS = msg.data[6];	
		T_Rotor = msg.data[7];	
		if (Motordata.last_angle == 2 * IFR_PI)	// 初始赋值，排除第一次赋值，防止第一次直接将abs_angle计算
			Motordata.last_angle = Motordata.Angle; 
		float Error = Motordata.Angle - Motordata.last_angle; // 当前角度与上一帧角度的差值
		// 当电机速度大于阈值（过滤噪声）时，计算多圈绝对角度（处理角度圈数溢出）
		if (IFR_ABS(Error) > 0.01f)	
		{
			Motordata.abs_angle += Error / this->set_reduction_ratio; // 累加角度差值得到绝对角度
			// 处理角度溢出：当差值小于-PMAX（顺时针多转一圈），绝对角度加2*PMAX；反之减2*PMAX
			if (Error < -DM_motor_prame.P_MAX) 
				Motordata.abs_angle += DM_motor_prame.P_MAX * 2.0f / this->set_reduction_ratio;
			else if (Error > DM_motor_prame.P_MAX)  
				Motordata.abs_angle -= DM_motor_prame.P_MAX * 2.0f / this->set_reduction_ratio;
		}
		Motordata.last_angle = Motordata.Angle; 
		Motordata.Updata = 1; // 判断电机是否断连标志位 置1
		Motordata.last_update_time = HAL_GetTick(); // 更新上次接到消息的时间戳
	}
}
/*******************************************************************************
* @功能     		: 设置电机扭矩输出值，考虑扭矩前馈补偿并限制上下限，最终生成CAN控制消息
* @参数1        : set_torque：电机输出扭矩的目标设置值（单位根据电机参数定义，如N·m）
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void IFR_DM_Motor::set_torque(float set_torque)
{
	this->tar_torque = set_torque;
	this->tar_torque *= this->set_decrease;
	// 扭矩上下限限制（防止扭矩输出溢出损坏电机）
	tar_torque = IFR_CLAMP(tar_torque, -_t_max, _t_max);		
	// 将限制后的浮点数扭矩转换为12位整数
	uint16_t _t_ff = float_to_uint(tar_torque, _t_max, 12);
	// 构造CAN控制消息（扭矩控制指令）
	CanMsg_t msg;
	msg.id =  get_motor_id(); 
	memset(msg.data, 0, 8);   
	msg.data[6] = (_t_ff >> 8) & 0x0F; // 扭矩整数高4位存入数据6
	msg.data[7] = (uint8_t)_t_ff;      // 扭矩整数低8位存入数据7
	generate_can_msg(msg);
}
/*******************************************************************************
* @功能     		: 设置DM电机扭矩输出，带支持前馈补偿
* @参数1        : set_torque：电机目标扭矩值（单位：N·m）
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
void IFR_DM_Motor::set_torque_offset(float _set_torque)
{
	this->tar_torque = _set_torque;
// 若前馈补偿函数有效，叠加角度对应的补偿扭矩
	if (Motor_Offset_MotoFunc != NULL)
		this->tar_torque += (*Motor_Offset_MotoFunc)(Motordata.Angle);
	set_torque(this->tar_torque); 
}
/*******************************************************************************
* @功能     	: 电机使能函数
* @参数1        : None
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DM_Motor::Enable()
{
	this->DM_Motor_Command(DM_Enable);
}
/*******************************************************************************
* @功能     	: 电机失能函数（clear_error为1则清除错误标志）
* @参数1        : None
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DM_Motor::Disable(uint8_t clear_error)
{
	this->DM_Motor_Command(DM_Disable);
	if (clear_error == 1)
		this->DM_Motor_Command(DM_ClearError);
}
/*******************************************************************************
* @功能     	: 发送电机控制命令（如使能、失能、清零等），生成对应的CAN控制消息
* @参数1        : Command：电机控制命令（DM_Command_t枚举类型，定义了各类控制指令） @arg  DM_Enable DM_Disable DM_SaveZero DM_ClearError
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DM_Motor::DM_Motor_Command(DM_Command_t Command)
{
	CanMsg_t msg;
	msg.id =  get_motor_id(); // 设置CAN消息ID为当前电机ID
	memset(msg.data, 0, 8); 
	// 数据0~6设置为0xFF（电机命令协议要求的固定填充值）
	for (uint8_t i = 0; i < 7; i++) 
		msg.data[i] = 0xFF;
	msg.data[7] = (uint8_t)Command; // 数据7存入控制命令
	generate_can_msg(msg);
}
/*******************************************************************************
* @功能     	: 将电机控制CAN消息加入发送队列，若队列无效或满则标记队列错误（不对外开放）
* @参数1        : msg：待发送的CAN消息结构体（已填充ID、数据等信息）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DM_Motor::generate_can_msg(CanMsg_t& msg)
{
	msg.can_identifier_type = CAN_STD; // 标记为标准CAN帧
	// 若消息队列有效且未满，将消息入队；否则标记队列错误
	if (msgQueue != NULL)
	{
		msgQueue->enqueue(msg);
		Motordata.Updata = 0; // 判断电机是否断连标志位 置0
	}
	if (msgQueue == NULL || msgQueue->is_full())
		Errorcode = MOTOR_MSGQUEUE_ERROR;
}
/*******************************************************************************
* @功能     	: DM速度环电机类的速度设置函数，通过速度PID计算目标扭矩，间接控制电机速度
* @参数1        : Speed_Tar：电机目标速度（单位根据电机参数定义，如rad/s）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DM_Speed_Motor::Motor_Speed_Set(float Speed_Tar)
{
	tar_speed = Speed_Tar;
	// 速度PID计算：输入目标速度与当前速度的偏差，输出目标扭矩
	tar_torque = Speed_PID.Positional_PID(tar_speed, Motordata.Speed);
	set_torque_offset(tar_torque);
}
/*******************************************************************************
* @功能     	: DM位置环电机类的单圈位置设置函数，通过位置-PID+速度-PID双环控制实现位置跟踪
* @参数1        : Pos_Tar：电机单圈目标位置（单位根据电机参数定义，如rad）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DM_Pos_Motor::Motor_Pos_Set(float Pos_Tar)
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
* @功能     	: DM位置环电机类的多圈绝对位置设置函数，基于多圈绝对角度实现连续位置控制
* @参数1        : AbsPos_Tar：电机多圈绝对目标位置（单位根据电机参数定义，如rad）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DM_Pos_Motor::Motor_AbsPos_Set(float AbsPos_Tar)
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
* @功能     	: DM位置环电机类的速度前馈系数设置函数，用于提高位置环响应速度
* @参数1        : pos_offest：电机速度前馈系数
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DM_Pos_Motor::motor_set_pos_offest(float pos_offest)
{
	this->pos_offest = pos_offest;
}
/*******************************************************************************
* @功能     	: DM自定义控制电机类的外部数据获取函数，接收外部输入的速度和角度（替代电机自身反馈）
* @参数1        : Custom_Speed：外部输入的自定义速度（用于替代Motordata.Speed）
* @参数2        : Custom_Angle：外部输入的自定义角度（用于替代Motordata.Angle）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DM_Custom_Motor::Motor_Custom_Get(float Custom_Speed, float Custom_Angle)
{
	this->custom_speed = Custom_Speed; // 保存自定义速度
	this->custom_angle = Custom_Angle; // 保存自定义角度
}
/*******************************************************************************
* @功能     	: DM自定义控制电机类的位置设置函数，基于外部输入的自定义数据实现位置控制
* @参数1        : Custom_Tar：自定义位置控制的目标值（与Custom_Angle单位一致）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DM_Custom_Motor::Motor_Custom_Angle_Set(float Custom_Tar)
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
* @功能     	: DM自定义控制电机类的速度设置函数，基于外部输入的自定义速度实现速度控制
* @参数1        : Custom_Speed_Tar：自定义速度控制的目标值（与Custom_Speed单位一致）
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void IFR_DM_Custom_Motor::Motor_Custom_Speed_Set(float Custom_Speed_Tar)
{
	tar_speed = Custom_Speed_Tar; 
	// 速度PID计算：输入自定义目标速度与当前自定义速度的偏差，输出目标扭矩
	tar_torque = Speed_PID.Positional_PID(tar_speed, this->custom_speed);
	set_torque(tar_torque); 
}
// 以下为电机数据转换函数，用于将uint数据和float数据互相转换（无需外部调用）
inline float IFR_DM_Motor::uint_to_float(int x_int, float x_max, int bits){
	return ((float)x_int)*(x_max*2.0f)/((float)((1<<bits)-1)) - x_max;
}
inline int IFR_DM_Motor::float_to_uint(float x, float x_max, int bits){
	return (int) ((x+x_max)*((float)((1<<bits)-1))/(x_max*2.0f));
}
inline float IFR_DM_Motor::uint_to_float(int x_int, float x_min, float x_max, int bits){
	return ((float)x_int)*(x_max - x_min)/((float)((1<<bits)-1)) + x_min;
}
inline int IFR_DM_Motor::float_to_uint(float x, float x_min, float x_max, int bits){
	return (int) ((x-x_min)*((float)((1<<bits)-1))/(x_max - x_min));
}
