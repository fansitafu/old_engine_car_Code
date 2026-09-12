/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_remote.c
  * Version			: v3.0
  * Author			: LiuHao Lijiawei Albert PanJiajun 
  * Date				: 2025-10-10
  * Description	:	支持各种DJI遥控器数据，如DT7、VT13，以及遥控器回传的键鼠数据
	*
  *********************************************************************
  */
/* Includes ---------------------------------------------------------*/
#include "ifr_remote.h"
/* Private variables -------------------------------------------------*/
static void frame_to_channels(WBUS_Frame_TypeDef *frame, WBUS_Data_TypeDef *data);
static void channels_to_remote(WBUS_Data_TypeDef *data, WBUS_Remote_Data_TypeDef *remote_data);
static uint16_t adjust_stick_dir(uint16_t raw_val, int8_t dir);
static uint8_t convert_switch(uint16_t raw_val, uint8_t switch_type);
static float bits11_to_300(uint16_t raw_val);
static float get_dial(uint16_t dial_val, uint8_t dial_channel);
static void DT7_Key_Analysis(void);

DT7_REMOTE_TypeDef DT7_Remote = {0.0f, 0.0f, 0.0f, 0.0f, RC_SWITCH_UP, RC_SWITCH_UP, 1024};
VT13_Remote_Keyboard_Data_TypeDef VT13_Remote = {0.0f, 0.0f, 0.0f, 0.0f, 1, 0, 0, 0, 1024, 0, 0, 0, 0, 0, 0, 0, 0};
KEYBOARD_TypeDef Keyboard;
WBUS_Remote_Data_TypeDef WBUS_Remote = {0};
/*******************************************************************************
* @功能     	: DJI遥控器（DT7、VT13）摇杆数据、波轮数据解析函数，带死区和归一化处理。
* @参数1        : 摇杆解析数据
* @返回值 		: float 死区和归一化处理后的处理
* @概述  		: None
*******************************************************************************/
static float djiremote_dead_zone_analysis(uint16_t rocker_data)
{
	if (rocker_data > 1024 - REMOTE_DEAD_ZONE && rocker_data < 1024 + REMOTE_DEAD_ZONE)	// 死区处理
		rocker_data = 1024;
	return ((float)rocker_data - 1024.0f) / 660.0f * 1.0f;	// 归一化
}
/*******************************************************************************
* @功能     	: 天地飞WBUS遥控器（ET08A）摇杆数据解析函数，带死区和归一化处理。
* @参数1        : 摇杆解析数据
* @返回值 		: float 死区和归一化处理后的处理
* @概述  		: None
*******************************************************************************/
static float wbusremote_dead_zone_analysis(uint16_t rocker_data)
{
	if (rocker_data > 1024 - REMOTE_DEAD_ZONE && rocker_data < 1024 + REMOTE_DEAD_ZONE)	// 死区处理
		rocker_data = 1024;
	return ((float)rocker_data - 1024.0f) / 670.0f * 1.0f;	// 归一化
}
/*******************************************************************************
* @功能     	: DJI遥控器（DT7）解析函数，无键盘数据解析。
* @参数1        : 解析数据包头指针
* @参数2        : 数据包长度
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void DT7_Remote_Analysis(uint8_t *pData,uint8_t len)
{
	if(NULL == pData)
		return;
	
	DT7_Remote.Chx_Right = djiremote_dead_zone_analysis(((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF);
	DT7_Remote.Chy_Right = djiremote_dead_zone_analysis((((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF);
	DT7_Remote.Chx_Left = djiremote_dead_zone_analysis((((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF);
	DT7_Remote.Chy_Left = djiremote_dead_zone_analysis((((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF);
	DT7_Remote.Switch_Left = ((pData[5] >> 4) & 0x000C) >> 2;
	DT7_Remote.Switch_Right = ((pData[5] >> 4) & 0x0003);
	DT7_Remote.Chz_Left = djiremote_dead_zone_analysis(((int16_t)pData[16] | ((int16_t)pData[17] << 8)) & 0x07FF);
	DT7_Remote.Updata = 1;
}
/*******************************************************************************
* @功能     	: DJI遥控器（DT7）解析函数，含键盘数据解析。
* @参数1        : 解析数据包头指针
* @参数2        : 数据包长度
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
void DT7_RemoteAndKeyboard_Analysis(uint8_t *pData,uint8_t len)
{
	if(NULL == pData)
		return;
	
	DT7_Remote.Chx_Right = djiremote_dead_zone_analysis(((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF);
	DT7_Remote.Chy_Right = djiremote_dead_zone_analysis((((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF);
	DT7_Remote.Chx_Left = djiremote_dead_zone_analysis((((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF);
	DT7_Remote.Chy_Left = djiremote_dead_zone_analysis((((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF);
	DT7_Remote.Switch_Left = ((pData[5] >> 4) & 0x000C) >> 2;
	DT7_Remote.Switch_Right = ((pData[5] >> 4) & 0x0003);
	DT7_Remote.Chz_Left = ((int16_t)pData[16] | ((int16_t)pData[17] << 8)) & 0x07FF;
	DT7_Remote.Mouse.X = pData[6] | (pData[7] << 8);   //!< Mouse X axis
	DT7_Remote.Mouse.Y = pData[8] | (pData[9] << 8);   //!< Mouse Y axis
	DT7_Remote.Mouse.Z = pData[10] | (pData[11] << 8); //!< Mouse Z axis
	DT7_Remote.Mouse.Press_L = pData[12]; //!< Mouse Left  Is Press ?
	DT7_Remote.Mouse.Press_R = pData[13]; //!< Mouse Right Is Press ?
	DT7_Remote.Key.V = (pData[14] | pData[15] << 8);
	DT7_Key_Analysis();
	DT7_Remote.Updata = 1;
}
/*******************************************************************************
* @功能     		: DJI遥控器（DT7）键鼠数据解析函数。
* @参数1        : 解析数据包头指针
* @参数2        : 数据包长度
* @返回值 			: void
* @概述  				: None
*******************************************************************************/
static void DT7_Key_Analysis(void)
{
	Keyboard.KEY_W = DT7_Remote.Key.V & 0x01;
	Keyboard.KEY_S = (DT7_Remote.Key.V & 0x02) >> 1;
	Keyboard.KEY_A = (DT7_Remote.Key.V & 0x04) >> 2;
	Keyboard.KEY_D = (DT7_Remote.Key.V & 0x08) >> 3;
	Keyboard.KEY_SHIFT = (DT7_Remote.Key.V & 0x10) >> 4;
	Keyboard.KEY_CTRL = (DT7_Remote.Key.V & 0x20) >> 5;
	Keyboard.KEY_Q = (DT7_Remote.Key.V & 0x40) >> 6;
	Keyboard.KEY_E = (DT7_Remote.Key.V & 0x80) >> 7;
	Keyboard.KEY_R = (DT7_Remote.Key.V & 0x0100) >> 8;
	Keyboard.KEY_F = (DT7_Remote.Key.V & 0x0200) >> 9;
	Keyboard.KEY_G = (DT7_Remote.Key.V & 0x0400) >> 10;
	Keyboard.KEY_Z = (DT7_Remote.Key.V & 0x0800) >> 11;
	Keyboard.KEY_X = (DT7_Remote.Key.V & 0x1000) >> 12;
	Keyboard.KEY_C = (DT7_Remote.Key.V & 0x2000) >> 13;
	Keyboard.KEY_V = (DT7_Remote.Key.V & 0x4000) >> 14;
	Keyboard.KEY_B = (DT7_Remote.Key.V & 0x8000) >> 15;
}
/*******************************************************************************
* @功能     	: DJI遥控器（VT13）解析函数，含键盘数据解析。
* @参数1        : 解析数据包头指针
* @参数2        : 数据包长度
* @返回值 		: void
* @概述  		: 新遥控器
*******************************************************************************/
void VT13_Remote_Analysis(uint8_t *pData, uint8_t len)
{
	static VT13_Remote_Keyboard_Mouset_TypeDef VT13_Remote_data_temp;
	if (len != sizeof(VT13_Remote_Keyboard_Mouset_TypeDef) || NULL == pData) 
		return;
	
	if(0xA9 == pData[0] && 0x53 == pData[1] && 1 == verify_crc16_check_sum(pData, len))
	{
		// 1、将数据提取
		memcpy((uint8_t *)&VT13_Remote_data_temp, pData, len);
		memcpy(&Keyboard, (uint16_t*)&VT13_Remote_data_temp.key, sizeof(VT13_Remote_data_temp.key));
		// 2、将数据处理解析
		VT13_Remote.Chx_Right = djiremote_dead_zone_analysis(VT13_Remote_data_temp.ch_0);
		VT13_Remote.Chy_Right = djiremote_dead_zone_analysis(VT13_Remote_data_temp.ch_1);
		VT13_Remote.Chx_Left = djiremote_dead_zone_analysis(VT13_Remote_data_temp.ch_2);
		VT13_Remote.Chy_Left = djiremote_dead_zone_analysis(VT13_Remote_data_temp.ch_3);
		VT13_Remote.button = VT13_Remote_data_temp.button;	VT13_Remote.fn = VT13_Remote_data_temp.fn;
		VT13_Remote.go_home = VT13_Remote_data_temp.go_home;	VT13_Remote.mode_sw = VT13_Remote_data_temp.mode_sw;
		VT13_Remote.mouse_left = VT13_Remote_data_temp.mouse_left;	VT13_Remote.mouse_middle = VT13_Remote_data_temp.mouse_middle;
		VT13_Remote.mouse_right = VT13_Remote_data_temp.mouse_right;	VT13_Remote.mouse_x = VT13_Remote_data_temp.mouse_x;
		VT13_Remote.mouse_y = VT13_Remote_data_temp.mouse_y;	VT13_Remote.mouse_z = VT13_Remote_data_temp.mouse_z;
		VT13_Remote.shutter = VT13_Remote_data_temp.shutter;	VT13_Remote.wheel = VT13_Remote_data_temp.wheel;
		VT13_Remote.Updata = 1;
	}
}
/*******************************************************************************
	标准的WBUS协议讲解，与SBUS协议相同。
	协议为25字节固定长度帧
字节位置	-> 字段名称	-> 长度（字节）-> 取值范围及功能描述
0			->	起始位（Start）->	1		-> 固定值 0x0F，用于帧同步。
1-22	->	通道数据			 ->	22	-> 包含 16 个比例通道的 11 位数据（共 176 位），0-2047。
23		->	标志位（Flags）->	1		-> 包含数字通道、状态指示位，具体定义见下方。
24		->	结束位（End）	 ->	1		-> 固定值 0x00，用于帧结束标识。
标志位（Flags，第 23 字节）
位			-> 	名称									-> 	取值及含义
bit7		-> 	数字通道 17（CH17）		-> 	0：关闭；1：开启（对应遥控器上的数字开关）。
bit6		-> 	数字通道 18（CH18）		-> 	0：关闭；1：开启（对应遥控器上的数字开关）。
bit5		-> 	帧丢失（Frame Lost）	-> 	0：正常；1：至少丢失一帧数据（触发接收机 LED 报警）。
bit4		-> 	失控保护（Failsafe）	-> 	0：正常；1：接收机进入失控保护状态（按预设动作执行）。
bit3-0	-> 	保留位								-> 	未定义，默认值为 0。
*******************************************************************************/
/*******************************************************************************
* @功能     	: 标准的WBUS解析函数，与SBUS协议相同。
* @参数1        : 解析数据包头指针
* @参数2        : 数据包长度
* @返回值 		: void
* @概述  		: 用于给天地飞系列遥控器数据解析
*******************************************************************************/
void WBUS_Remote_Analysis(uint8_t *pData, uint8_t len)
{
	uint8_t index = 0;
	WBUS_Frame_TypeDef *wbus_frame;
	WBUS_Data_TypeDef wbus_data;
	while(index + sizeof(WBUS_Frame_TypeDef) <= len)
	{
		if (0x0F == pData[index] && 0x00 == pData[index + 24])	// 循环查找帧头和帧尾
		{
			wbus_frame = (WBUS_Frame_TypeDef*)&pData[index];
			frame_to_channels(wbus_frame, &wbus_data);
			channels_to_remote(&wbus_data, &WBUS_Remote);
		}
		index++;	//	将索引后移
	}
}
/*******************************************************************************
* @功能     	: 将数据帧中的22个字节的数据转成16通道数据和标志位数据（不对外开放）
* @参数1        : 解析数据包帧指针
* @参数2        : 解析数据包通道数据指针
* @返回值 		: void
* @概述  		: 用于给天地飞系列遥控器数据解析
*******************************************************************************/
static void frame_to_channels(WBUS_Frame_TypeDef *frame, WBUS_Data_TypeDef *data)
{
	// 解码16个通道的11位数据，共16个通道，一个通道11位，共22字节
	data->channels[0]  = (frame->data[0]    | (frame->data[1]  << 8)) & 0x07FF;
	data->channels[1]  = (frame->data[1]>>3 | (frame->data[2]  << 5)) & 0x07FF;
	data->channels[2]  = (frame->data[2]>>6 | (frame->data[3]  << 2) | (frame->data[4] << 10)) & 0x07FF;
	data->channels[3]  = (frame->data[4]>>1 | (frame->data[5]  << 7)) & 0x07FF;
	data->channels[4]  = (frame->data[5]>>4 | (frame->data[6]  << 4)) & 0x07FF;
	data->channels[5]  = (frame->data[6]>>7 | (frame->data[7]  << 1) | (frame->data[8] << 9)) & 0x07FF;
	data->channels[6]  = (frame->data[8]>>2 | (frame->data[9]  << 6)) & 0x07FF;
	data->channels[7]  = (frame->data[9]>>5 | (frame->data[10] << 3)) & 0x07FF;
	data->channels[8]  = (frame->data[11]   | (frame->data[12] << 8)) & 0x07FF;
	data->channels[9]  = (frame->data[12]>>3 | (frame->data[13] << 5)) & 0x07FF;
	data->channels[10] = (frame->data[13]>>6 | (frame->data[14] << 2) | (frame->data[15] << 10)) & 0x07FF;
	data->channels[11] = (frame->data[15]>>1 | (frame->data[16] << 7)) & 0x07FF;
	data->channels[12] = (frame->data[16]>>4 | (frame->data[17] << 4)) & 0x07FF;
	data->channels[13] = (frame->data[17]>>7 | (frame->data[18] << 1) | (frame->data[19] << 9)) & 0x07FF;
	data->channels[14] = (frame->data[19]>>2 | (frame->data[20] << 6)) & 0x07FF;
	data->channels[15] = (frame->data[20]>>5 | (frame->data[21] << 3)) & 0x07FF;
	data->ch17 = (frame->flags & 0x80) != 0;				// 数字通道 17（CH17）
	data->ch18 = (frame->flags & 0x40) != 0;				// 数字通道 18（CH18）
	data->frame_lost = (frame->flags & 0x20) != 0;	// 帧丢失（Frame Lost）
	data->failsafe_activated = (frame->flags & 0x10) != 0;	// 失控保护（Failsafe）
}
/******************************************************************************* 
* @功能     	: 将数据帧中的22个字节的数据转成常用的遥控器摇杆数据（可根据遥控器的设置 自定义摇杆、拨杆、旋钮对应的通道）（不对外开放）
* @参数1        : 解析数据包通道数据指针
* @参数2        : 对外开放的遥控器数据指针
* @返回值 		: void
* @概述  		: 用于给天地飞系列遥控器数据解析
*******************************************************************************/
static void channels_to_remote(WBUS_Data_TypeDef *data, WBUS_Remote_Data_TypeDef *remote_data)
{
	remote_data->Chx_Left  = wbusremote_dead_zone_analysis(adjust_stick_dir(data->channels[CHX_LEFT_CHANNEL], CHX_LEFT_DIR));
	remote_data->Chy_Left  = wbusremote_dead_zone_analysis(adjust_stick_dir(data->channels[CHY_LEFT_CHANNEL], CHY_LEFT_DIR));
	remote_data->Chx_Right = wbusremote_dead_zone_analysis(adjust_stick_dir(data->channels[CHX_RIGHT_CHANNEL], CHX_RIGHT_DIR));
	remote_data->Chy_Right = wbusremote_dead_zone_analysis(adjust_stick_dir(data->channels[CHY_RIGHT_CHANNEL], CHY_RIGHT_DIR));	
	remote_data->Switch_A = convert_switch(data->channels[SWITCH_A_CHANNEL], SWITCH_A_TYPE);
	remote_data->Switch_B = convert_switch(data->channels[SWITCH_B_CHANNEL], SWITCH_B_TYPE);	
	remote_data->Switch_C = convert_switch(data->channels[SWITCH_C_CHANNEL], SWITCH_C_TYPE);	
	remote_data->Switch_D = convert_switch(data->channels[SWITCH_D_CHANNEL], SWITCH_D_TYPE);	
	remote_data->Dial_Left = get_dial(data->channels[DIAL_LEFT_CHANNEL], DIAL_LEFT_CHANNEL);
	remote_data->Dial_Right = get_dial(data->channels[DIAL_RIGHT_CHANNEL], DIAL_RIGHT_CHANNEL);
	remote_data->ch17 = data->ch17;
	remote_data->ch18 = data->ch18;
	remote_data->frame_lost = data->frame_lost;
	remote_data->failsafe_activated = data->failsafe_activated;
}
/*******************************************************************************
* @功能     	: 处理摇杆数据正反向（不对外开放）
* @参数1        : 0-2047的原始数据
* @参数2        : 是否正反向
* @返回值 		: uint16_t
* @概述  		: 用于处理摇杆数据
*******************************************************************************/
static uint16_t adjust_stick_dir(uint16_t raw_val, int8_t dir) 
{
	if (-1 == dir) 
		return 2048- raw_val;  // 反向：原始值反转（0<->2047，1024不变）
	else 
		return raw_val;         // 正向：保持原始值
}
/*******************************************************************************
* @功能     	: 将11位原始值（0-2047）转换为拨杆开关状态（不对外开放）
* @参数1        : 0-2047的原始数据
* @参数2        : switch_type（拨杆类型）
* @返回值 		: uint8_t
* @概述  		: 用于处理拨杆开关	开关位置对应分别为 688 1359
*******************************************************************************/
static uint8_t convert_switch(uint16_t raw_val, uint8_t switch_type) 
{
	if (SWITCH_TYPE_3POS == switch_type)
	{
		// 三段开关（取三段的各一半位置来判断）：0-688 = 1，688-1359 = 2，1359-2047 = 3
		if (688 >= raw_val) 
			return 1;
		else if (1359 >= raw_val) 
			return 2;
		else
			return 3;
	}
	else
		// 两段开关：0 - 1023 = 1，1024 - 2047 = 2
		return (1023 >= raw_val) ? 1 : 2;
}
/*******************************************************************************
* @功能     		: 将11位原始值（0-2047）转换为-151.8-151.8（不对外开放）
* @参数1        : 11位原始值（0-2047）
* @返回值 			: float
* @概述  				: 用于处理旋钮数据	开关位置对应分别为 353 1024 1694
*******************************************************************************/
static float bits11_to_300(uint16_t raw_val) 
{
	return (float)(raw_val - 1024.0f) / 2048.0f * 305.67f;
}
/*******************************************************************************
* @功能     		: 将通道中旋钮的数据提取出来转换为0-100（不对外开放）
* @参数1        : 11位原始值（0-2047）
* @参数2        : 旋钮对应的通道名
* @返回值 			: void
* @概述  				: 用于处理旋钮数据	开关位置对应分别为 353 1024 1694
*******************************************************************************/
static float get_dial(uint16_t dial_val, uint8_t dial_channel) 
{
	int switch_channels_num[4] = {SWITCH_A_CHANNEL, SWITCH_B_CHANNEL, SWITCH_C_CHANNEL, SWITCH_D_CHANNEL};
	int switch_type_num[4] = {SWITCH_A_TYPE, SWITCH_B_TYPE, SWITCH_C_TYPE, SWITCH_D_TYPE};
	uint8_t dial_channel_type = 0;
	float dial_val_to_100 = 0;
	for (int index = 0; index < 4; index++)
	{
		if (dial_channel == switch_channels_num[index])
			dial_channel_type = switch_type_num[index];
	}
	float dial_val_to_300 = bits11_to_300(dial_val);	// 先将0-2047单位制转换为-152-152单位制
	if (SWITCH_TYPE_3POS == dial_channel_type)
	{
		// 三段开关：-152--50 = 1，-50-50 = 2，50-152 = 3
		if (dial_val_to_300 < -50) 
			dial_val_to_100 = 100 - ((float)(dial_val_to_300 - (-130.0f)) / 60.0f * 100.0f);
		else if (dial_val_to_300 < 50) 
			dial_val_to_100 = 100 - ((float)(dial_val_to_300 - (-30.0f)) / 60.0f * 100.0f);
		else 
			dial_val_to_100 = 100 - ((float)(dial_val_to_300 - (70.1f)) / 60.0f * 100.0f);
	}
	else
	{
		// 两段开关：-150 - 0 = 1，0 - 150 = 2
		if (dial_val_to_300 < 0)
			dial_val_to_100 = 100 - ((dial_val_to_300 + (-130.0f)) / 60.0f * 100.0f);
		else 
			dial_val_to_100 = 100 - ((dial_val_to_300 + (70.0f)) / 60.0f * 100.0f);
	}
	return dial_val_to_100;
}

