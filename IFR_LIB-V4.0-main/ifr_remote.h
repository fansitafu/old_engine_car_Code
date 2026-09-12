/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_remote.h
  * Version			: v3.0
  * Author			: LiuHao Lijiawei Albert PanJiajun 
  * Date				: 2025-10-10
  * Description	:	支持各种DJI遥控器数据，如DT7、VT13，以及遥控器回传的键鼠数据
	*
  *********************************************************************
  */
#ifndef __IFR_REMOTE_H_
#define __IFR_REMOTE_H_
/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ifr_crc.h"
#include <string.h>
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
#define REMOTE_DEAD_ZONE 3	// 遥控器死区，默认3，可以自行修改
	 
#define REMOTE_MID_VALUE 1024	// 遥控器中值，所有遥控器中值都是1024
#define RC_SWITCH_UP        1	// DT7拨杆上
#define RC_SWITCH_MIDDLE    3	// DT7拨杆中
#define RC_SWITCH_DOWN      2	// DT7拨杆下
typedef struct
{
	float Chx_Left;
	float Chy_Left;
	float Chx_Right;
	float Chy_Right;
	uint8_t  Switch_Left;
	uint8_t  Switch_Right;
	uint16_t Chz_Left;
	struct
	{
		int16_t X;
		int16_t Y;
		int16_t Z;
		uint8_t Press_L;
		uint8_t Press_R;
	}Mouse;
 struct
 {
		uint16_t V;
 } Key;
 uint8_t Updata;
} DT7_REMOTE_TypeDef;	// DT7 遥控器数据结构 32字节

// VT13协议部分
typedef __packed struct 
{ 
	uint8_t soft_1; 
	uint8_t soft_2; 
	uint64_t ch_0:11; 
	uint64_t ch_1:11; 
	uint64_t ch_2:11; 
	uint64_t ch_3:11; 
	uint64_t mode_sw:2; 
	uint64_t go_home:1; 
	uint64_t fn:1; 
	uint64_t button:1; 
	uint64_t wheel:11; 
	uint64_t shutter:1; 
	int16_t mouse_x; 
	int16_t mouse_y; 
	int16_t mouse_z; 
	uint8_t mouse_left:2; 
	uint8_t mouse_right:2; 
	uint8_t mouse_middle:2; 
	uint16_t key; 
	uint16_t crc16; 
}	VT13_Remote_Keyboard_Mouset_TypeDef;  // VT13 遥控器信息 21字节

typedef __packed struct 
{
	float Chx_Left;
	float Chy_Left;
	float Chx_Right;
	float Chy_Right;
	uint8_t mode_sw; 
	uint8_t go_home; 
	uint8_t fn; 
	uint8_t button; 
	uint16_t wheel; 
	uint8_t shutter; 
	int16_t mouse_x; 
	int16_t mouse_y; 
	int16_t mouse_z; 
	uint8_t mouse_left; 
	uint8_t mouse_right; 
	uint8_t mouse_middle; 
	uint8_t Updata;
} VT13_Remote_Keyboard_Data_TypeDef;	// VT13 遥控器处理过后信息 33字节


// 键鼠协议部分
typedef __packed struct 
{
	uint8_t KEY_W:1; 
	uint8_t KEY_S:1; 
	uint8_t KEY_A:1; 
	uint8_t KEY_D:1; 
	uint8_t KEY_SHIFT:1; 
	uint8_t KEY_CTRL:1; 
	uint8_t KEY_Q:1; 
	uint8_t KEY_E:1; 
	uint8_t KEY_R:1; 
	uint8_t KEY_F:1; 
	uint8_t KEY_G:1; 
	uint8_t KEY_Z:1; 
	uint8_t KEY_X:1; 
	uint8_t KEY_C:1; 
	uint8_t KEY_V:1; 
	uint8_t KEY_B:1; 
}	KEYBOARD_TypeDef;	// 统一的键鼠数据结构 2字节

// 天地飞WBUS协议部分
typedef __packed struct 
{
	uint8_t start_byte;	// 帧头，0x0F
	uint8_t data[22];		// 16通道数据，22字节
	uint8_t flags;			// 标志位
	uint8_t end_byte;		// 帧尾，0x00
}	WBUS_Frame_TypeDef; // WBUS完整帧格式 25字节

typedef __packed struct 
{
	uint16_t channels[16];
	uint8_t ch17;
	uint8_t ch18;
	uint8_t frame_lost;
	uint8_t failsafe_activated;
} WBUS_Data_TypeDef;	// 36字节

typedef __packed struct 
{
  float Chx_Left;		// 左摇杆X轴
	float Chy_Left;		// 左摇杆Y轴
	float Chx_Right;	// 左摇杆X轴
	float Chy_Right;	// 左摇杆Y轴
	uint8_t Switch_A;	// 拨杆A
	uint8_t Switch_B;	// 拨杆B
	uint8_t Switch_C;	// 拨杆C
	uint8_t Switch_D;	// 拨杆D
	float Dial_Left;	// 左侧旋钮
	float Dial_Right;	// 右侧旋钮
	uint8_t ch17;			// 数字通道 17（CH17）
	uint8_t ch18;			// 数字通道 18（CH18）
	uint8_t frame_lost;	// 帧丢失（Frame Lost）
	uint8_t failsafe_activated;	// 失控保护（Failsafe）
} WBUS_Remote_Data_TypeDef;	// 天地飞 遥控器信息 32字节

// 以下是ET08A的通道映射配置，请根据遥控器上的设置修改以下宏定义 或者 将遥控器上的设置修改成以下宏定义的样子
// 1. 通道映射配置：用户指定每个控制量对应16个通道中的哪一个（0-15）
#define CHX_LEFT_CHANNEL    3   // 左摇杆X轴对应通道3
#define CHY_LEFT_CHANNEL    1   // 左摇杆Y轴对应通道1
#define CHX_RIGHT_CHANNEL   0   // 右摇杆X轴对应通道0
#define CHY_RIGHT_CHANNEL   2   // 右摇杆Y轴对应通道2
#define SWITCH_A_CHANNEL    4   // 开关A对应通道4
#define SWITCH_B_CHANNEL    5   // 开关B对应通道5
#define SWITCH_C_CHANNEL    6   // 开关C对应通道6
#define SWITCH_D_CHANNEL    7   // 开关D对应通道7
#define DIAL_LEFT_CHANNEL   5   // 左侧旋钮对应通道5
#define DIAL_RIGHT_CHANNEL  6   // 右侧旋钮对应通道6
// 2. 方向配置：1=正向（原始值），-1=反向（反转值）（仅适用于摇杆类16位通道）（正负负正为常用设置，对应右上为正）
#define CHX_LEFT_DIR        1   	// 左摇杆X轴方向
#define CHY_LEFT_DIR        -1  	// 左摇杆Y轴方向
#define CHX_RIGHT_DIR       -1   	// 右摇杆X轴方向
#define CHY_RIGHT_DIR       1   	// 右摇杆Y轴方向
// 3. 开关类型配置（可选，根据遥控器开关类型设置）
// 三段开关：0-682 = 1，683-1365 = 2，1366-2047 = 3 
// 两段开关：0-1023 = 1，1024-2047 = 2
#define SWITCH_TYPE_2POS    0   // 两段开关
#define SWITCH_TYPE_3POS    1   // 三段开关
#define SWITCH_A_TYPE       SWITCH_TYPE_2POS  // 开关A为两段
#define SWITCH_B_TYPE       SWITCH_TYPE_3POS  // 开关B为三段
#define SWITCH_C_TYPE       SWITCH_TYPE_3POS  // 开关C为三段
#define SWITCH_D_TYPE       SWITCH_TYPE_2POS  // 开关D为两段

extern DT7_REMOTE_TypeDef DT7_Remote;
extern VT13_Remote_Keyboard_Data_TypeDef VT13_Remote;
extern KEYBOARD_TypeDef Keyboard;
extern WBUS_Remote_Data_TypeDef WBUS_Remote;
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

void DT7_Remote_Analysis(uint8_t *pData,uint8_t len);
void DT7_RemoteAndKeyboard_Analysis(uint8_t *pData,uint8_t len);
void VT13_Remote_Analysis(uint8_t *pData, uint8_t len);
void WBUS_Remote_Analysis(uint8_t *pData, uint8_t len);

/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */
#ifdef __cplusplus
 }
#endif
#endif
