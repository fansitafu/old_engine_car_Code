#ifndef _MY_JUDGE_H_
#define _MY_JUDGE_H_


#include "ifr_lib.h"



#define robot_state 0x201
#define shooting_heat 0x202
/*************************************

裁判系统对于车子的限制和车子的等级


*************************************/
typedef __packed struct
{
  uint8_t robot_id; 
  uint8_t robot_level; 
  uint16_t current_HP;  
  uint16_t maximum_HP; 
  uint16_t shooter_barrel_cooling_value; 
  uint16_t shooter_barrel_heat_limit; 
  uint16_t chassis_power_limit;  
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_1_barrel_heat; 
  uint16_t shooter_17mm_2_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat; 

}Judge_DataTypedef;
/***************************************************
0x201

0 1 本机器人 ID
1 1 机器人等级
2 2 机器人当前血量
4 2 机器人血量上限
6 2 机器人射击热量每秒冷却值
8 2 机器人射击热量上限
10 2 机器人底盘功率上限
12 1 电源管理模块的输出情况：
bit 0：gimbal 口输出，0 为无输出，1 为 24V 输出
bit 1：chassis 口输出，0 为无输出，1 为 24V 输出
bit 2：shooter 口输出，0 为无输出，1 为 24V 输出
****************************************************/
typedef __packed struct 
{ 
 uint8_t robot_id; 
 uint8_t robot_level; 
 uint16_t current_HP; 
 uint16_t maximum_HP; 
 uint16_t shooter_barrel_cooling_value; 
 uint16_t shooter_barrel_heat_limit; 
 uint16_t chassis_power_limit; 
 uint8_t power_management_gimbal_output : 1; 
 uint8_t power_management_chassis_output : 1; 
 uint8_t power_management_shooter_output : 1; 
}robot_status_t;


/**************************************

0x202

0 2 保留位1 
2 2 保留位2
4 4 保留位3
8 2 缓冲能量（单位：J） 
10 2 第1个17mm发射机构的射击热量 
12 2 第2个17mm发射机构的射击热量 
14 2 42mm发射机构的射击热量 
*******************************************/

typedef __packed struct 
{ 
  uint16_t reserved1; 
  uint16_t reserved2; 
  float reserved3; 
  uint16_t buffer_energy; 
  uint16_t shooter_17mm_1_barrel_heat; 
  uint16_t shooter_17mm_2_barrel_heat; 
  uint16_t shooter_42mm_barrel_heat; 
}power_heat_data_t;
void data_transmit(void);
void JudgeData_analysis(uint8_t *pdata, uint8_t len);

#endif






