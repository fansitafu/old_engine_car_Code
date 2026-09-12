#ifndef CAR_BASIS_CONTROL_H
#define CAR_BASIS_CONTROL_H

#include "ifr_lib.h"
#include "fdcan.h"
#include "self_codebase_include.h"
#include "bmi088.h"


#define REMOTE_TO_SPEED 500.0f // 遥控器转速度比例


typedef struct
{
	
	float dir_x;
	float dir_y;
	float yaw;
	

}Move_Coordinate ;

extern CanMsgQueue        Engine_Basis_CAN1_Queue; // CAN消息队列
extern IFR_FDCAN_ClassDef Engine_Basis_FDCAN1; // FDCAN对象
extern IFR_DJI_Speed_Motor DJ_Motor1, DJ_Motor2, DJ_Motor3, DJ_Motor4; // 电机对象

extern StateConfig_Extend Car_Move_State; // 车体状态
extern StateConfig_Extend Car_Stop_State; // 车体状态
extern Move_Coordinate Basis_Move_Coordinate; // 车辆坐标结构体

extern IFR_PID Direction_angle_PID;

extern float Direction_angle_tar;

extern float DJ_Motor1_Speed_Tar; // 电机目标速度
extern float DJ_Motor2_Speed_Tar; // 电机目标速度
extern float DJ_Motor3_Speed_Tar; // 电机目标速度
extern float DJ_Motor4_Speed_Tar; // 电机目标速度



void Init_car_basis(void);
void Basis_Move_onStay_state(StateMachine* fsm, uint32_t elapsedMs);
void Basis_stop_onStay_state(StateMachine* fsm, uint32_t elapsedMs);

void Basis_Motor_Date_Updating();

void Half_judge(void);















#endif // CAR_BASIS_CONTROL_H