#include "Init_car_arm.h"


//杆1长85mm，杆2长370mm，杆3长100mm，杆4长275mm

CanMsgQueue        Mechanical_arm_CAN2_Queue; // 达妙CAN消息队列
CanMsgQueue        Mechanical_arm_CAN3_Queue; // 灵足CAN消息队列

IFR_FDCAN_ClassDef Mechanical_arm_FDCAN2;
IFR_FDCAN_ClassDef Mechanical_arm_FDCAN3; // FDCAN对象

IFR_RS_Pos_Motor LZ01 (LZ01_Offset, 1, RobStride_02, &Mechanical_arm_CAN2_Queue),
                 LZ02 (LZ02_Offset, 2, RobStride_03, &Mechanical_arm_CAN2_Queue),
                 LZ03 (LZ03_Offset, 3, RobStride_06, &Mechanical_arm_CAN2_Queue);
//                 LZ04 (4, RobStride_00, &Mechanical_arm_CAN3_Queue),
//                 LZ05 (5, RobStride_05, &Mechanical_arm_CAN3_Queue),
//                 LZ06 (6, RobStride_05, &Mechanical_arm_CAN3_Queue);

IFR_DM_Pos_Motor DM04 (4, DM_4310, &Mechanical_arm_CAN3_Queue);
IFR_DM_Pos_Motor DM05 (5, DM_3507, &Mechanical_arm_CAN3_Queue);
IFR_DM_Pos_Motor DM06 (6, DM_3507, &Mechanical_arm_CAN3_Queue);







////////////////////////////////////////////////////////////////////////////
float LZ_speed_kp = 0.06f;
float LZ_speed_ki = 0.01f;
float LZ_speed_kd = 0.01f;
float LZ_Speed_Integral_Separation_Threshold = 6.0f;

float LZ_pos_kp = 0.06f;
float LZ_pos_ki = 0.01f;
float LZ_pos_kd = 0.01f;
float LZ_Pos_Integral_Separation_Threshold = 6.0f;



void mechanical_arm_init() 
{

  Mechanical_arm_FDCAN3.FDCAN_Init(&hfdcan3, &Mechanical_arm_CAN3_Queue); // 初始化FDCAN3，绑定消息队列
 	Mechanical_arm_FDCAN2.FDCAN_Init(&hfdcan2, &Mechanical_arm_CAN2_Queue); // 初始化FDCAN2，绑定消息队列

	Mechanical_arm_FDCAN2.RegisterMotor(&LZ01); // 注册电机对象到FDCAN控制器
	Mechanical_arm_FDCAN2.RegisterMotor(&LZ02);
	Mechanical_arm_FDCAN2.RegisterMotor(&LZ03);
	Mechanical_arm_FDCAN3.RegisterMotor(&DM04);
	Mechanical_arm_FDCAN3.RegisterMotor(&DM05);
	Mechanical_arm_FDCAN3.RegisterMotor(&DM06);


	LZ01.Speed_PID.PID_Init(5, 0, LZ_speed_kd, 10.0f, 1000.0f, 500.0f, 0, 5, 0.3f);
	LZ02.Speed_PID.PID_Init(10, 0, LZ_speed_kd, 10.0f, 1000.0f, 500.0f, 0, 5, 0.3f);
	LZ03.Speed_PID.PID_Init(7, 0, LZ_speed_kd, 10.0f, 1000.0f, 500.0f, 0.001, 5, 0.3f);
	DM04.Speed_PID.PID_Init(1, 0, LZ_speed_kd, 10.0f, 1000.0f, 500.0f, 0, 5, 0.3f);
	DM05.Speed_PID.PID_Init(1, 0, LZ_speed_kd, 10.0f, 1000.0f, 500.0f, 0, 5, 0.3f);
	DM06.Speed_PID.PID_Init(0.25, 0, LZ_speed_kd, 10.0f, 1000.0f, 500.0f, 0, 5, 0.3f);

	LZ01.Pos_PID.PID_Init(10, 0, LZ_pos_kd, 500.0f, 1000.0f, 500.0f, 0, 0.2f, 0.3f);
	LZ02.Pos_PID.PID_Init(10, 0, LZ_pos_kd, 500.0f, 1000.0f, 500.0f, 0, 0.2f, 0.3f);
	LZ03.Pos_PID.PID_Init(9, 0, LZ_pos_kd, 500.0f, 1000.0f, 500.0f, 0.01, 0.2f, 0.3f);
	DM04.Pos_PID.PID_Init(10, 0, LZ_pos_kd, 500.0f, 1000.0f, 500.0f, 0, 0.2f, 0.3f);
	DM05.Pos_PID.PID_Init(80, 0, LZ_pos_kd, 500.0f, 1000.0f, 500.0f, 0, 0.2f, 0.3f);
	DM06.Pos_PID.PID_Init(10, 0, LZ_pos_kd, 500.0f, 1000.0f, 500.0f, 0, 0.2f, 0.3f);


//	arm_inverse_6dof.Set_all_limit_angles(0.35f, 3.6f, 
//										  0.05f, 2.7f, 
//										   0.0f, 3.0f, 
//										  -2.8f, 2.8f, 
//										  -2.8f, 2.8f, 
//										  -2.8f, 2.8f);


}


float LZ03_L_G = 5.0f;
float LZ03_Offset_elt = 0.0f;

float LZ03_Offset( float angle_)
{
	
	float LZ03_offset_angle = angle_ - 3.75f;
	float LZ01_angle;
	float LZ02_angle;
	MotorData_t LZ02_data;
	LZ02_data = LZ02.get_motordata();
	LZ02_angle = LZ02_data.Angle - 1.14f;

	LZ03_Offset_elt = cos(LZ03_offset_angle - LZ02_angle) * LZ03_L_G; // 根据输入的角度计算扭矩补偿值（这里以cos函数为例，实际可以根据机械臂的特性进行调整）




	return LZ03_Offset_elt;

}



	float LZ02_LZ03_L_G = 12.0f;
//	float LZ03_G = 4.0f;
	float LZ02_Offset_elt = 0.0f;
float LZ02_Offset( float angle_ )
{
	float LZ02_offset_angle = angle_ - 1.14f;
	float LZ01_angle;
	float LZ03_angle;
	MotorData_t LZ03_data;


	LZ03_data = LZ03.get_motordata();
	LZ03_angle = LZ03_data.Angle - 3.75f;
	float G02;
//	G02 = LZ03_elt * sin(LZ03_angle - angle_) * sin(LZ03_angle - angle_);


	LZ02_Offset_elt = cos(LZ02_offset_angle ) * LZ02_LZ03_L_G - cos(LZ02_offset_angle - LZ03_angle) * LZ03_L_G; // 根据输入的角度计算扭矩补偿值（这里以cos函数为例，实际可以根据机械臂的特性进行调整）




	return LZ02_Offset_elt;

}



float Offset_k = 0;
float LZ01_Offset(float angle_)
{


	float	LZ01_Offset_elt;
	LZ01_Offset_elt = - Offset_k * (angle_ - 3.0f);

	return LZ01_Offset_elt;

}



