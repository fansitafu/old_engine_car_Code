#include "mechanical_arm_control.h"

//杆1长85mm，杆2长370mm，杆3长100mm，杆4长275mm

CanMsgQueue        Mechanical_arm_CAN2_Queue; // 达妙CAN消息队列
CanMsgQueue        Mechanical_arm_CAN3_Queue; // 灵足CAN消息队列

IFR_FDCAN_ClassDef Mechanical_arm_FDCAN2;
IFR_FDCAN_ClassDef Mechanical_arm_FDCAN3; // FDCAN对象

IFR_RS_Pos_Motor LZ01 (LZ01_Offset, 1, RobStride_02, &Mechanical_arm_CAN3_Queue),
                 LZ02 (LZ02_Offset, 2, RobStride_03, &Mechanical_arm_CAN3_Queue),
                 LZ03 (LZ03_Offset, 3, RobStride_02, &Mechanical_arm_CAN3_Queue);


IFR_DM_Pos_Motor DM04 (4, DM_4310,&Mechanical_arm_CAN2_Queue),
                 DM05 (5, DM_3507,&Mechanical_arm_CAN2_Queue),
                 DM06 (6, DM_3507,&Mechanical_arm_CAN2_Queue);



//Arm_Inverse_Kinematics_6DOF arm_inverse_6dof;


////////////////////////////////////////////////////////////////////////////
float LZ_speed_kp = 0.06f;
float LZ_speed_ki = 0.01f;
float LZ_speed_kd = 0.01f;
float LZ_Speed_Integral_Separation_Threshold = 6.0f;

float LZ_pos_kp = 0.06f;
float LZ_pos_ki = 0.01f;
float LZ_pos_kd = 0.01f;
float LZ_Pos_Integral_Separation_Threshold = 6.0f;


////////////////////////////////////////////////////////////////////////////
float DM_speed_kp = 0.06f;
float DM_speed_ki = 0.01f;
float DM_speed_kd = 0.01f;
float DM_Speed_Integral_Separation_Threshold = 6.0f;

float DM_pos_kp = 0.06f;
float DM_pos_ki = 0.01f;
float DM_pos_kd = 0.01f;
float DM_Pos_Integral_Separation_Threshold = 6.0f;

void mechanical_arm_init() 
{

  Mechanical_arm_FDCAN3.FDCAN_Init(&hfdcan3, &Mechanical_arm_CAN3_Queue); // 初始化FDCAN3，绑定消息队列
	Mechanical_arm_FDCAN3.RegisterMotor(&LZ01); // 注册电机对象到FDCAN控制器
	Mechanical_arm_FDCAN3.RegisterMotor(&LZ02);
	Mechanical_arm_FDCAN3.RegisterMotor(&LZ03);


	LZ01.Speed_PID.PID_Init(3, 0, LZ_speed_kd, 10.0f, 1000.0f, 500.0f, 0, 5, 0.3f);
	LZ02.Speed_PID.PID_Init(8, 0, LZ_speed_kd, 10.0f, 1000.0f, 500.0f, 0, 5, 0.3f);
	LZ03.Speed_PID.PID_Init(4, 0, LZ_speed_kd, 10.0f, 1000.0f, 500.0f, 0, 5, 0.3f);

	LZ01.Pos_PID.PID_Init(12, 0, LZ_pos_kd, 100.0f, 1000.0f, 500.0f, 0, 0.2f, 0.3f);
	LZ02.Pos_PID.PID_Init(22, 0, LZ_pos_kd, 100.0f, 1000.0f, 500.0f, 0, 0.2f, 0.3f);
	LZ03.Pos_PID.PID_Init(20, 0, LZ_pos_kd, 100.0f, 1000.0f, 500.0f, 0, 0.2f, 0.3f);





    

	Mechanical_arm_FDCAN2.FDCAN_Init(&hfdcan2, &Mechanical_arm_CAN2_Queue); // 初始化FDCAN2，绑定消息队列
	Mechanical_arm_FDCAN2.RegisterMotor(&DM04); // 注册电机对象到FDCAN控制器
	Mechanical_arm_FDCAN2.RegisterMotor(&DM05);
	Mechanical_arm_FDCAN2.RegisterMotor(&DM06);


	DM04.Speed_PID.PID_Init(0.3, 0, DM_speed_kd, 1000.0f, 1000.0f, 500.0f, 0, 5, 0.3f);
	DM05.Speed_PID.PID_Init(0.3, 0, DM_speed_kd, 20.0f, 1000.0f, 500.0f, 0, 5, 0.3f);
	DM06.Speed_PID.PID_Init(0.1, 0, DM_speed_kd, 10.0f, 1000.0f, 500.0f, 0, 5, 0.3f);

	DM04.Pos_PID.PID_Init(50, 0, DM_pos_kd, 1000.0f, 1000.0f, 500.0f, 0, 0.03, 0.3f);
	DM05.Pos_PID.PID_Init(50, 0, DM_pos_kd, 100.0f, 1000.0f, 500.0f, 0, 0.03, 0.3f);
	DM06.Pos_PID.PID_Init(50, 0, DM_pos_kd, 100.0f, 1000.0f, 500.0f, 0, 0.03, 0.3f);

//	arm_inverse_6dof.Set_all_limit_angles(0.35f, 3.6f, 
//										  0.05f, 2.7f, 
//										   0.0f, 3.0f, 
//										  -2.8f, 2.8f, 
//										  -2.8f, 2.8f, 
//										  -2.8f, 2.8f);


}


float LZ03_L_G = 3.7f;
float LZ03_Offset_elt = 0.0f;

float LZ03_Offset( float angle_)
{
	

	float LZ01_angle;
	float LZ02_angle;
	MotorData_t LZ02_data;
	LZ02_data = LZ02.get_motordata();
	LZ02_angle = LZ02_data.Angle;

	LZ03_Offset_elt = cos(angle_ - LZ02_angle) * LZ03_L_G; // 根据输入的角度计算扭矩补偿值（这里以cos函数为例，实际可以根据机械臂的特性进行调整）




	return LZ03_Offset_elt;

}



	float LZ02_LZ03_L_G = 11.5f;
//	float LZ03_G = 4.0f;
	float LZ02_Offset_elt = 0.0f;
float LZ02_Offset( float angle_ )
{

	float LZ01_angle;
	float LZ03_angle;
	MotorData_t LZ03_data;


	LZ03_data = LZ03.get_motordata();
	LZ03_angle = LZ03_data.Angle;
	float G02;
//	G02 = LZ03_elt * sin(LZ03_angle - angle_) * sin(LZ03_angle - angle_);


	LZ02_Offset_elt = cos(angle_ ) * LZ02_LZ03_L_G - cos(angle_ - LZ03_angle) * LZ03_L_G; // 根据输入的角度计算扭矩补偿值（这里以cos函数为例，实际可以根据机械臂的特性进行调整）




	return LZ02_Offset_elt;

}



float Offset_k = 0;
float LZ01_Offset(float angle_)
{


	float	LZ01_Offset_elt;
	LZ01_Offset_elt = - Offset_k * (angle_ - 3.0f);

	return LZ01_Offset_elt;

}






//// matrix_math T_0_Matrix(4, 4);
//// matrix_math T_1_Matrix(4, 4);
//// matrix_math T_2_Matrix(4, 4);
//// matrix_math T_3_Matrix(4, 4);
//// matrix_math T_4_Matrix(4, 4);
//matrix_math T_fin(4,4);
//matrix_math T_tran(4,4);
//matrix_math T_0_(4, 4);
//matrix_math T_1_(4, 4);
//matrix_math T_2_(4, 4);
//matrix_math T_3_(4, 4);

//float T_watch1[4][4];

//float angle_z = 100;

//void Arm_matrix_calculate()
//{
//	MotorData_t LZ01_data, LZ02_data, LZ03_data, DM04_data, DM05_data, DM06_data;
//	LZ01_data = LZ01.get_motordata();
//	LZ02_data = LZ02.get_motordata();
//	LZ03_data = LZ03.get_motordata();
//	DM04_data = DM04.get_motordata();
//	DM05_data = DM05.get_motordata();
//	DM06_data = DM06.get_motordata();
//	float LZ01_angle = LZ01_data.Angle;
//	float LZ02_angle = LZ02_data.Angle;
//	float LZ03_angle = LZ03_data.Angle;
//	float DM04_angle = DM04_data.Angle;
//	float DM05_angle = DM05_data.Angle;
//	float DM06_angle = DM06_data.Angle;

//	float DH_angle_LZ01 = -(LZ01_angle - 2.8f); // LZ01的DH角度需要根据实际安装位置进行调整，这里假设初始位置为3.0rad
//	float DH_angle_LZ02 = -LZ02_angle;
//	float DH_angle_LZ03 = LZ03_angle - pi/2;
//	float DH_angle_DM04 = DM04_angle - 3.5f + pi/2 ;
//	float DH_angle_DM05 = DM05_angle ;
//	float DH_angle_DM06 = DM06_angle ;

//	// matrix_math T_tran(4,4);
//	// T_0_Matrix = DH_Matrix(0, 0, 0, DH_angle_LZ01);
//	// T_1_Matrix = DH_Matrix(-pi/2, 0, 80, DH_angle_LZ02);
//	// T_2_Matrix = DH_Matrix(0, 370, -80, DH_angle_LZ03);
//	// T_3_Matrix = DH_Matrix(pi/2, 0, 375, DH_angle_DM04);


//	arm_inverse_6dof.Update_DH(0,     0,    0,   DH_angle_LZ01,
//							   -pi/2, 0,    80,  DH_angle_LZ02,
//							   0,     370, -80,  DH_angle_LZ03,
//							   pi/2,  0,    375, DH_angle_DM04,
//							   pi/2, 0,    0,   DH_angle_DM05,
//							   0,     0,    0,   DH_angle_DM06);

//	T_fin = arm_inverse_6dof.arm_forward_kinematics(4);

//	matrix_math target_m(4, 4) ;
//	float target_m_arr[16] = {0, 0.5736, 0.8192, 100,									 
//							  0, -0.8192, 0.5736, 100,
//							  1, 0, 0, 100,
//							  0, 0, 0, 1};
//	target_m.set_data_arr(target_m_arr);
//	arm_inverse_6dof.calculate_inverse_kinematics(target_m);

//	// arm_inverse_6dof.coordinate_inverse_solution(-angle_z, angle_z, angle_z);
//	// float R_postion[9] = {0, -1, 0,
//	// 					  1, 0, 0,
//	// 					  0, 0, 1};
//	// matrix_math R_tran(3,3);
//	// R_tran.set_data_arr(R_postion);
//	// arm_inverse_6dof.posture_inverse_solution(R_tran);




//	// T_0_ = arm_inverse_6dof.get_T_matrix(0);
//	// T_1_ = arm_inverse_6dof.get_T_matrix(1);
//	// T_2_ = arm_inverse_6dof.get_T_matrix(2);
//	// T_3_ = arm_inverse_6dof.get_T_matrix(3);


//	// T_tran = matrix_multiplication(T_0_, T_1_);
////	T_tran = matrix_multiplication(T_0_Matrix, T_1_Matrix);
////	 T_tran = matrix_multiplication(T_tran, T_2_Matrix);
////	 T_fin = matrix_multiplication(T_tran, T_3_Matrix);
//// T_fin = matrix_multiplication( 
////             matrix_multiplication( 
////                 matrix_multiplication(T_0_, T_1_), 
////                 T_2_ 
////             ), 
////             T_3_ 
////          );

////循环将T_fin的值赋给T_watch
//	for (uint8_t i = 0; i < 4; ++i)
//	{
//		for (uint8_t j = 0; j < 4; ++j)
//		{
//			T_watch1[i][j] = T_fin.get_value(i, j);
//		}
//	}

////	for (uint8_t i = 0; i < 4; ++i)
////	{
////		for (uint8_t j = 0; j < 4; ++j)
////		{
////			T_watch2[i][j] = T_2_Matrix.get_value(i, j);
////		}
////	}


//}






