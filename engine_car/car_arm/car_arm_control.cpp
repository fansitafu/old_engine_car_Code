#include "car_arm_control.h"

Arm_Inverse_Kinematics_6DOF arm_inverse_6dof;




// matrix_math T_0_Matrix(4, 4);
// matrix_math T_1_Matrix(4, 4);
// matrix_math T_2_Matrix(4, 4);
// matrix_math T_3_Matrix(4, 4);
// matrix_math T_4_Matrix(4, 4);
matrix_math T_fin(4,4);
matrix_math T_tran(4,4);
matrix_math T_0_(4, 4);
matrix_math T_1_(4, 4);
matrix_math T_2_(4, 4);
matrix_math T_3_(4, 4);

float T_watch1[4][4];

float angle_z = 100;

void Arm_matrix_calculate()
{
	MotorData_t LZ01_data, LZ02_data, LZ03_data, DM04_data, DM05_data, DM06_data;
	LZ01_data = LZ01.get_motordata();
	LZ02_data = LZ02.get_motordata();
	LZ03_data = LZ03.get_motordata();
	// DM04_data = DM04.get_motordata();
	// DM05_data = DM05.get_motordata();
	// DM06_data = DM06.get_motordata();
	float LZ01_angle = LZ01_data.Angle;
	float LZ02_angle = LZ02_data.Angle;
	float LZ03_angle = LZ03_data.Angle;
	float DM04_angle = DM04_data.Angle;
	float DM05_angle = DM05_data.Angle;
	float DM06_angle = DM06_data.Angle;

	float DH_angle_LZ01 = -(LZ01_angle - 2.8f); // LZ01的DH角度需要根据实际安装位置进行调整，这里假设初始位置为3.0rad
	float DH_angle_LZ02 = -LZ02_angle;
	float DH_angle_LZ03 = LZ03_angle - pi/2;
	float DH_angle_DM04 = DM04_angle - 3.5f + pi/2 ;
	float DH_angle_DM05 = DM05_angle ;
	float DH_angle_DM06 = DM06_angle ;

	// matrix_math T_tran(4,4);
	// T_0_Matrix = DH_Matrix(0, 0, 0, DH_angle_LZ01);
	// T_1_Matrix = DH_Matrix(-pi/2, 0, 80, DH_angle_LZ02);
	// T_2_Matrix = DH_Matrix(0, 370, -80, DH_angle_LZ03);
	// T_3_Matrix = DH_Matrix(pi/2, 0, 375, DH_angle_DM04);


	arm_inverse_6dof.Update_DH(0,     0,    0,   DH_angle_LZ01,
							   -pi/2, 0,    80,  DH_angle_LZ02,
							   0,     370, -80,  DH_angle_LZ03,
							   pi/2,  0,    375, DH_angle_DM04,
							   pi/2, 0,    0,   DH_angle_DM05,
							   0,     0,    0,   DH_angle_DM06);

	T_fin = arm_inverse_6dof.arm_forward_kinematics(4);

	matrix_math target_m(4, 4) ;
	float target_m_arr[16] = {0, 0.5736, 0.8192, 100,									 
							  0, -0.8192, 0.5736, 100,
							  1, 0, 0, 100,
							  0, 0, 0, 1};
	target_m.set_data_arr(target_m_arr);
	arm_inverse_6dof.calculate_inverse_kinematics(target_m);

	// arm_inverse_6dof.coordinate_inverse_solution(-angle_z, angle_z, angle_z);
	// float R_postion[9] = {0, -1, 0,
	// 					  1, 0, 0,
	// 					  0, 0, 1};
	// matrix_math R_tran(3,3);
	// R_tran.set_data_arr(R_postion);
	// arm_inverse_6dof.posture_inverse_solution(R_tran);




	// T_0_ = arm_inverse_6dof.get_T_matrix(0);
	// T_1_ = arm_inverse_6dof.get_T_matrix(1);
	// T_2_ = arm_inverse_6dof.get_T_matrix(2);
	// T_3_ = arm_inverse_6dof.get_T_matrix(3);


	// T_tran = matrix_multiplication(T_0_, T_1_);
//	T_tran = matrix_multiplication(T_0_Matrix, T_1_Matrix);
//	 T_tran = matrix_multiplication(T_tran, T_2_Matrix);
//	 T_fin = matrix_multiplication(T_tran, T_3_Matrix);
// T_fin = matrix_multiplication( 
//             matrix_multiplication( 
//                 matrix_multiplication(T_0_, T_1_), 
//                 T_2_ 
//             ), 
//             T_3_ 
//          );

//循环将T_fin的值赋给T_watch
	for (uint8_t i = 0; i < 4; ++i)
	{
		for (uint8_t j = 0; j < 4; ++j)
		{
			T_watch1[i][j] = T_fin.get_value(i, j);
		}
	}

//	for (uint8_t i = 0; i < 4; ++i)
//	{
//		for (uint8_t j = 0; j < 4; ++j)
//		{
//			T_watch2[i][j] = T_2_Matrix.get_value(i, j);
//		}
//	}


}







