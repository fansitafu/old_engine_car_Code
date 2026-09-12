#include "arm_inverse_kinematics.h"



//float w1;
//float w2;
//float w3;
//float w4;


//int Arm_Inverse_Kinematics_6DOF::trigonometric_function(float A_, float B_, float C_, float *u1, float *u2)
//{
//    int num_solutions = solve_quadratic_equation(A_ + C_, -2 * B_, C_ - A_, u1, u2);
//    return num_solutions;
//}





//void Arm_Inverse_Kinematics_6DOF::calculate_inverse_kinematics(const matrix_math &target_matrix)
//{

//    this->target_matrix = target_matrix;
//    // matrix_math R_position(3, 3);
//    // for (uint8_t i = 0; i < 3; ++i)
//    // {
//    //     for (uint8_t j = 0; j < 3; ++j)
//    //     {
//    //         R_position.set_value(i, j, target_matrix.get_value(i, j));
//    //     }
//    // }

//    float P_x = target_matrix.get_value(0, 3);
//    float P_y = target_matrix.get_value(1, 3);
//    float P_z = target_matrix.get_value(2, 3);

//    matrix_math T_3_point_5_to_0 = coordinate_inverse_solution(P_x, P_y, P_z);

//    matrix_math R_0_to_3_point_5(3, 3);
//    for (uint8_t i = 0; i < 3; ++i)
//    {
//        for (uint8_t j = 0; j < 3; ++j)
//        {
//            R_0_to_3_point_5.set_value(i, j, T_3_point_5_to_0.get_value(i, j));
//        }
//    }
//    R_0_to_3_point_5 = matrix_transpose(R_0_to_3_point_5);

//    matrix_math R_target(3, 3);
//    for (uint8_t i = 0; i < 3; ++i)
//    {
//        for (uint8_t j = 0; j < 3; ++j)
//        {
//            R_target.set_value(i, j, target_matrix.get_value(i, j));
//        }
//    }


//    matrix_math R_6_to_3_point_5 = matrix_multiplication(R_0_to_3_point_5, R_target);
//    
//    posture_inverse_solution(R_6_to_3_point_5);


//}






//matrix_math Arm_Inverse_Kinematics_6DOF::coordinate_inverse_solution(float x_, float y_, float z_)
//{   
//    float target_x = x_;
//    float target_y = y_;
//    float target_z = z_;

//    float r = target_x * target_x + target_y * target_y + target_z * target_z;
//    float z = target_z;

//    float theta_1;
//    float theta_2;
//    float theta_3;
//    if (DH_a[1] == 0)
//    {
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// 计算电机_3的值
//        float A = 2 * DH_a[2] * DH_a[3] - 2 * DH_d[1] * DH_d[2] * sin(DH_alpha[3]) * sin(DH_alpha[2]);
//        float B = 2 * DH_a[2] * DH_d[3] * sin(DH_alpha[3]) + 2 * DH_d[1] * DH_a[3] * sin(DH_alpha[2]);
//        float C = r - (square(DH_a[1]) + square(DH_a[2]) + square(DH_a[3]) + square(DH_d[1]) + square(DH_d[2]) + square(DH_d[3]) + 2 * DH_d[2] * DH_d[3] * cos(DH_alpha[3]) + 2 * DH_d[1] * DH_d[2] * cos(DH_alpha[2]) + 2 * DH_d[1] * DH_d[3] * cos(DH_alpha[2]) * cos(DH_alpha[3]));
//        
//        float u1, u2;
//        // int num_solutions = solve_quadratic_equation(A + C, -2 * B, C - A, &u1, &u2);
//        int num_solutions = trigonometric_function(A, B, C, &u1, &u2);

//        //解出的两个解，分别对应于两个解，取其中之一即可
//        if (num_solutions == 0)
//        {
//            // 无解，可能是目标位置不可达
//            return matrix_math();
//        }


//        float theta_3_1 = 2.0f * atan(u1); // 计算theta_3的值，使用u1作为解
//        float LZ_theta_3_1 = theta_3_1 + PI / 2; // 记录解的数量，调试用
//        float theta_3_2 = 2.0f * atan(u2); // 计算theta_3的值，使用u2作为解
//        float LZ_theta_3_2 = theta_3_2 + PI / 2; // 记录解的数量，调试用

//        if(joint_angles_limit[2].Angle_min > joint_angles_limit[2].Angle_max )
//        {
//            if(LZ_theta_3_1 > joint_angles_limit[2].Angle_min && LZ_theta_3_1 < joint_angles_limit[2].Angle_max + 2 * PI)
//            {
//                target_joint_angles[2] = LZ_theta_3_1;
//                theta_3 = theta_3_1;
//            }
//            else if(LZ_theta_3_2 > joint_angles_limit[2].Angle_min && LZ_theta_3_2 < joint_angles_limit[2].Angle_max + 2 * PI)
//            {
//                target_joint_angles[2] = LZ_theta_3_2;
//                theta_3 = theta_3_2;
//            }
//            else
//            {
//                // 无解，可能是目标位置不可达
//                return matrix_math();
//            }
//        }
//        else
//        {
//            if(LZ_theta_3_1 > joint_angles_limit[2].Angle_min && LZ_theta_3_1 < joint_angles_limit[2].Angle_max)
//            {
//                target_joint_angles[2] = LZ_theta_3_1;
//                theta_3 = theta_3_1;
//            }
//            else if(LZ_theta_3_2 > joint_angles_limit[2].Angle_min && LZ_theta_3_2 < joint_angles_limit[2].Angle_max)
//            {
//                target_joint_angles[2] = LZ_theta_3_2;
//                theta_3 = theta_3_2;
//            }
//            else
//            {
//                // 无解，可能是目标位置不可达
//                return matrix_math();
//            }
//        }

//    
//        w1 = theta_3_1;
//        w2 = LZ_theta_3_1;
//        w3 = theta_3_2;
//        w4 = LZ_theta_3_2;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// 计算电机_2的值
//        float f1 = DH_a[3] * cos(theta_3) + DH_d[3] * sin(theta_3) * sin(DH_alpha[3]) + DH_a[2];
//        float f2 = DH_a[3] * cos(DH_alpha[2]) * sin(theta_3) - DH_d[3] * sin(DH_alpha[3]) * cos(DH_alpha[2]) * cos(theta_3) - DH_d[3] * sin(DH_alpha[2]) * cos(DH_alpha[3]) - DH_d[2] * sin(DH_alpha[2]);
//        float f3 = DH_a[3] * sin(DH_alpha[2]) * sin(theta_3) - DH_d[3] * sin(DH_alpha[3]) * sin(DH_alpha[2]) * cos(theta_3) + DH_d[3] * cos(DH_alpha[2]) * cos(DH_alpha[3]) + DH_d[2] * cos(DH_alpha[2]);
//        float k1 = f1;
//        float k2 = -f2;
//        float k4 = f3 * cos(DH_alpha[1]) - DH_d[1] * cos(DH_alpha[1]);
//        float A2 = -k2;
//        float B2 = k1;
//        float C2 = (z - k4) / sin(DH_alpha[1]);
//        
//        float u3, u4;
//        int num_solutions_2 = trigonometric_function(A2, B2, C2, &u3, &u4);

//        float theta_2_1 = 2.0f * atan(u3);
//        float theta_2_2 = 2.0f * atan(u4);

//        float LZ_theta_2_1 = -theta_2_1;
//        float LZ_theta_2_2 = -theta_2_2;


//        if(joint_angles_limit[1].Angle_min > joint_angles_limit[1].Angle_max )
//        {
//            if(LZ_theta_2_1 > joint_angles_limit[1].Angle_min && LZ_theta_2_1 < joint_angles_limit[1].Angle_max + 2 * PI)
//            {
//                target_joint_angles[1] = LZ_theta_2_1;
//                theta_2 = theta_2_1;
//            }
//            else if(LZ_theta_2_2 > joint_angles_limit[1].Angle_min && LZ_theta_2_2 < joint_angles_limit[1].Angle_max + 2 * PI)
//            {
//                target_joint_angles[1] = LZ_theta_2_2;
//                theta_2 = theta_2_2;
//            }
//            else
//            {
//                // 无解，可能是目标位置不可达
//                return matrix_math();
//            }
//        }
//        else
//        {
//            if(LZ_theta_2_1 > joint_angles_limit[1].Angle_min && LZ_theta_2_1 < joint_angles_limit[1].Angle_max)
//            {
//                target_joint_angles[1] = LZ_theta_2_1;
//                theta_2 = theta_2_1;
//            }
//            else if(LZ_theta_2_2 > joint_angles_limit[1].Angle_min && LZ_theta_2_2 < joint_angles_limit[1].Angle_max)
//            {
//                target_joint_angles[1] = LZ_theta_2_2;
//                theta_2 = theta_2_2;
//            }
//            else
//            {
//                // 无解，可能是目标位置不可达
//                return matrix_math();
//            }
//        }







/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//// 计算电机_1的值
//        float g1 = cos(theta_2_1) * f1 - sin(theta_2_1) * f2 + DH_a[1];
//        float g2 = sin(theta_2_1) * cos(DH_alpha[1]) * f1 + cos(theta_2_1) * cos(DH_alpha[1]) * f2 - sin(DH_alpha[1]) * f3 - DH_d[1] * sin(DH_alpha[1]);
//        float A3 = g1;
//        float B3 = -g2;
//        float C3 = target_x;

//        float u5, u6;
//        int num_solutions_3 = trigonometric_function(A3, B3, C3, &u5, &u6);

//        float theta_1_1 = 2.0f * atan(u5);
//        float theta_1_2 = 2.0f * atan(u6);

//        float LZ_theta_1_1 = -theta_1_1 + 2.8f;
//        float LZ_theta_1_2 = -theta_1_2 + 2.8f;

//        if(joint_angles_limit[0].Angle_min > joint_angles_limit[0].Angle_max )
//        {
//            if(LZ_theta_1_1 > joint_angles_limit[0].Angle_min && LZ_theta_1_1 < joint_angles_limit[0].Angle_max + 2 * PI)
//            {
//                target_joint_angles[0] = LZ_theta_1_1;
//                theta_1 = theta_1_1;
//            }
//            else if(LZ_theta_1_2 > joint_angles_limit[0].Angle_min && LZ_theta_1_2 < joint_angles_limit[0].Angle_max + 2 * PI)
//            {
//                target_joint_angles[0] = LZ_theta_1_2;
//                theta_1 = theta_1_2;
//            }
//            else
//            {
//                // 无解，可能是目标位置不可达
//                return matrix_math();
//            }
//        }
//        else
//        {
//            if(LZ_theta_1_1 > joint_angles_limit[0].Angle_min && LZ_theta_1_1 < joint_angles_limit[0].Angle_max)
//            {
//                target_joint_angles[0] = LZ_theta_1_1;
//                theta_1 = theta_1_1;
//            }
//            else if(LZ_theta_1_2 > joint_angles_limit[0].Angle_min && LZ_theta_1_2 < joint_angles_limit[0].Angle_max)
//            {
//                target_joint_angles[0] = LZ_theta_1_2;
//                theta_1 = theta_1_2;
//            }
//            else
//            {
//                // 无解，可能是目标位置不可达
//                return matrix_math();

//            }
//        }







//    }
//    else if (sin(DH_alpha[1]) == 0)
//    {
//        
//    }
//    else
//    {
//        
//    }



//    matrix_math T_3_point_5_to_0(4, 4);
//    matrix_math T_1_to_0(4, 4);
//    matrix_math T_2_to_1(4, 4);
//    matrix_math T_3_to_2(4, 4);
//    matrix_math T_3_point_5_to_3(4, 4);
//    T_3_point_5_to_0.set_unit_matrix(); // 初始化为单位矩阵

//	T_1_to_0 = DH_Matrix(0, 0, 0, target_joint_angles[0]);
//	T_2_to_1 = DH_Matrix(-PI/2, 0, 80, target_joint_angles[1]);
//	T_3_to_2 = DH_Matrix(0, 370, -80, target_joint_angles[2]);
//	T_3_point_5_to_3 = DH_Matrix(PI/2, 0, 375, 0);
//	// T_3_Matrix = DH_Matrix(pi/2, 0, 375, DH_angle_DM04);



//    T_3_point_5_to_0 = matrix_multiplication( T_3_point_5_to_0 ,T_1_to_0);
//    T_3_point_5_to_0 = matrix_multiplication( T_3_point_5_to_0 ,T_2_to_1);
//    T_3_point_5_to_0 = matrix_multiplication( T_3_point_5_to_0 ,T_3_to_2);
//    T_3_point_5_to_0 = matrix_multiplication( T_3_point_5_to_0 ,T_3_point_5_to_3);




//    return T_3_point_5_to_0;


//    


//}


//void Arm_Inverse_Kinematics_6DOF::posture_inverse_solution(const matrix_math &R_6_to_3_point_5)
//{




//    MotorData_t DM04_data_;
//    MotorData_t DM05_data_;
//    MotorData_t DM06_data_;
//    DM04_data_ = DM04.get_motordata();
//    DM05_data_ = DM05.get_motordata();
//    DM06_data_ = DM06.get_motordata();
//    float r11 = R_6_to_3_point_5.get_value(0, 0);
//    float r12 = R_6_to_3_point_5.get_value(0, 1);
//    float r13 = R_6_to_3_point_5.get_value(0, 2);
//    float r21 = R_6_to_3_point_5.get_value(1, 0);
//    float r22 = R_6_to_3_point_5.get_value(1, 1);
//    float r23 = R_6_to_3_point_5.get_value(1, 2);
//    float r31 = R_6_to_3_point_5.get_value(2, 0);
//    float r32 = R_6_to_3_point_5.get_value(2, 1);
//    float r33 = R_6_to_3_point_5.get_value(2, 2);

//    float joint_angle_4;
//    float joint_angle_5;
//    float joint_angle_6;
//    float reduction_ratio = 0.3f; // 假设减速比为1

//    joint_angle_5 = atan2( sqrt(r31 * r31 + r32 * r32), r33);                            //在解角度中sqrt(r31 * r31 + r32 * r32)取正数是为了解的唯一性，牺牲一部分姿态的平滑性，但在轨迹规划中可以尝试使用负数来增加解的多样性，或者在特定情况下切换正负数以实现更平滑的姿态变化
//    float sin_theta5 = sin(joint_angle_5);
//   
//    if (joint_angle_5 == 0)
//    {
//        joint_angle_4 = DM04_data_.Angle;
//        joint_angle_6 = atan2(-r12, r11) - reduction_ratio * DM04_data_.Angle;
//    }
//    else if (joint_angle_5 == PI)
//    {
//        joint_angle_4 = DM04_data_.Angle;
//        joint_angle_6 = atan2(r12, -r11) - reduction_ratio * DM04_data_.Angle;
//    }
//    else
//    {
//        joint_angle_4 = atan2(r23 / sin_theta5, r13 / sin_theta5);
//        joint_angle_6 = atan2(r32 / sin_theta5, -r31 / sin_theta5);
//    }

//    // joint_angle_4 = atan2(r23 / sin(joint_angle_4), r13 / sin(joint_angle_4));
//    // joint_angle_6 = atan2(r32 / sin(joint_angle_4), -r31 / sin(joint_angle_4));


//    target_joint_angles[3] = joint_angle_4 - 3.87f;
//    target_joint_angles[4] = -joint_angle_5;
//    target_joint_angles[5] = joint_angle_6 ;







//}









//void Arm_Inverse_Kinematics_6DOF::Set_single_limit_angles(float limit_angles_min, float limit_angles_max, uint8_t num_joint)
//{
//    if (num_joint < 1 || num_joint > 6)
//    {
//        // 无效的关节编号，处理错误
//        return;
//    }
//    joint_angles_limit[num_joint - 1].Angle_min = limit_angles_min;
//    joint_angles_limit[num_joint - 1].Angle_max = limit_angles_max;
//}



//void Arm_Inverse_Kinematics_6DOF::Set_all_limit_angles(float limit_angles_min_1, float limit_angles_max_1, 
//                                                       float limit_angles_min_2, float limit_angles_max_2, 
//                                                       float limit_angles_min_3, float limit_angles_max_3, 
//                                                       float limit_angles_min_4, float limit_angles_max_4, 
//                                                       float limit_angles_min_5, float limit_angles_max_5, 
//                                                       float limit_angles_min_6, float limit_angles_max_6)
//{
//    joint_angles_limit[0].Angle_min = limit_angles_min_1;
//    joint_angles_limit[0].Angle_max = limit_angles_max_1;
//    joint_angles_limit[1].Angle_min = limit_angles_min_2;
//    joint_angles_limit[1].Angle_max = limit_angles_max_2;
//    joint_angles_limit[2].Angle_min = limit_angles_min_3;
//    joint_angles_limit[2].Angle_max = limit_angles_max_3;
//    joint_angles_limit[3].Angle_min = limit_angles_min_4;
//    joint_angles_limit[3].Angle_max = limit_angles_max_4;
//    joint_angles_limit[4].Angle_min = limit_angles_min_5;
//    joint_angles_limit[4].Angle_max = limit_angles_max_5;
//    joint_angles_limit[5].Angle_min = limit_angles_min_6;
//    joint_angles_limit[5].Angle_max = limit_angles_max_6;
//}





















