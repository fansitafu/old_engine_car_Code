#include "Arm_calculate_basis.h"



Arm_calculate_basis :: Arm_calculate_basis(): Matrix_T0(4,4), Matrix_T1(4,4), Matrix_T2(4,4), Matrix_T3(4,4), Matrix_T4(4,4), Matrix_T5(4,4)
{

    Matrix_arr[0] = Matrix_T0;
    Matrix_arr[1] = Matrix_T1;
    Matrix_arr[2] = Matrix_T2;
    Matrix_arr[3] = Matrix_T3;
    Matrix_arr[4] = Matrix_T4;
    Matrix_arr[5] = Matrix_T5;
    Matrix_num = 6;




}

void Arm_calculate_basis::Update_DH(float T0_a_alpha, float T0_a, float T0_d, float T0_O_theta, 
                                  float T1_a_alpha, float T1_a, float T1_d, float T1_O_theta, 
                                  float T2_a_alpha, float T2_a, float T2_d, float T2_O_theta, 
                                  float T3_a_alpha, float T3_a, float T3_d, float T3_O_theta, 
                                  float T4_a_alpha, float T4_a, float T4_d, float T4_O_theta, 
                                  float T5_a_alpha, float T5_a, float T5_d, float T5_O_theta)                               
{   

    DH_alpha[0] = T0_a_alpha;
    DH_alpha[1] = T1_a_alpha;
    DH_alpha[2] = T2_a_alpha;
    DH_alpha[3] = T3_a_alpha;
    DH_alpha[4] = T4_a_alpha;
    DH_alpha[5] = T5_a_alpha;

    DH_a[0] = T0_a;
    DH_a[1] = T1_a;
    DH_a[2] = T2_a;
    DH_a[3] = T3_a;
    DH_a[4] = T4_a;
    DH_a[5] = T5_a;

    DH_d[0] = T0_d; 
    DH_d[1] = T1_d;
    DH_d[2] = T2_d;
    DH_d[3] = T3_d;
    DH_d[4] = T4_d;
    DH_d[5] = T5_d;

    DH_O_theta[0] = T0_O_theta;
    DH_O_theta[1] = T1_O_theta;
    DH_O_theta[2] = T2_O_theta;
    DH_O_theta[3] = T3_O_theta;
    DH_O_theta[4] = T4_O_theta;
    DH_O_theta[5] = T5_O_theta;

    Matrix_T0 = DH_Matrix(T0_a_alpha, T0_a, T0_d, T0_O_theta);
    Matrix_T1 = DH_Matrix(T1_a_alpha, T1_a, T1_d, T1_O_theta);
    Matrix_T2 = DH_Matrix(T2_a_alpha, T2_a, T2_d, T2_O_theta);
    Matrix_T3 = DH_Matrix(T3_a_alpha, T3_a, T3_d, T3_O_theta);
    Matrix_T4 = DH_Matrix(T4_a_alpha, T4_a, T4_d, T4_O_theta);
    Matrix_T5 = DH_Matrix(T5_a_alpha, T5_a, T5_d, T5_O_theta);


    Matrix_arr[0] = Matrix_T0;
    Matrix_arr[1] = Matrix_T1;
    Matrix_arr[2] = Matrix_T2;
    Matrix_arr[3] = Matrix_T3;
    Matrix_arr[4] = Matrix_T4;
    Matrix_arr[5] = Matrix_T5;

}

void Arm_calculate_basis::Init_DH(matrix_math T0, matrix_math T1, matrix_math T2, matrix_math T3, matrix_math T4, matrix_math T5)
{
    Matrix_T0 = T0;
    Matrix_T1 = T1;
    Matrix_T2 = T2;
    Matrix_T3 = T3;
    Matrix_T4 = T4;
    Matrix_T5 = T5;
}



matrix_math Arm_calculate_basis::get_T_matrix(uint8_t num)
{
    if (num < Matrix_num) 
    {
        return Matrix_arr[num];
    }
    else 
    {
        // 如果请求的矩阵编号超过范围，返回一个默认的1x1矩阵
        return matrix_math(1, 1); 
    } 
}



matrix_math Arm_calculate_basis::arm_forward_kinematics(uint8_t num_end_T)
{
    matrix_math T_result(4, 4);
    T_result.set_unit_matrix(); // 初始化为单位矩阵

    if (num_end_T < Matrix_num)
    {
        for (int i = 0; i < num_end_T; i++)
        {

            T_result = matrix_multiplication( T_result , Matrix_arr[i] );
        }

        return T_result;
    }
    else
    {
        // 如果请求的矩阵编号超过范围，返回一个默认的1x1矩阵
        return matrix_math(1, 1);
    }


}



















