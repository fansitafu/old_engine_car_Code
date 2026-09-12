#ifndef ARM_CALCULATE_BASIS_H
#define ARM_CALCULATE_BASIS_H


#include "matrix_math.h"



class Arm_calculate_basis
{   
    
    
public:
    Arm_calculate_basis();
    ~Arm_calculate_basis(){};

    void Update_DH(float T0_a_alpha, float T0_a, float T0_d, float T0_O_theta, 
                   float T1_a_alpha, float T1_a, float T1_d, float T1_O_theta, 
                   float T2_a_alpha, float T2_a, float T2_d, float T2_O_theta, 
                   float T3_a_alpha, float T3_a, float T3_d, float T3_O_theta, 
                   float T4_a_alpha, float T4_a, float T4_d, float T4_O_theta, 
                   float T5_a_alpha, float T5_a, float T5_d, float T5_O_theta);

    void Init_DH(matrix_math T0, matrix_math T1, matrix_math T2, matrix_math T3, matrix_math T4, matrix_math T5);

    matrix_math get_T_matrix(uint8_t num);

    matrix_math arm_forward_kinematics(uint8_t num_end_T);



protected:

// 机械臂的关节坐标转换参数矩阵
    matrix_math Matrix_T0;
    matrix_math Matrix_T1;
    matrix_math Matrix_T2;
    matrix_math Matrix_T3;
    matrix_math Matrix_T4;
    matrix_math Matrix_T5;
    uint8_t Matrix_num;
    matrix_math Matrix_arr[6];

    // float DH_0[4];
    // float DH_1[4];
    // float DH_2[4];
    // float DH_3[4];
    // float DH_4[4];
    // float DH_5[4];
    float DH_alpha[6];
    float DH_a[6];
    float DH_d[6];
    float DH_O_theta[6];


};












































#endif // ARM_CALCULATE_BASIS_H