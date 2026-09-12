#ifndef ARM_INVERSE_KINEMATICS_H
#define ARM_INVERSE_KINEMATICS_H


#include "matrix_math.h"
#include "Arm_calculate_basis.h"
#include "Math_calculate.h"

extern IFR_DM_Pos_Motor DM04, DM05, DM06;

typedef struct
{
    float Angle_min;
    float Angle_max;
} Angle_limit_t;


class Arm_Inverse_Kinematics_6DOF : public Arm_calculate_basis
{

public:

    Arm_Inverse_Kinematics_6DOF():target_matrix(4, 4){};
    ~Arm_Inverse_Kinematics_6DOF(){};



    void calculate_inverse_kinematics(const matrix_math &target_matrix);

    matrix_math coordinate_inverse_solution(float x_, float y_, float z_);

    void posture_inverse_solution(const matrix_math &R_6_to_3_point_5);

    int trigonometric_function(float A_, float B_, float C_, float *u1, float *u2);///A*sin +B*cos =C的形式，计算角度值

    void Set_single_limit_angles(float limit_angles_min, float limit_angles_max, uint8_t num_joint);

    void Set_all_limit_angles(float limit_angles_min_1, float limit_angles_max_1, 
                              float limit_angles_min_2, float limit_angles_max_2, 
                              float limit_angles_min_3, float limit_angles_max_3, 
                              float limit_angles_min_4, float limit_angles_max_4, 
                              float limit_angles_min_5, float limit_angles_max_5, 
                              float limit_angles_min_6, float limit_angles_max_6);

    // float target_x;
    // float target_y;
    // float target_z;


    matrix_math target_matrix;

    // matrix_math target_posture_matrix;

    float target_joint_angles[6];
    Angle_limit_t joint_angles_limit[6];


};


















































#endif // ARM_INVERSE_KINEMATICS_H