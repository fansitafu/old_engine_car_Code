#ifndef CAR_ARM_CONTROL_H
#define CAR_ARM_CONTROL_H





#define pi 3.1415926f

#include "arm_inverse_kinematics.h"
#include "ifr_lib.h"
#include "usart.h"
#include "tim.h"
#include "fdcan.h"
#include "matrix_math.h"
#include "Init_car_arm.h"


void Arm_matrix_calculate();



extern Arm_Inverse_Kinematics_6DOF arm_inverse_6dof;



#endif // CAR_ARM_CONTROL_H