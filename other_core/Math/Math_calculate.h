#ifndef MATH_CALCULATE_H
#define MATH_CALCULATE_H


#include <math.h>



// 数学常量：避免重复写PI，弧度/角度转换
#define PI 3.1415926535
#define RAD2DEG 180.0/PI  // 弧度→角度（看结果用）
#define DEG2RAD PI/180.0  // 角度→弧度（计算用）





//一元二次方程求解函数
int solve_quadratic_equation(float a, float b, float c, float *x1, float *x2);

float square(float x);





























#endif // MATH_CALCULATE_H