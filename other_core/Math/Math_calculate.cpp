#include "Math_calculate.h"

int solve_quadratic_equation(float a, float b, float c, float *x1, float *x2)
{
    float discriminant = b * b - 4 * a * c; // 计算判别式

    if (discriminant < 0)
    {
        // 无实数解
        return 0;
    }
    else if (discriminant == 0)
    {
        // 有一个实数解
        *x1 = -b / (2 * a);
        return 1;
    }
    else
    {
        // 有两个实数解
        float sqrt_discriminant = sqrt(discriminant);
        *x1 = (-b + sqrt_discriminant) / (2 * a);
        *x2 = (-b - sqrt_discriminant) / (2 * a);
        return 2;
    }
}





float square(float x)
{
    return x * x;
}




