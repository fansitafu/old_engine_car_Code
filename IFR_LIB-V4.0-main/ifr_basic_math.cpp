#include "ifr_basic_math.h"
// 基础数学函数


// 基础滤波器
// 一阶低通滤波器
float low_pass_filter(float input, float output, float alpha) 
{
    return alpha * input + (1 - alpha) * output;
}

// 二阶低通滤波器
float butterworth_filter(ButterworthFilter_t *filter, float input) 
{
    float output = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2 
                  - filter->a1 * filter->y1 - filter->a2 * filter->y2;
    
    // 更新历史值
    filter->x2 = filter->x1;
    filter->x1 = input;
    filter->y2 = filter->y1;
    filter->y1 = output;
    
    return output;
}

