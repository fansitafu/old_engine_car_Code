#include "Transition_type.h"







/**
 * @brief  任意范围线性映射 + 输出钳位（浮点版，输出严格限制在范围内）
 * @param  x        输入值
 * @param  in_min   输入范围最小值
 * @param  in_max   输入范围最大值
 * @param  out_min  输出范围最小值
 * @param  out_max  输出范围最大值
 * @return 映射后的输出值，范围 [out_min, out_max]
 */
float map_linear_clamp(float x, float in_min, float in_max, float out_min, float out_max)
{
    if (in_max == in_min)return out_min;
    float result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    // 钳位到输出边界
    if (result > out_max)result = out_max;
    if (result < out_min) result = out_min;

    return result;
}






/**
 * @brief  将任意整数角度归一化到 [-180, 180] 范围
 * @param  angle  输入任意整数角度（单位°）
 * @return 归一化后角度，范围 [-180, 179]
 */
float angle_normalize_180_float(float angle)
{
    float res = fmodf(angle, 360.0f);
    if (res >= 180.0f) res -= 360.0f;
    else if (res < -180.0f) res += 360.0f;
    return res;
}


/**
 * @brief  等范围角度纯偏移，自动360°周期折叠，输出保持[-180,180)
 * @param  angle   原始角度（单位°）
 * @param  offset  角度偏移量（单位°，正为顺时针偏移）
 * @return 偏移并归一化后的角度
 */
float angle_equal_range_offset(float angle, float offset)
{
    return angle_normalize_180_float(angle + offset);
}







/**
 * @brief  辅助：将任意弧度归一化到 [-π, π) 区间
 */
float rad_norm_pi(float rad)
{
    float res = fmodf(rad, TWO_PI);
    if (res >= PI) res -= TWO_PI;
    else if (res < -PI) res += TWO_PI;
    return res;
}




/**
 * @brief  等区间弧度值纯偏移映射（自动2π周期折叠）
 * @param  rad     原始输入角度（单位：rad）
 * @param  offset  固定偏移量（单位：rad）
 * @return 偏移并归一化后的角度，范围 [-π, π)
 */
float rad_cycle_offset(float rad, float offset)
{
    return rad_norm_pi(rad + offset);
}
