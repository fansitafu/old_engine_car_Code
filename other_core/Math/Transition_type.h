#ifndef TRANSITION_TYPE_H
#define TRANSITION_TYPE_H




#include <stdint.h>
#include <math.h>

#define PI       3.1415926f
#define TWO_PI   (2.0f * PI)
#define DEGREE_TO_RADIAN(arg) ((arg) * PI / 180.0f)
#define RADIAN_TO_DEGREE(arg) ((arg) * 180.0f / PI)

float map_linear_clamp(float x, float in_min, float in_max, float out_min, float out_max);


float angle_normalize_180_float(float angle);
float angle_equal_range_offset(float angle, float offset);

float rad_norm_pi(float rad);
float rad_cycle_offset(float rad, float offset);































#endif // TRANSITION_TYPE_H




























