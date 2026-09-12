#ifndef IFR_BASIC_MATH_H
#define IFR_BASIC_MATH_H
/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_basic_math.cpp
  * Version			: v1.0
  * Author			: PanJiajun 
  * Date				: 2026-2-24
  * Description	:	IFR 基础数学工具库
	*
  *********************************************************************
  */
#ifdef __cplusplus
 extern "C" {
#endif
#ifdef __cplusplus
 }
#endif
#include "main.h"
#include <math.h>
typedef struct {
    float x1, x2;  // 输入历史
    float y1, y2;  // 输出历史
    float a1, a2;  // 系数
    float b0, b1, b2; // 系数
} ButterworthFilter_t;  // 二阶低通滤波器结构体

#define IFR_M_PI          3.14159265358979323846f  // 圆周率
#define IFR_M_TWO_PI      6.28318530717958647692f  // 2π
#define IFR_M_HALF_PI     1.57079632679489661923f  // π/2
#define IFR_M_SQRT2       1.41421356237309504880f  // √2
#define IFR_M_SQRT1_2     0.70710678118654752440f  // 1/√2
#define IFR_M_E           2.71828182845904523536f  // 自然对数的底
#define IFR_M_LN2         0.69314718055994530942f  // ln(2)
#define IFR_M_LN10        2.30258509299404568402f  // ln(10)

#endif // IFR_BASIC_MATH_H
