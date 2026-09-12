/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  * FileName 		: ifr_sys_time.cpp
  * Version			: v1.0 
  * Author			: PanJiajun
  * Date			: 2025-10-10
  * Description	:	IFR 库中的 系统计时器 （基于中科大的开源）
  *
  *********************************************************************
  */

/**
 * 要求: 使能并绑定一个32位定时器, 开中断, PSC分频到1MHz, ARR为3600000000 - 1（8个0）
 * 保证arr计数器1us增一次, 1h触发一次中断
 */

#ifndef IFR_SYS_TIME_H
#define IFR_SYS_TIME_H

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "ifr_tim.h"
#ifdef __cplusplus
 extern "C" {
#endif
#ifdef __cplusplus
}
#endif
#ifdef HAL_TIM_MODULE_ENABLED//如果底下是虚的说明你没使用任何定时器
/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Specialized, 系统时间戳
 *
 */
class ifr_Timestamp_ClassDef
{
public:
    // 显式定义构造函数，初始化成员变量
    ifr_Timestamp_ClassDef() : TIM_Overflow_Count(0) {}

    void Init(TIM_HandleTypeDef *htim);

    inline uint64_t Get_Current_Timestamp() const;

    inline float Get_Now_Second() const;

    inline float Get_Now_Millisecond() const;

    inline uint64_t Get_Now_Microsecond() const;

    void Set_Overflow_Count(uint32_t count) { TIM_Overflow_Count = count; }

    uint32_t Get_Overflow_Count() const { return TIM_Overflow_Count; };

protected:
    // 初始化相关常量
    IFR_TIM_ClassDef IFR_TIM_Handler;

    // 定时器溢出计数, 一小时溢出一次
    uint32_t TIM_Overflow_Count; // C++98: 移除类内初始化
    // 
    uint64_t Calculate_Timestamp() const;
};

/**
 * @brief Reusable, 时间对象,
 * 可以为负数，重载了加减乘除、+=、-=、*=、/= 、>=、<=、==、!= 等运算符, 用于计算、比较不同的时间戳对象
 *
 */
class ifr_time_calculate_ClassDef
{
public:
    // 构造函数 (C++98: 所有成员在初始化列表中初始化)
    ifr_time_calculate_ClassDef() : Microsecond(0) {}

    explicit ifr_time_calculate_ClassDef(const int64_t &__Microsecond) : Microsecond(__Microsecond) {}

    ifr_time_calculate_ClassDef(const int32_t &__Second, const int16_t &__Millisecond, const int16_t &__Microsecond)
        : Microsecond((int64_t)(__Second) * 1000000LL + (int64_t)(__Millisecond) * 1000LL + (int64_t)(__Microsecond)) {}

    // 复制构造函数 (C++98: 保留，移除移动构造)
    ifr_time_calculate_ClassDef(const ifr_time_calculate_ClassDef &Time) : Microsecond(Time.Microsecond) {}

    // 析构函数 
    ~ifr_time_calculate_ClassDef() {};

    // 复制赋值函数
    ifr_time_calculate_ClassDef &operator=(const ifr_time_calculate_ClassDef &Time)
    {
        if (this != &Time)
        {
            Microsecond = Time.Microsecond;
        }
        return (*this);
    }

    inline ifr_time_calculate_ClassDef operator+(const ifr_time_calculate_ClassDef &Time) const
    {
        return (ifr_time_calculate_ClassDef(Microsecond + Time.Get_Microsecond()));
    }

    inline ifr_time_calculate_ClassDef operator-(const ifr_time_calculate_ClassDef &Time) const
    {
        return (ifr_time_calculate_ClassDef(Microsecond - Time.Get_Microsecond()));
    }

    inline ifr_time_calculate_ClassDef operator*(const float &value) const
    {
        return (ifr_time_calculate_ClassDef((int64_t)((float) (Microsecond) * value)));
    }

    inline friend ifr_time_calculate_ClassDef operator*(const float &value, const ifr_time_calculate_ClassDef &Time)
    {
        return (ifr_time_calculate_ClassDef((int64_t)((float) (Time.Get_Microsecond()) * value)));
    }

    inline ifr_time_calculate_ClassDef operator/(const float &value) const
    {
        return (ifr_time_calculate_ClassDef((int64_t)((float) (Microsecond) / value)));
    }

    inline ifr_time_calculate_ClassDef &operator+=(const ifr_time_calculate_ClassDef &Time)
    {
        Microsecond += Time.Get_Microsecond();
        return (*this);
    }

    inline ifr_time_calculate_ClassDef &operator-=(const ifr_time_calculate_ClassDef &Time)
    {
        Microsecond -= Time.Get_Microsecond();
        return (*this);
    }

    inline ifr_time_calculate_ClassDef &operator*=(const float &value)
    {
        Microsecond = (int64_t)((float) (Microsecond) * value);
        return (*this);
    }

    inline ifr_time_calculate_ClassDef &operator/=(const float &value)
    {
        Microsecond = (int64_t)((float) (Microsecond) / value);
        return (*this);
    }

    inline bool operator>(const ifr_time_calculate_ClassDef &Time) const
    {
        return (Microsecond > Time.Get_Microsecond());
    }

    inline bool operator<(const ifr_time_calculate_ClassDef &Time) const
    {
        return (Microsecond < Time.Get_Microsecond());
    }

    inline bool operator>=(const ifr_time_calculate_ClassDef &Time) const
    {
        return (Microsecond >= Time.Get_Microsecond());
    }

    inline bool operator<=(const ifr_time_calculate_ClassDef &Time) const
    {
        return (Microsecond <= Time.Get_Microsecond());
    }

    inline bool operator==(const ifr_time_calculate_ClassDef &Time) const
    {
        return (Microsecond == Time.Get_Microsecond());
    }

    inline bool operator!=(const ifr_time_calculate_ClassDef &Time) const
    {
        return (Microsecond != Time.Get_Microsecond());
    }

    inline float Get_Second() const;

    inline float Get_Millisecond() const;

    inline int64_t Get_Microsecond() const;

protected:
    // 微秒
    int64_t Microsecond; 
};

/* Exported variables --------------------------------------------------------*/

extern ifr_Timestamp_ClassDef ifr_sys_timestamp;


void Delay_Second(const uint32_t &Second);

void Delay_Millisecond(const uint32_t &Millisecond);

void Delay_Microsecond(const uint32_t &Microsecond);


/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获取当前时间
 *
 * @return uint64_t 当前时间
 */
inline uint64_t ifr_Timestamp_ClassDef::Get_Current_Timestamp() const
{
    return (Calculate_Timestamp());
}

/**
 * @brief 获取当前时间, 单位秒
 *
 * @return float 当前时间, 单位秒
 */
inline float ifr_Timestamp_ClassDef::Get_Now_Second() const
{
    return ((float) (Calculate_Timestamp()) / 1000000.0f);
}

/**
 * @brief 获取当前时间, 单位毫秒
 *
 * @return float 当前时间, 单位毫秒
 */
inline float ifr_Timestamp_ClassDef::Get_Now_Millisecond() const
{
    return ((float) (Calculate_Timestamp()) / 1000.0f);
}

/**
 * @brief 获取当前时间, 单位微秒
 *
 * @return uint64_t 当前时间, 单位微秒
 */
inline uint64_t ifr_Timestamp_ClassDef::Get_Now_Microsecond() const
{
    return (Calculate_Timestamp());
}

/**
 * @brief 获取时间, 单位秒
 *
 * @return float 时间, 单位秒
 */
inline float ifr_time_calculate_ClassDef::Get_Second() const
{
    return ((float) Microsecond / 1000000.0f);
}

/**
 * @brief 获取时间, 单位毫秒
 *
 * @return float 时间, 单位毫秒
 */
inline float ifr_time_calculate_ClassDef::Get_Millisecond() const
{
    return ((float) Microsecond / 1000.0f);
}

/**
 * @brief 获取时间, 单位微秒
 *
 * @return int64_t 时间, 单位微秒
 */
inline int64_t ifr_time_calculate_ClassDef::Get_Microsecond() const
{
    return (Microsecond);
}
#endif // HAL_TIM_MODULE_ENABLED
#endif // IFR_SYS_TIME_H
