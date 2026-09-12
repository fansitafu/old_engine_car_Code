/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_key.h
  * Version		: v1.0
  * Author		: panjiajun
  * Date			: 2025-11-10
  * Description	:	IFR按键库
  *********************************************************************
  */
/*******************************************************************************
* @功能     		: 按键处理类声明
* @参数         : None
* @返回值 				: void
* @概述  				: 定义按键状态检测、防抖和事件触发接口 注意：同一IFR_Key对象只能使用update（回调模式）或set_one_num（直接赋值模式）中的一种，不可同时使用。
*******************************************************************************/
#ifndef IFR_KEY_H
#define IFR_KEY_H
#include "main.h"
// 前向声明IFR_Key类
class IFR_Key;
/*******************************************************************************
* @功能     		: 按键触发状态枚举
* @参数         : None
* @返回值 				: void
* @概述  				: 定义按键触发的不同状态
*******************************************************************************/
typedef enum
{
	RISING,             // 上升沿触发
	FALLING,            // 下降沿触发
	RISING_OR_FALLING,  // 双边沿触发
	LONG_PRESS,         // 长按触发
} Key_state;
/*******************************************************************************
* @功能     		: 按键使能状态枚举
* @参数         : None
* @返回值 				: void
* @概述  				: 定义按键是否使能
*******************************************************************************/
typedef enum
{
  KEY_ENABLE,         // 按键使能
  KEY_DISABLE,        // 按键禁用
} Key_active_state;
/*******************************************************************************
* @功能     		: 长按触发模式枚举
* @参数         : None
* @返回值 				: void
* @概述  				: 定义长按触发的模式
*******************************************************************************/
typedef enum
{
	ONCE,               // 长按单次触发
	CONTINUE,           // 长按连续触发
} Long_press_state;
/*******************************************************************************
* @功能     		: 上升沿回调函数类型
* @参数         : None
* @返回值 				: void
* @概述  				: 定义上升沿回调函数类型
*******************************************************************************/
typedef void (*risingFunc) (IFR_Key* fsm, uint8_t State);
/*******************************************************************************
* @功能     		: 下降沿回调函数类型
* @参数         : None
* @返回值 				: void
* @概述  				: 定义下降沿回调函数类型
*******************************************************************************/
typedef void (*fallingFunc) (IFR_Key* fsm, uint8_t State);
/*******************************************************************************
* @功能     		: 双边沿回调函数类型
* @参数         : None
* @返回值 				: void
* @概述  				: 定义双边沿回调函数类型
*******************************************************************************/
typedef void (*rising_or_fallingFunc) (IFR_Key* fsm, uint8_t State);
/*******************************************************************************
* @功能     		: 长按回调函数类型
* @参数         : None
* @返回值 				: void
* @概述  				: 定义长按回调函数类型
*******************************************************************************/
typedef void (*long_pressFunc) (IFR_Key* fsm, uint8_t State);
/*******************************************************************************
* @功能     		: 按键处理类
* @参数         : None
* @返回值 				: void
* @概述  				: 实现按键状态检测、防抖和事件触发 注意：同一IFR_Key对象只能使用update（回调模式）或set_one_num（直接赋值模式）中的一种，不可同时使用。
*******************************************************************************/
class IFR_Key {
private:
    /**
      * @概述	当前按键状态（原始状态）
      */
    uint8_t m_currentstate;
    /**
      * @概述	上一次按键状态
      */
    uint8_t last_state;
    /**
      * @概述	经过防抖的稳定状态
      */
    uint8_t stable_state;
    /**
      * @概述	防抖计数器
      */
    uint8_t debounce_counter;
    /**
      * @概述	防抖阈值（连续检测次数）
      */
    uint8_t debounce_threshold;
    /**
      * @概述	按键按下开始时间（毫秒）
      */
    uint32_t press_start_time;
    /**
      * @概述	长按判断阈值（毫秒）
      */
    uint32_t long_press_threshold;
    /**
      * @概述	长按事件是否已触发标志
      */
    uint8_t long_press_triggered;
    /**
      * @概述	长按触发模式（单次/连续）
      */
    Long_press_state long_press_state;
    /**
      * @概述	按键使能状态
      */
    Key_active_state key_active_state;
    /**
      * @概述	上升沿回调函数指针
      */
    risingFunc rising_cb;
    /**
      * @概述	下降沿回调函数指针
      */
    fallingFunc falling_cb;
    /**
      * @概述	双边沿回调函数指针
      */
    rising_or_fallingFunc edge_cb;
    /**
      * @概述	长按回调函数指针
      */
    long_pressFunc long_press_cb;
    /**
      * @概述	获取当前系统时间（毫秒）
      * @返回值	当前系统时间
      */
    uint32_t get_current_time(void);
public:
		//构造函数
    IFR_Key(void);
		// 注册回调函数
    void register_rising(risingFunc func);
    void register_falling(fallingFunc func);
    void register_both_edges(rising_or_fallingFunc func);
    void register_long_press(long_pressFunc func, Long_press_state is_once, uint32_t time);
		// 调用事件或设置值函数
    void set_one_num(uint8_t new_state, Key_state key_state, float *need_set_num, float set_num);
    void update(uint8_t new_state);
    void set_debounce_threshold(uint8_t threshold);
    void Key_state_set(Key_active_state is_enable);
    uint8_t get_current_state(void);
    void set_long_press_param(Long_press_state type, uint32_t threshold);
    uint8_t get_stable_state(void);
};
#endif // IFR_KEY_H
