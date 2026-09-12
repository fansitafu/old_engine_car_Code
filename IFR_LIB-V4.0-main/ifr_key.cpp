/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_key.cpp
  * Version		: v1.0
  * Author		: panjiajun
  * Date			: 2025-11-10
  * Description	:	IFR按键库
  *********************************************************************
  */
/*******************************************************************************
* @功能     		: 按键处理类实现
* @参数         : None
* @返回值 				: void
* @概述  				: 实现按键状态检测、防抖和事件触发, 注意：同一IFR_Key对象只能使用update（回调模式）或set_one_num（直接赋值模式）中的一种，不可同时使用。
*******************************************************************************/
#include "ifr_key.h"
/*******************************************************************************
* @功能     		: 构造函数实现
* @参数         : None
* @返回值 				: void
* @概述  				: 初始化按键状态机成员变量
*******************************************************************************/
IFR_Key::IFR_Key(void)
{
    m_currentstate = 0;
    last_state = 0;
    stable_state = 0;
    debounce_counter = 0;
    debounce_threshold = 3;  		// 默认防抖阈值，连续3次相同状态确认
    press_start_time = 0;
    long_press_threshold = 0;  	// 默认长按时间0ms
    long_press_triggered = 0;
    key_active_state = KEY_ENABLE;  // 默认使能
    long_press_state = ONCE;        // 初始化长按类型为"单次触发"
    // 初始化回调函数指针为NULL
    rising_cb = NULL;
    falling_cb = NULL;
    edge_cb = NULL;
    long_press_cb = NULL;
}
/*******************************************************************************
* @功能     		: 获取当前系统时间
* @参数         : None
* @返回值 				: uint32_t - 当前系统时间（毫秒）
* @概述  				: 调用HAL库获取系统时间
*******************************************************************************/
uint32_t IFR_Key::get_current_time(void)
{
    return HAL_GetTick();
}
/*******************************************************************************
* @功能     	: 注册上升沿回调函数
* @参数1      : func - 回调函数指针
* @返回值 			: void
* @概述  			: 设置上升沿触发时调用的函数
*******************************************************************************/
void IFR_Key::register_rising(risingFunc func)
{
    rising_cb = func;
}
/*******************************************************************************
* @功能     	: 注册下降沿回调函数
* @参数1      : func - 回调函数指针
* @返回值 			: void
* @概述  			: 设置下降沿触发时调用的函数
*******************************************************************************/
void IFR_Key::register_falling(fallingFunc func)
{
    falling_cb = func;
}
/*******************************************************************************
* @功能     	: 注册双边沿回调函数
* @参数1      : func - 回调函数指针
* @返回值 			: void
* @概述  			: 设置双边沿触发时调用的函数
*******************************************************************************/
void IFR_Key::register_both_edges(rising_or_fallingFunc func)
{
    edge_cb = func;
}
/*******************************************************************************
* @功能     	: 注册长按回调函数
* @参数1      : func - 回调函数指针
* @参数2      : is_once - 长按触发模式（单次/连续）
* @参数3      : time - 长按时间阈值（毫秒）
* @返回值 			: void
* @概述  			: 设置长按触发时调用的函数及相关参数
*******************************************************************************/
void IFR_Key::register_long_press(long_pressFunc func, Long_press_state is_once, uint32_t time)
{
    long_press_cb = func;
    long_press_state = is_once;
    long_press_threshold = time;
}
/*******************************************************************************
* @功能     	: 设置防抖阈值
* @参数1      : threshold - 防抖阈值（连续检测次数）
* @返回值 			: void
* @概述  			: 设置按键状态变化所需的连续相同状态次数
*******************************************************************************/
void IFR_Key::set_debounce_threshold(uint8_t threshold)
{
    debounce_threshold = threshold;
}
/*******************************************************************************
* @功能     	: 设置长按参数
* @参数1      : type - 长按触发模式（单次/连续）
* @参数2      : threshold - 长按时间阈值（毫秒）
* @返回值 			: void
* @概述  			: 设置长按触发的模式和时间阈值
*******************************************************************************/
void IFR_Key::set_long_press_param(Long_press_state type, uint32_t threshold)
{
    long_press_state = type;        // 设置长按类型（ONCE/CONTINUE）
    long_press_threshold = threshold; // 设置长按阈值（ms）
}
/*******************************************************************************
* @功能     	: 更新按键状态
* @参数1      : new_state - 新的按键状态
* @返回值 			: void
* @概述  			: 处理按键状态更新，包括防抖和事件触发 注意：同一IFR_Key对象只能使用update（回调模式）或set_one_num（直接赋值模式）中的一种，不可同时使用。
*******************************************************************************/
void IFR_Key::update(uint8_t new_state)
{
    // 如果按键被禁用，直接返回
    if (key_active_state == KEY_DISABLE)
        return;
    last_state = m_currentstate;
    m_currentstate = new_state;
    // 防抖处理逻辑
    if (m_currentstate == stable_state)
    {
        // 当前状态与稳定状态一致，重置计数器
        debounce_counter = 0;
    }
    else
    {
        // 当前状态与稳定状态不一致，增加计数器
        debounce_counter++;
        // 连续多次检测到相同状态，确认状态变化
        if (debounce_counter >= debounce_threshold)
        {
            uint8_t previous_stable = stable_state;
            stable_state = m_currentstate;
            debounce_counter = 0;
            // 检测上升沿 (0 -> 1)
            if (previous_stable == 0 && stable_state == 1)
            {
                if (rising_cb != NULL)
                {
                    rising_cb(this, stable_state);
                }
                if (edge_cb != NULL)
                {
                    edge_cb(this, stable_state);
                }
                // 记录按下开始时间
                press_start_time = get_current_time();
                long_press_triggered = 0;
            }
            // 检测下降沿 (1 -> 0)
            else if (previous_stable == 1 && stable_state == 0)
            {
                if (falling_cb != NULL)
                {
                    falling_cb(this, stable_state);
                }
                if (edge_cb != NULL)
                {
                    edge_cb(this, stable_state);
                }
            }
        }
    }
    // 处理长按事件
    if (stable_state == 1 && long_press_cb != NULL) // 按键处于按下状态且有长按事件
    {
        uint32_t current_time = get_current_time();
        // 检测长按单次(按键处于按下状态且长按事件未触发)
        if (long_press_state == ONCE && !long_press_triggered)
        {
            // 检查是否超过长按阈值
            if (current_time - press_start_time >= long_press_threshold)
            {
                long_press_cb(this, stable_state);
                long_press_triggered = 1; // 标记为已触发，避免重复调用
            }
        }
        else if (long_press_state == CONTINUE)
        {
            // 检查是否超过长按阈值
            if (current_time - press_start_time >= long_press_threshold)
            {
                long_press_cb(this, stable_state);
            }
        }
    }
}
/*******************************************************************************
* @功能     	: 设置一个数值（根据按键状态）
* @参数1      : new_state - 新的按键状态
* @参数2      : key_state - 触发状态（上升沿/下降沿/长按等）
* @参数3      : need_set_num - 需要设置的数值指针
* @参数4      : set_num - 要设置的数值
* @返回值 			: void
* @概述  			: 根据按键状态设置指定变量的值 注意：同一IFR_Key对象只能使用update（回调模式）或set_one_num（直接赋值模式）中的一种，不可同时使用。
*******************************************************************************/
void IFR_Key::set_one_num(uint8_t new_state, Key_state key_state, float *need_set_num, float set_num)
{
    // 如果按键被禁用或指针为空，直接返回
    if (key_active_state == KEY_DISABLE || need_set_num == NULL)
        return;
    last_state = m_currentstate;
    m_currentstate = new_state;
    // 防抖处理逻辑
    if (m_currentstate == stable_state)
    {
        // 当前状态与稳定状态一致，重置计数器
        debounce_counter = 0;
    }
    else
    {
        // 当前状态与稳定状态不一致，增加计数器
        debounce_counter++;
        // 连续多次检测到相同状态，确认状态变化
        if (debounce_counter >= debounce_threshold)
        {
            uint8_t previous_stable = stable_state;
            stable_state = m_currentstate;
            debounce_counter = 0;
            // 检测上升沿 (0 -> 1)
            if (previous_stable == 0 && stable_state == 1)
            {
                if (key_state == RISING || key_state == RISING_OR_FALLING)
                {
                    *need_set_num = set_num;
                }
                // 记录按下开始时间
                press_start_time = get_current_time();
                long_press_triggered = 0;
            }
            // 检测下降沿 (1 -> 0)
            else if (previous_stable == 1 && stable_state == 0)
            {
                if (key_state == FALLING || key_state == RISING_OR_FALLING)
                {
                    *need_set_num = set_num;
                }
            }
        }
    }
    // 处理长按事件
    if (stable_state == 1 && key_state == LONG_PRESS) // 按键处于按下状态且有长按事件
    {
        uint32_t current_time = get_current_time();
        // 检测长按单次(按键处于按下状态且长按事件未触发)
        if (long_press_state == ONCE && !long_press_triggered)
        {
            // 检查是否超过长按阈值
            if (current_time - press_start_time >= long_press_threshold)
            {
                *need_set_num = set_num;
                long_press_triggered = 1; // 标记为已触发，避免重复调用
            }
        }
        else if (long_press_state == CONTINUE)
        {
            // 检查是否超过长按阈值
            if (current_time - press_start_time >= long_press_threshold)
            {
                *need_set_num = set_num;
            }
        }
    }
}
/*******************************************************************************
* @功能     		: 设置按键使能状态
* @参数1      : is_enable - 使能或禁用
* @返回值 		: void
* @概述  			: 启用或禁用按键处理
*******************************************************************************/
void IFR_Key::Key_state_set(Key_active_state is_enable)
{
    key_active_state = is_enable;
}
/*******************************************************************************
* @功能     		: 获取当前原始状态
* @参数         : None
* @返回值 				: uint8_t - 当前按键状态
* @概述  				: 返回未经防抖处理的当前按键状态
*******************************************************************************/
uint8_t IFR_Key::get_current_state(void)
{
    return m_currentstate;
}
/*******************************************************************************
* @功能     		: 获取稳定状态
* @参数         : None
* @返回值 				: uint8_t - 经过防抖的稳定状态
* @概述  				: 返回经过防抖处理后的稳定按键状态
*******************************************************************************/
uint8_t IFR_Key::get_stable_state(void)
{
    return stable_state;
}
