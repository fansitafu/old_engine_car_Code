/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_state_machine.cpp
  * Version		: v1.0
  * Author		: panjiajun
  * Date			: 2025-11-10
  * Description	:	IFR状态机库
  *********************************************************************
  */
#include "ifr_state_machine.h"
/*******************************************************************************
* @功能     	: 查找状态配置
* @参数1      : state - 要查找的状态标识
* @返回值 			: const StateConfig* - 找到的配置项指针
* @概述  			: 在已注册状态中查找指定标识的配置
*******************************************************************************/
const StateConfig* StateMachine::findConfig(uint8_t state) const {
    // 遍历所有已注册状态
    for (uint8_t i = 0; i < m_stateCount; ++i) 
		{
        // 匹配状态标识
        if (m_states[i].state == state) 
				{
            return &m_states[i];
        }
    }
    return NULL; // 未找到匹配项
}
/*******************************************************************************
* @功能     		: 构造函数实现
* @参数1      : maxStates - 最大支持状态数
* @参数2      : stateBuffer - 预分配的状态配置数组
* @返回值 		: void
* @概述  			: 初始化状态机成员变量
*******************************************************************************/
StateMachine::StateMachine(uint8_t maxStates, StateConfig* stateBuffer) 
    : m_currentState(0),        // 初始当前状态为0
      m_prevState(0),           // 初始上一个状态为0
      m_stateEnterTime(0),      // 初始进入时间为0
      m_currentTime(0),         // 初始当前时间为0
      m_states(stateBuffer),    // 指向外部提供的配置数组
      m_maxStates(maxStates),   // 初始化最大状态数
      m_stateCount(0) // 初始状态数为0
{     
}
/*******************************************************************************
* @功能     		: 析构函数实现
* @参数         : None
* @返回值 				: void
* @概述  				: 清理状态机资源
*******************************************************************************/
StateMachine::~StateMachine() 
{
    // 无动态资源需要释放
}
/*******************************************************************************
* @功能     		: 注册状态配置
* @参数1      : config - 要注册的状态配置
* @返回值 		: bool - 注册成功返回true
* @概述  			: 将新状态添加到状态机
*******************************************************************************/
bool StateMachine::registerState(const StateConfig& config) 
{
    // 检查状态数组是否已满
    if (m_stateCount >= m_maxStates) return false;
    // 检查状态是否已存在
    if (findConfig(config.state) != NULL) return false;
    // 添加新状态配置到数组
    m_states[m_stateCount] = config;
    m_stateCount++;
    return true;
}
/*******************************************************************************
* @功能     		: 状态切换
* @参数1      : newState - 目标状态标识
* @返回值 		: void
* @概述  			: 执行状态切换流程
*******************************************************************************/
void StateMachine::setState(uint8_t newState) 
{
    // 验证目标状态是否有效且不是当前状态
    if (findConfig(newState) == NULL || newState == m_currentState) 
		{
        return; // 无效或重复状态，忽略
    }
    // 获取当前状态配置
    const StateConfig* currentConfig = findConfig(m_currentState);
    // 执行当前状态的退出动作（如果存在）
    if (currentConfig != NULL && currentConfig->onExit != NULL) 
		{
        currentConfig->onExit(this, newState);
    }
    // 更新状态记录
    m_prevState = m_currentState;
    m_currentState = newState;
    m_stateEnterTime = m_currentTime; // 记录状态进入时间
    // 获取新状态配置
    const StateConfig* newConfig = findConfig(newState);
    // 执行新状态的进入动作（如果存在）
    if (newConfig != NULL && newConfig->onEnter != NULL) {
        newConfig->onEnter(this, m_prevState);
    }
}
/*******************************************************************************
* @功能     		: 更新时间戳
* @参数         : 每次递增的时间ms
* @返回值 				: void
* @概述  				: 递增内部时间计数器，建议使用1ms定时器循环去跑这个函数
*******************************************************************************/
void StateMachine::updateTime(uint16_t time_once) 
{
    m_currentTime += time_once; // 简单递增时间计数器
}
/*******************************************************************************
* @功能     		: 执行存续动作
* @参数         : None
* @返回值 				: void
* @概述  				: 调用当前状态的onStay函数
*******************************************************************************/
void StateMachine::executeStayAction() 
{
    // 获取当前状态配置
    const StateConfig* currentConfig = findConfig(m_currentState);
    // 如果配置存在且定义了存续动作
    if (currentConfig != NULL && currentConfig->onStay != NULL) 
		{
        // 计算在当前状态的持续时间
        uint32_t elapsed = m_currentTime - m_stateEnterTime;
        // 调用存续动作函数
        currentConfig->onStay(this, elapsed);
    }
}
