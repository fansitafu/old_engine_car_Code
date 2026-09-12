/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_state_machine.h
  * Version		: v1.0
  * Author		: panjiajun
  * Date			: 2025-11-10
  * Description	:	IFR状态机库
  *********************************************************************
  */
/*******************************************************************************
* @功能     		: 有限状态机类声明
* @参数         : None
* @返回值 				: void
* @概述  				: 定义状态机核心类及状态配置结构
*******************************************************************************/
#ifndef IFR_STATEMACHINE_H
#define IFR_STATEMACHINE_H
#include <stdint.h>
#include "main.h"
// 前向声明StateMachine类
class StateMachine;
/*******************************************************************************
* @功能     		: 状态进入动作函数类型
* @参数         : None
* @返回值 				: void
* @概述  				: 定义状态进入时调用的函数类型
*******************************************************************************/
typedef void (*StateEnterFunc)(StateMachine* fsm, uint8_t prevState);
/*******************************************************************************
* @功能     		: 状态退出动作函数类型
* @参数         : None
* @返回值 				: void
* @概述  				: 定义状态退出时调用的函数类型
*******************************************************************************/
typedef void (*StateExitFunc)(StateMachine* fsm, uint8_t nextState);
/*******************************************************************************
* @功能     		: 状态存续动作函数类型
* @参数         : None
* @返回值 				: void
* @概述  				: 定义状态持续期间周期性调用的函数类型
*******************************************************************************/
typedef void (*StateStayFunc)(StateMachine* fsm, uint32_t elapsedMs);
/*******************************************************************************
* @功能     		: 状态配置结构体
* @参数         : None
* @返回值 				: void
* @概述  				: 绑定状态标识与对应的动作函数
*******************************************************************************/
struct StateConfig 
{
    /**
      * @概述	状态唯一标识符
      */
    uint8_t state;
    /**
      * @概述	状态进入动作函数指针
      */
    StateEnterFunc onEnter;
    /**
      * @概述	状态退出动作函数指针
      */
    StateExitFunc onExit;
    /**
      * @概述	状态存续动作函数指针
      */
    StateStayFunc onStay;
};
/*******************************************************************************
* @功能     		: 有限状态机类
* @参数         : None
* @返回值 				: void
* @概述  				: 实现状态管理、切换和动作执行的核心类
*******************************************************************************/
class StateMachine 
{
protected:
    /**
      * @概述	当前活跃状态标识
      */
    uint8_t m_currentState;
    /**
      * @概述	上一个状态标识
      */
    uint8_t m_prevState;
    /**
      * @概述	进入当前状态的时间戳(毫秒)
      */
    uint32_t m_stateEnterTime;
    /**
      * @概述	当前系统时间戳(毫秒)
      */
    uint32_t m_currentTime;
    /**
      * @概述	状态配置数组指针
      */
    StateConfig* m_states;
    /**
      * @概述	支持的最大状态数量
      */
    uint8_t m_maxStates;
    /**
      * @概述	当前已注册的状态数量
      */
    uint8_t m_stateCount;
    /**
      * @概述	根据状态标识查找配置项
      * @参数1	state - 要查找的状态标识
      * @返回值	找到的配置项指针或NULL
      */
    const StateConfig* findConfig(uint8_t state) const;
public:
    StateMachine(uint8_t maxStates, StateConfig* stateBuffer);
    ~StateMachine();
    bool registerState(const StateConfig& config);
    void setState(uint8_t newState);
    void updateTime(uint16_t time_once);
    void executeStayAction();
    uint8_t getCurrentState() const { return m_currentState; }
    uint32_t getStateDuration() const { return m_currentTime - m_stateEnterTime; }
    uint8_t getPrevState() const { return m_prevState; }
};
#endif // IFR_STATEMACHINE_H
