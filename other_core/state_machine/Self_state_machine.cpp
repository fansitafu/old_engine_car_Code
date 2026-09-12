#include "Self_state_machine.h"



StateConfig_Extend* Car_StateConfig[MAX_STATE_MACHINES]; // 车辆状态配置结构体
StateConfig Car_StateConfig_base[MAX_STATE_MACHINES]; // 车辆状态配置结构体
StateMachine_Extend Car_StateMachine(MAX_STATE_MACHINES, Car_StateConfig, Car_StateConfig_base); // 车辆状态机结构体

StateConfig Car_StateConfig_base1[MAX_STATE_MACHINES];
StateMachine Car_StateMachine1(MAX_STATE_MACHINES, Car_StateConfig_base1);


uint8_t Flag_state[MAX_STATE_MACHINES];
// StateConfig_Extend* Data_updata_StateConfig[MAX_STATE_MACHINES]; // 车辆状态配置结构体
// StateConfig Data_updata_StateConfig_base[MAX_STATE_MACHINES]; // 车辆状态配置结构体
// StateMachine_Extend Data_updata_StateMachine(MAX_STATE_MACHINES, Data_updata_StateConfig, Data_updata_StateConfig_base); // 车辆状态机结构体




// Basis_Control_ClassDef::Basis_Control_ClassDef()
// {

//     for (uint8_t i = 0; i < MAX_STATE_MACHINES; i++)
//     {
//         _State_Machine_ID[i] = -1; // 初始化状态机ID为-1，表示未注册
//     }

// }

// /*******************************************************************************/
// * @功能     	: 状态机注册函数
// * @参数1        : 要注册的状态机进入函数指针
// * @参数2        : 要注册的状态机退出函数指针
// * @参数3        : 要注册的状态机运行函数指针
// * @参数4        : 要注册的状态机ID
// * @返回值 		: bool，是否注册成功
// * @概述  		: 用于注册以后，状态机运行时，根据ID调用对应函数指针执行状态机逻辑
// *******************************************************************************/
// bool Basis_Control_ClassDef::RegisterSetState(void (*State_in)(void), void (*State_exit)(void), void (*State_run)(void), int StateID)
// {
//     // 检查参数：电机非空 + 未超过最大注册数
//     if (State_in == NULL && State_exit == NULL && State_run == NULL)
//         return false;
//     if (StateID >= MAX_STATE_MACHINES)
//         return false;


//     // 新增注册
//     _State_Machine_In_Func[StateID] = State_in;
//     _State_Machine_Exit_Func[StateID] = State_exit;
//     _State_Machine_Run_Func[StateID] = State_run;

//     // _StateCount++;  
    
//     return true;
// }

/*******************************************************************************/
// void Basis_Control_ClassDef::Status_Running(int StateID)
// {
//     static int first_run = 0; // 用于标记是否第一次运行状态机
//     static int last_state_id = -1; // 用于记录上一次运行的状态机ID


//     if (last_state_id != -1)
//     {
//         if (_State_Machine_Exit_Func[last_state_id] != NULL && last_state_id != StateID) // 如果上一个状态机的退出函数指针不为空且上一个状态机ID有效
//         {
//             _State_Machine_Exit_Func[last_state_id](); // 调用上一个状态机的退出函数
//         }

//     }

//     if (_State_Machine_In_Func[StateID] != NULL && last_state_id != StateID)
//     {
//         _State_Machine_In_Func[StateID]();
//     }

//     if (_State_Machine_Run_Func[StateID] != NULL )
//     {
//         _State_Machine_Run_Func[StateID]();
//     }    

//     last_state_id = StateID; // 更新上一次运行的状态机ID

// }




//状态机的另一个使用方法,遍历所有的状态机，并根据标志位决定这个状态机是否运行

StateMachine_Extend::StateMachine_Extend(uint8_t maxStates, StateConfig_Extend** StateBuffer_extend, StateConfig* StateBuffer_base)
    : m_states_extend(StateBuffer_extend),
      StateMachine(maxStates, StateBuffer_base)
{
}




StateConfig_Extend* StateMachine_Extend::findConfig_extend(uint8_t state)
{
    // 遍历所有已注册状态
    for (uint8_t i = 0; i < m_stateCount; ++i) 
		{
        // 匹配状态标识
        if (m_states_extend[i]->config_base.state == state) 
				{
            return m_states_extend[i];
        }
    }
    return NULL; // 未找到匹配项
}



bool StateMachine_Extend::registerState_extend(StateConfig_Extend* config)
{
        // 检查状态数组是否已满
    if (m_stateCount >= m_maxStates) return false;
    // 检查状态是否已存在
    if (findConfig_extend(config->config_base.state) != NULL) return false;
    // 添加新状态配置到数组
    m_states_extend[m_stateCount] = config;
    registerState(config->config_base); // 调用基类的registerState方法注册基础状态配置
    return true;
}








void StateMachine_Extend::Status_Running()
{

    for (int priority_ = 0; priority_ < 3; priority_++)
    {

        for (int i = 0; i < m_stateCount; i++)
        {
            if (m_states_extend[i]->If_run == RUNNING && m_states_extend[i]->priority == priority_)
            {
                setState(m_states_extend[i]->config_base.state);
                executeStayAction();
            }
            else
            {
                continue; // 如果状态机未运行，则跳过
            }
        }
    }


}




bool StateMachine_Extend::Change_State_If_run(uint8_t state, uint8_t If_run_flag)
{
    StateConfig_Extend* config_ = findConfig_extend(state);
    if(config_ == NULL) return false;
    config_->If_run = If_run_flag;
    return true;

}





bool StateMachine_Extend::Change_State_If_run(const StateConfig_Extend& config, uint8_t If_run_flag)
{
    StateConfig_Extend* config_ = findConfig_extend(config.config_base.state);  
    if (config_ == NULL) return false;
    config_->If_run = If_run_flag;
    return true;

}

