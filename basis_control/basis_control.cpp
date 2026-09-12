#include "basis_control.h"




// Basis_Control_ClassDef::Basis_Control_ClassDef()
// {

//     for (uint8_t i = 0; i < MAX_STATE_MACHINES; i++)
//     {
//         _State_Machine_ID[i] = -1; // 初始化状态机ID为-1，表示未注册
//     }

// }







// /*******************************************************************************
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





























