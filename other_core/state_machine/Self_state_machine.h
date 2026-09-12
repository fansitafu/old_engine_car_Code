#ifndef SETUP_AND_DETECTION_FUNCTION_FLAG_H
#define SETUP_AND_DETECTION_FUNCTION_FLAG_H


#include "ifr_state_machine.h"


#define RUNNING 1 // 运行标志
#define NOT_RUNNING 0 // 未运行标志
#define HIGH_PRIORITY 0 // 高优先级标志
#define MEDIUM_PRIORITY 1 // 中优先级标志
#define LOW_PRIORITY 2 // 低优先级标志
#define MAX_STATE_MACHINES 20 // 最大状态机数量


// extern uint8_t Flag_state[MAX_STATE_MACHINES];

extern StateConfig Car_StateConfig_base1[MAX_STATE_MACHINES];
extern StateMachine Car_StateMachine1;

// #define MAX_STATE_MACHINES 10 // 最大状态机数量
// #define COMPLETED 1 // 完成标志
// #define NOT_COMPLETED 0 // 未完成标志


// typedef void (*State_Machine_In_FuncPtr)(void);          //状态机进入时函数指针，
// typedef void (*State_Machine_Exit_FuncPtr)(void);        //状态机退出时函数指针，
// typedef void (*State_Machine_Run_FuncPtr)(void);         //状态机运行时函数指针,

// class Basis_Control_ClassDef
// {   
// public:
//     Basis_Control_ClassDef();
//     ~Basis_Control_ClassDef();
//     bool RegisterSetState(void (*State_in)(void), void (*State_exit)(void), void (*State_run)(void), int StateID);        //注册状态机函数
//     void Status_Running(int StateID);
//     void Basis_Control_ClassDef_Init();
//     void Basis_Control_ClassDef_Uninit();
//     void Basis_Control_ClassDef_Run();
//     void Basis_Control_ClassDef_Stop();
//     void Basis_Control_ClassDef_Reset();
//     void Basis_Control_ClassDef_SetParam(int param1, int param2, int param3, int param4, int param5, int param6, int param7, int param8, int param9, int param10);

// protected:
//     State_Machine_In_FuncPtr   _State_Machine_In_Func[MAX_STATE_MACHINES]; //= {NULL};        //状态机进入时函数指针，
//     State_Machine_Exit_FuncPtr _State_Machine_Exit_Func[MAX_STATE_MACHINES]; //= {NULL};    //状态机退出时函数指针，
//     State_Machine_Run_FuncPtr  _State_Machine_Run_Func[MAX_STATE_MACHINES]; //= {NULL};    //状态机运行时函数指针,
//     int _State_Machine_ID[MAX_STATE_MACHINES];
//     // int _StateCount = 0; // 已注册状态机数量
// };


struct StateConfig_Extend
{    
    uint8_t If_run; // 是否运行标志
    uint8_t priority; // 优先级标志
    StateConfig config_base; // 基础状态配置

};


class StateMachine_Extend : public StateMachine
{

public:
    StateMachine_Extend(uint8_t maxStates, StateConfig_Extend** stateBuffer, StateConfig* StateBuffer_base);


    StateConfig_Extend* findConfig_extend(uint8_t state);
    bool registerState_extend(StateConfig_Extend* config);
    void Status_Running();
    bool Change_State_If_run(uint8_t state, uint8_t If_run_flag);
    bool Change_State_If_run(const StateConfig_Extend& config, uint8_t If_run_flag);



private:
    StateConfig_Extend** m_states_extend;

};





extern StateConfig_Extend* Car_StateConfig[MAX_STATE_MACHINES]; // 车辆状态配置结构体
extern StateConfig Car_StateConfig_base[MAX_STATE_MACHINES]; // 车辆状态配置结构体
extern StateMachine_Extend Car_StateMachine; // 车辆状态机结构体












#endif // SETUP_AND_DETECTION_FUNCTION_FLAG_H