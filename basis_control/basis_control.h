#ifndef _BASIS_CONTROL_H
#define _BASIS_CONTROL_H


#include "ifr_lib.h"



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
































#endif /* _BASIS_CONTROL_H */