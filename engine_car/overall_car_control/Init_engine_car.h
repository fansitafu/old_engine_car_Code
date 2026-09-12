#ifndef INIT_ENGINE_CAR_H
#define INIT_ENGINE_CAR_H

#include "ifr_lib.h"
#include "car_basis_control.h"
#include "self_codebase_include.h"
#include "usart.h"
#include "tim.h"
#include "bmi088.h"
#include "Judge.h"
#include "Init_car_arm.h"







extern IFR_TIM_ClassDef   Motor_TIM2; // TIM2
extern IFR_TIM_ClassDef   State_TIM1; // TIM1
extern IFR_TIM_ClassDef   TIM3_1ms; // TIM3

extern float Custom_motor_target_Angle[6];



void Data_Flag_updating_mode();

void Motor_Mode_Flag(void);
void Motor_Date_Updating(void);
void TIM3_1ms_Callback();

void Init_engine_car();
void IMU_Data_Updating();

void Arm_custom_control_onStay_state(StateMachine* fsm, uint32_t elapsedMs);
void Arm_stop_onStay_state(StateMachine* fsm, uint32_t elapsedMs);
void Power_star_onStay_state(StateMachine* fsm, uint32_t elapsedMs);

void Arm_Motor_Date_Updating(void);

void Arm_rise_ready_onStay_state(StateMachine* fsm, uint32_t elapsedMs);
void Arm_rise_ready_onEnter_state(StateMachine* fsm, uint8_t  prevState);
void Arm_fall_ready_onStay_state(StateMachine* fsm, uint32_t elapsedMs);
void Arm_fall_ready_onEnter_state(StateMachine* fsm, uint8_t  prevState);


void All_stop_onStay_state(StateMachine* fsm, uint32_t elapsedMs);
void Car_Moving_onStay_state(StateMachine* fsm, uint32_t elapsedMs);
void Car_arm_rise_ready_onStay_state(StateMachine* fsm, uint32_t elapsedMs);
void Car_arm_fall_ready_onStay_state(StateMachine* fsm, uint32_t elapsedMs);
void Car_custom_control_onStay_state(StateMachine* fsm, uint32_t elapsedMs);

void Only_arm_control_onStay_state(StateMachine* fsm, uint32_t elapsedMs);
void Only_arm_control_onEnter_state(StateMachine* fsm, uint8_t prevState);

void Servo_SetAngle(uint16_t angle);

void Only_basis_control_onStay_state(StateMachine* fsm, uint32_t elapsedMs);

#endif // INIT_ENGINE_CAR_H