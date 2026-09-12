#ifndef __ENGINE_BASIS_CONTROL_H
#define __ENGINE_BASIS_CONTROL_H

#include "ifr_lib.h"
#include "usart.h"
#include "tim.h"
#include "fdcan.h"

#include "Receive_remote_ifr.h"



// extern IFR_USART_ClassDef Engine_Basis_Control_USART (&huart5, WBUS_Remote_Analysis);

void Engine_Basis_Control_Init(void);
void FDCAN_motor_mode();
void mode_switch();
void Move_mode();
void mode2();
void mode3();



typedef struct
{
	
	float dir_x;
	float dir_y;
	float yaw;
	

}Coordinate_car ;

extern float tar_elt;



#endif