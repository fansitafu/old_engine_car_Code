#ifndef __INIT_CAR_ARM_H
#define __INIT_CAR_ARM_H




#include "ifr_lib.h"
#include "fdcan.h"






extern CanMsgQueue        Mechanical_arm_CAN2_Queue; // 达妙CAN消息队列
extern CanMsgQueue        Mechanical_arm_CAN3_Queue; // 灵足CAN消息队列
extern IFR_FDCAN_ClassDef Mechanical_arm_FDCAN2; // FDCAN对象
extern IFR_FDCAN_ClassDef Mechanical_arm_FDCAN3; // FDCAN对象
extern IFR_RS_Pos_Motor LZ01;
extern IFR_RS_Pos_Motor LZ02;
extern IFR_RS_Pos_Motor LZ03;
//extern IFR_RS_Pos_Motor LZ04;
//extern IFR_RS_Pos_Motor LZ05;
//extern IFR_RS_Pos_Motor LZ06;
extern IFR_DM_Pos_Motor DM04 ;
extern IFR_DM_Pos_Motor DM05 ;
extern IFR_DM_Pos_Motor DM06 ;

void mechanical_arm_init() ;
float LZ03_Offset( float angle_);
float LZ02_Offset( float angle_);
float LZ01_Offset( float angle_);





#endif // __INIT_CAR_ARM_H