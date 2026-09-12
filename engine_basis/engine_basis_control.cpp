#include "engine_basis_control.h"




uint8_t usart5_rx_buffer[50] = {0};  // USART5接收缓冲区

IFR_USART_ClassDef Engine_Basis_USART;


IFR_TIM_ClassDef   Engine_Basis_TIM; // TIM2



CanMsgQueue        Engine_Basis_CAN1_Queue; // CAN消息队列
IFR_FDCAN_ClassDef Engine_Basis_FDCAN1; // FDCAN对象
IFR_DJI_Speed_Motor DJ_Motor1(0x201, DJI_3508, &Engine_Basis_CAN1_Queue), 
                    DJ_Motor2(0x202, DJI_3508, &Engine_Basis_CAN1_Queue), 
                    DJ_Motor3(0x203, DJI_3508, &Engine_Basis_CAN1_Queue), 
                    DJ_Motor4(0x204, DJI_3508, &Engine_Basis_CAN1_Queue); // 电机对象

Coordinate_car Car_Coordinate; // 车辆坐标结构体

float init_kp = 0.01f;
float init_ki = 0.01f;
float init_kd = 0.01f;
float init_Integral_Separation_Threshold = 6.0f;


void Engine_Basis_Control_Init(void)
{

//	Engine_Basis_USART.ifr_usart_Init(&huart5, WBUS_Remote_Analysis);  // 初始化USART5，并设置解析函数为WBUS_Remote_Analysis
	Remote_AnyType.Remote_Init(&huart1, REMOTE_TYPE_VT13);
	
	
	Engine_Basis_FDCAN1.RegisterMotor(&DJ_Motor1); // 注册电机对象到FDCAN控制器
	Engine_Basis_FDCAN1.RegisterMotor(&DJ_Motor2);
	Engine_Basis_FDCAN1.RegisterMotor(&DJ_Motor3);
	Engine_Basis_FDCAN1.RegisterMotor(&DJ_Motor4);
  Engine_Basis_FDCAN1.FDCAN_Init(&hfdcan1, &Engine_Basis_CAN1_Queue); // 初始化FDCAN1，绑定消息队列

	DJ_Motor1.Speed_PID.PID_Init(init_kp, init_ki, init_kd, 10.0f, 1000.0f, 500.0f, 1.0f, init_Integral_Separation_Threshold, 0.3f);
	DJ_Motor2.Speed_PID.PID_Init(init_kp, init_ki, init_kd, 10.0f, 1000.0f, 500.0f, 1.0f, init_Integral_Separation_Threshold, 0.3f);
	DJ_Motor3.Speed_PID.PID_Init(init_kp, init_ki, init_kd, 10.0f, 1000.0f, 500.0f, 1.0f, init_Integral_Separation_Threshold, 0.3f);
	DJ_Motor4.Speed_PID.PID_Init(init_kp, init_ki, init_kd, 10.0f, 1000.0f, 500.0f, 1.0f, init_Integral_Separation_Threshold, 0.3f);
	
	DM04.Enable();
	DM05.Enable();
	DM06.Enable();
	Engine_Basis_TIM.TIM_ITStart(&htim2, FDCAN_motor_mode); // 初始化TIM2，开启定时器中断
}       


uint32_t now;

/**
 * 定时器回调函数中的处理函数
 * 
 */
int time = 0;
int rc_connected = 0;
uint16_t last_systick;
uint8_t DM_swit = 0;
void FDCAN_motor_mode()
{



    

if (time != 999)
{

			time++;
}	
else
{
			Engine_Basis_FDCAN1.Motor_All_Enable();
			Mechanical_arm_FDCAN3.Motor_All_Enable();
			Mechanical_arm_FDCAN2.Motor_All_Enable();
        //遥控器断开连接时，停止电机输出
    if(WBUS_Remote.Switch_D == 1) // 模式切换
    {
        DJ_Motor1.set_electric(0); // 设置电机1的电流为0
        DJ_Motor2.set_electric(0); // 设置电机2的电流为0
        DJ_Motor3.set_electric(0); // 设置电机3的电流为0
        DJ_Motor4.set_electric(0); // 设置电机4的电流为0

        LZ01.set_torque(0.0f); // 设置电机4的电流为0
        LZ02.set_torque(0.0f); // 设置电机4的电流为0
        LZ03.set_torque(0.0f); // 设置电机4的电流为0


        DM04.set_torque(0.0f); // 设置电机4的电流为0
				DM05.set_torque(0.0f); // 设置电机4的电流为0
				DM06.set_torque(0.0f); // 设置电机4的电流为0
//				if(DM_swit == 0)
//				{
//					DM05.set_torque(0.0f); // 设置电机4的电流为0
//					DM_swit = 1;
//				}
//				else if(DM_swit == 1)
//				{
//					DM06.set_torque(0.0f); // 设置电机4的电流为0
//					DM_swit = 0;
//				}

    }
    else if(WBUS_Remote.Switch_D == 2) // 模式切换
    {
       mode_switch(); // 模式切换函数
    }
}

    now = HAL_GetTick();


//DM06.set_torque(0.0f);
//DM04.set_torque(0.0f);
//DM05.set_torque(0.0f);

	Engine_Basis_FDCAN1.SendQueuedMsgs();
	Mechanical_arm_FDCAN3.SendQueuedMsgs();
	Mechanical_arm_FDCAN2.SendQueuedMsgs();

//	Arm_matrix_calculate();

}






void mode_switch()
{
    // 模式切换逻辑（如有需要，可以在此处添加不同模式的切换条件和处理）

    if(WBUS_Remote.Switch_C == 1) // 切换到模式1
    {
        Move_mode();
				LZ01.set_torque(0.0f); // 设置电机4的电流为0
        LZ02.set_torque(0.0f); // 设置电机4的电流为0
        LZ03.set_torque(0.0f); // 设置电机4的电流为0
        DM04.set_torque(0.0f); // 设置电机4的电流为0
				DM05.set_torque(0.0f); // 设置电机4的电流为0
				DM06.set_torque(0.0f); // 设置电机4的电流为0
    }
    else if(WBUS_Remote.Switch_C == 2) // 切换到模式2
    {
        mode2();
				DJ_Motor1.set_electric(0); // 设置电机1的电流为0
        DJ_Motor2.set_electric(0); // 设置电机2的电流为0
        DJ_Motor3.set_electric(0); // 设置电机3的电流为0
        DJ_Motor4.set_electric(0); // 设置电机4的电流为0
    }
    else if(WBUS_Remote.Switch_C == 3) // 切换到模式3
    {
        // 模式3的具体控制逻辑（如有需要，可以在此处实现模式3的控制算法）
        Move_mode();
				mode2();

    }
}



float DJ_Motor1_Speed_Tar = 0.0f; // 电机目标速度
float DJ_Motor2_Speed_Tar = 0.0f; // 电机目标速度
float DJ_Motor3_Speed_Tar = 0.0f; // 电机目标速度
float DJ_Motor4_Speed_Tar = 0.0f; // 电机目标速度
float speed_percent = 500.0f; // 设置遥控器速度百分比

void Move_mode()
{
    // 模式1的具体控制逻辑（如有需要，可以在此处实现模式1的控制算法）

    Car_Coordinate.dir_x = WBUS_Remote.Chx_Left  * speed_percent; // 从遥控器输入获取车辆前后方向
    Car_Coordinate.dir_y = WBUS_Remote.Chy_Left  * speed_percent; // 从遥控器输入获取车辆左右方向
    Car_Coordinate.yaw   = WBUS_Remote.Chx_Right * speed_percent; // 从遥控器输入获取车辆旋转方向





    DJ_Motor1_Speed_Tar = - Car_Coordinate.dir_x + Car_Coordinate.dir_y - Car_Coordinate.yaw; // 根据遥控器输入计算电机1的目标速度
    DJ_Motor2_Speed_Tar = - Car_Coordinate.dir_x - Car_Coordinate.dir_y - Car_Coordinate.yaw;
    DJ_Motor3_Speed_Tar =   Car_Coordinate.dir_x - Car_Coordinate.dir_y - Car_Coordinate.yaw;
    DJ_Motor4_Speed_Tar =   Car_Coordinate.dir_x + Car_Coordinate.dir_y - Car_Coordinate.yaw;

    DJ_Motor1.Motor_Speed_Set(DJ_Motor1_Speed_Tar); // 设置电机1的目标速度为0
    DJ_Motor2.Motor_Speed_Set(DJ_Motor2_Speed_Tar); // 设置电机2的目标速度为0
    DJ_Motor3.Motor_Speed_Set(DJ_Motor3_Speed_Tar); // 设置电机3的目标速度为0
    DJ_Motor4.Motor_Speed_Set(DJ_Motor4_Speed_Tar); // 设置电机4的目标速度为0


}



float tar_elt = 0.0f;
void mode2()
{
    // 模式2的具体控制逻辑（如有需要，可以在此处实现模式2的控制算法）
		






//	LZ01.Motor_Pos_Set(arm_inverse_6dof.target_joint_angles[0]); // 设置电机4的电流为0
//	LZ02.Motor_Pos_Set(arm_inverse_6dof.target_joint_angles[1]); // 设置电机4的电流为0
//	LZ03.Motor_Pos_Set(arm_inverse_6dof.target_joint_angles[2]); // 设置电机4的电流为0

//	DM04.Motor_Pos_Set(arm_inverse_6dof.target_joint_angles[3]); // 设置电机4的电流为0
//	DM05.Motor_Pos_Set(arm_inverse_6dof.target_joint_angles[4]); // 设置电机4的电流为0
//	DM06.Motor_Pos_Set(arm_inverse_6dof.target_joint_angles[5]); // 设置电机4的电流为0

//	DM04.Motor_Pos_Set(0); // 设置电机4的电流为0
//	DM05.Motor_Pos_Set(0); // 设置电机4的电流为0
//	DM06.Motor_Pos_Set(0); // 设置电机4的电流为0

//		DM06.Motor_Speed_Set(10);
//    DM06.Motor_Pos_Set(0.0f); 
//	DM05.Motor_Pos_Set(0.0f); 
//	DM04.Motor_Pos_Set(3.6f); 
				// LZ01.set_torque(0.0f); // 设置电机4的电流为0
        // LZ02.set_torque(0.0f); // 设置电机4的电流为0
        // LZ03.set_torque(0.0f); // 设置电机4的电流为0
        // DM04.set_torque(0.0f); // 设置电机4的电流为0
				// DM05.set_torque(0.0f); // 设置电机4的电流为0
				// DM06.set_torque(0.0f); // 设置电机4的电流为0



	Mechanical_arm_FDCAN3.Motor_All_Enable();
	Mechanical_arm_FDCAN2.Motor_All_Enable();

	Mechanical_arm_FDCAN3.SendQueuedMsgs();
	Mechanical_arm_FDCAN2.SendQueuedMsgs();



}

void mode3()
{
    
}








