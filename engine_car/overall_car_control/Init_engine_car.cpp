#include "Init_engine_car.h"

  #define OPEN 40
#define CLOSE 110


IFR_TIM_ClassDef   Motor_TIM2; // TIM2
IFR_TIM_ClassDef   State_TIM1; // TIM1
IFR_TIM_ClassDef   TIM3_1ms; // TIM3
// IFR_USART_ClassDef IFR_USART5;
IFR_USART_ClassDef Judgment_Date_USART; // 串口1，接收裁判系统数据
IFR_USART_ClassDef IFR_USART7; // 串口7，接收IMU数据


StateConfig Car_Moving_State = {0x00, NULL, NULL, Car_Moving_onStay_state}; // 车体状态
StateConfig All_Stop_State = {0x01, NULL, NULL, All_stop_onStay_state}; // 车体状态
StateConfig Car_arm_rise_State = {0x02, Arm_rise_ready_onEnter_state, NULL, Car_arm_rise_ready_onStay_state}; // 车体状态
StateConfig Car_arm_fall_State = {0x03, Arm_fall_ready_onEnter_state, NULL, Car_arm_fall_ready_onStay_state}; // 车体状态
StateConfig Car_Custom_control_State = {0x04, NULL, NULL, Car_custom_control_onStay_state}; // 车体状态
StateConfig Only_arm_control_State = {0x05, Only_arm_control_onEnter_state, NULL, Only_arm_control_onStay_state}; // 车体状态
StateConfig Only_basis_control_State = {0x06, Only_arm_control_onEnter_state, NULL, Only_basis_control_onStay_state}; // 车体状态




uint16_t an_d = 90;
//int CLOSE = 40;

void Init_engine_car()
{
    

  Init_car_basis();
  	mechanical_arm_init();
	Judgment_Date_USART.ifr_usart_Init(&huart10, JudgeData_analysis); // 初始化串口3，绑定裁判系统数据解析函数
	// IFR_USART5.ifr_usart_Init(&huart5, &DT7_Remote_Analysis);
	Remote_AnyType.Remote_Init(&huart5, REMOTE_TYPE_DT7);  //绑定遥控器的类型和其所用的串口

	IFR_USART7.ifr_usart_Init(&huart7, Custom_Remote_Controller_Analysis); // 初始化串口7，绑定IMU数据解析函数


	Car_StateMachine1.registerState(Car_Moving_State); // 注册状态机状态0
	Car_StateMachine1.registerState(All_Stop_State); // 注册状态机状态1
	Car_StateMachine1.registerState(Car_arm_rise_State); // 注册状态机状态2
	Car_StateMachine1.registerState(Car_arm_fall_State); // 注册状态机状态3
	Car_StateMachine1.registerState(Car_Custom_control_State); // 注册状态机状态4
	Car_StateMachine1.registerState(Only_arm_control_State); // 注册状态机状态5
	Car_StateMachine1.registerState(Only_basis_control_State); // 注册状态机状态6

	State_TIM1.TIM_ITStart(&htim1, Data_Flag_updating_mode); // 初始化TIM1，开启定时器中断，绑定状态机函数执行函数
	TIM3_1ms.TIM_ITStart(&htim3, TIM3_1ms_Callback); // 初始化TIM3，开启定时器中断，绑定电机数据更新函数
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	Servo_SetAngle(an_d);
	Car_StateMachine1.setState(0x01);
	Car_StateMachine1.executeStayAction();
}



//将数据更新也采用状态机形式
void Data_Flag_updating_mode()
{




	Motor_Mode_Flag();
}




void TIM3_1ms_Callback()
{
	Calibrate();
}




int time_;



void Motor_Mode_Flag(void)
{
	Remote_AnyType_Rx_Data_TypeDef* remote_data_ = Remote_AnyType.GetRemoteData();

	uint32_t Remote_Updata_Time = HAL_GetTick();
	uint32_t Remote_Last_Updata_Time = remote_data_->Last_Updata_Time;

	if (Remote_Updata_Time < 2000)
	{

		return; // 如果系统时间小于2000毫秒，直接返回，不进行状态切换,状态机里执行默认运行的状态
	}


time_++;
if(time_ < 2000)
{

	Engine_Basis_FDCAN1.Motor_All_Enable();
	Mechanical_arm_FDCAN3.Motor_All_Enable();
	Mechanical_arm_FDCAN2.Motor_All_Enable();

	Engine_Basis_FDCAN1.SendQueuedMsgs();
	Mechanical_arm_FDCAN3.SendQueuedMsgs();
	Mechanical_arm_FDCAN2.SendQueuedMsgs();

	return;
}



	if (Remote_Updata_Time - Remote_Last_Updata_Time >= 100 || remote_data_->Switch_Left == 1)  Car_StateMachine1.setState(0x01);
	else if(remote_data_->Switch_Left == 2 && remote_data_->Switch_Right == 1)					Car_StateMachine1.setState(0x03);
	else if(remote_data_->Switch_Left == 2 && remote_data_->Switch_Right == 3)					Car_StateMachine1.setState(0x03);
	else if(remote_data_->Switch_Left == 2 && remote_data_->Switch_Right == 2)					Car_StateMachine1.setState(0x02);
	else if(remote_data_->Switch_Left == 3 && remote_data_->Switch_Right == 1)				    Car_StateMachine1.setState(0x03);
	else if(remote_data_->Switch_Left == 3 && remote_data_->Switch_Right == 3)					Car_StateMachine1.setState(0x05);
	else if(remote_data_->Switch_Left == 3 && remote_data_->Switch_Right == 2)					Car_StateMachine1.setState(0x06);


Servo_SetAngle(an_d);

	Car_StateMachine1.executeStayAction();

	Engine_Basis_FDCAN1.Motor_All_Enable();
	Mechanical_arm_FDCAN3.Motor_All_Enable();
	Mechanical_arm_FDCAN2.Motor_All_Enable();

	Engine_Basis_FDCAN1.SendQueuedMsgs();
	Mechanical_arm_FDCAN3.SendQueuedMsgs();
	Mechanical_arm_FDCAN2.SendQueuedMsgs();


}


void Motor_Date_Updating(void)
{
	Basis_Motor_Date_Updating();
	Arm_Motor_Date_Updating();
}



float Custom_motor_target_Angle[6];

void Arm_Motor_Date_Updating(void)
{
    Custom_motor_target_Angle[0] = Custom_joint_Radian[0] + 3.5f;
    Custom_motor_target_Angle[1] = Custom_joint_Radian[1] + 0.3f;
    Custom_motor_target_Angle[2] = Custom_joint_Radian[2] + 0.3f;
    Custom_motor_target_Angle[3] = Custom_joint_Radian[3] + 2.16f;
    Custom_motor_target_Angle[4] = Custom_joint_Radian[4] + 3.05f;
    Custom_motor_target_Angle[5] = Custom_joint_Radian[5] - 0.0f;

}







void Arm_stop_onStay_state(StateMachine* fsm, uint32_t elapsedMs)
{


	LZ01.set_torque(0.0f); // 设置电机4的电流为0
	LZ02.set_torque(0.0f); // 设置电机4的电流为0
	LZ03.set_torque(0.0f); // 设置电机4的电流为0
	DM04.set_torque(0.0f); // 设置电机4的电流为0
	DM05.set_torque(0.0f); // 设置电机4的电流为0
	DM06.set_torque(0.0f); // 设置电机4的电流为0



}




void Arm_custom_control_onStay_state(StateMachine* fsm, uint32_t elapsedMs)
{
	Motor_Date_Updating();

    LZ01.Motor_Pos_Set(Custom_motor_target_Angle[0]); // 设置电机4的电流为0
    LZ02.Motor_Pos_Set(Custom_motor_target_Angle[1]); // 设置电机4的电流为0
    LZ03.Motor_Pos_Set(Custom_motor_target_Angle[2]); // 设置电机4的电流为0
    DM04.Motor_Pos_Set(Custom_motor_target_Angle[3]); // 设置电机4的电流为0
    DM05.Motor_Pos_Set(Custom_motor_target_Angle[4]); // 设置电机4的电流为0
    DM06.Motor_Pos_Set(Custom_motor_target_Angle[5]); // 设置电机4的电流为0


}



void Arm_rise_ready_onStay_state(StateMachine* fsm, uint32_t elapsedMs)
{


	if(Custom_motor_target_Angle[0] < 2.85f ){Custom_motor_target_Angle[0] += 0.002f;}
	else if(Custom_motor_target_Angle[0] > 2.85f ){Custom_motor_target_Angle[0] -= 0.002f;}
	if(Custom_motor_target_Angle[1] < 2.25f ){Custom_motor_target_Angle[1] += 0.002f;}
	else if(Custom_motor_target_Angle[1] > 2.25f ){Custom_motor_target_Angle[1] -= 0.002f;}
	if(Custom_motor_target_Angle[2] < 5.0f ){Custom_motor_target_Angle[2] += 0.002f;}
	else if(Custom_motor_target_Angle[2] > 5.0f ){Custom_motor_target_Angle[2] -= 0.002f;}
	if(Custom_motor_target_Angle[3] < 1.0f ){Custom_motor_target_Angle[3] += 0.002f;}
	else if(Custom_motor_target_Angle[3] > 1.0f ){Custom_motor_target_Angle[3] -= 0.002f;}
	if(Custom_motor_target_Angle[4] < -1.04f ){Custom_motor_target_Angle[4] += 0.002f;}
	else if(Custom_motor_target_Angle[4] > -1.04f ){Custom_motor_target_Angle[4] -= 0.002f;}
Custom_motor_target_Angle[5] = 2.0f;

	LZ01.Motor_Pos_Set(Custom_motor_target_Angle[0]); // 设置电机4的电流为0
	LZ02.Motor_Pos_Set(Custom_motor_target_Angle[1]); // 设置电机4的电流为0
	LZ03.Motor_Pos_Set(Custom_motor_target_Angle[2]); // 设置电机4的电流为0
	DM04.Motor_Pos_Set(Custom_motor_target_Angle[3]); // 设置电机4的电流为0
	DM05.Motor_Pos_Set(Custom_motor_target_Angle[4]); // 设置电机4的电流为0
	DM06.Motor_Pos_Set(Custom_motor_target_Angle[5]); // 设置电机4的电流为0

	


}



void Arm_rise_ready_onEnter_state(StateMachine* fsm, uint8_t prevState)
{
	
	// if(prevState == 0x04)
	// {
	    
	//     Custom_motor_target_Angle[0] = 2.85f;
	// 	Custom_motor_target_Angle[1] = 2.25f;
	// 	Custom_motor_target_Angle[2] = 5.0f ;
	// 	Custom_motor_target_Angle[3] = 1.0f;
	// 	Custom_motor_target_Angle[4] = 2.05f;
	// 	Custom_motor_target_Angle[5] = 4.15f;
	// }
	// else if(prevState == 0x03)
	// {
	//    	Custom_motor_target_Angle[0] = 2.85f;
	// 	Custom_motor_target_Angle[1] = 1.15f;
	// 	Custom_motor_target_Angle[2] = 3.8f;
	// 	Custom_motor_target_Angle[3] = 1.0f;
	// 	Custom_motor_target_Angle[4] = 2.05f;
	// 	Custom_motor_target_Angle[5] = 4.15f; 
	// }


		Custom_motor_target_Angle[0] = LZ01.get_motordata().Angle;
		Custom_motor_target_Angle[1] = LZ02.get_motordata().Angle;
		Custom_motor_target_Angle[2] = LZ03.get_motordata().Angle;
		Custom_motor_target_Angle[3] = DM04.get_motordata().Angle;
		Custom_motor_target_Angle[4] = DM05.get_motordata().Angle;
		Custom_motor_target_Angle[5] = DM06.get_motordata().Angle;	


}

bool ZONE(float a,float b, float zone)
{
	if(a-b > 0)
	{
		if(a-b > zone)
		{
			return false;
		}
		else
		{
			return true;
		}
	}
	else if(a-b <= 0)
	{
	    if(a-b < -zone)
		{
			return false;
		}
		else
		{
			return true;
		}
	}
}

int process;

void Arm_fall_ready_onStay_state(StateMachine* fsm, uint32_t elapsedMs)
{



    if((ZONE(Custom_motor_target_Angle[0], 2.85f, 0.01f) == false || ZONE(Custom_motor_target_Angle[1], 2.25f, 0.01f) == false || ZONE(Custom_motor_target_Angle[2], 5.0f, 0.01f) == false || ZONE(Custom_motor_target_Angle[3], 1.0f, 0.01f) == false || ZONE(Custom_motor_target_Angle[4], -1.04f, 0.01f) == false ) && process == 0)
    {
		if(Custom_motor_target_Angle[1] < 1.3f && Custom_motor_target_Angle[2] < 4.0f)
		{
		    process = 1;

		}

		if(Custom_motor_target_Angle[0] < 2.85f ){Custom_motor_target_Angle[0] += 0.002f;}
		else if(Custom_motor_target_Angle[0] > 2.85f ){Custom_motor_target_Angle[0] -= 0.002f;}
		if(Custom_motor_target_Angle[1] < 2.25f ){Custom_motor_target_Angle[1] += 0.002f;}
		else if(Custom_motor_target_Angle[1] > 2.25f ){Custom_motor_target_Angle[1] -= 0.002f;}
		if(Custom_motor_target_Angle[2] < 5.0f ){Custom_motor_target_Angle[2] += 0.002f;}
		else if(Custom_motor_target_Angle[2] > 5.0f ){Custom_motor_target_Angle[2] -= 0.002f;}
		if(Custom_motor_target_Angle[3] < 1.0f ){Custom_motor_target_Angle[3] += 0.002f;}
		else if(Custom_motor_target_Angle[3] > 1.0f ){Custom_motor_target_Angle[3] -= 0.002f;}
		if(Custom_motor_target_Angle[4] <-1.04f ){Custom_motor_target_Angle[4] += 0.001f;}
		else if(Custom_motor_target_Angle[4] > -1.04f ){Custom_motor_target_Angle[4] -= 0.002f;}


		
	}
	else 
	{

		if(ZONE(Custom_motor_target_Angle[0], 2.85f, 0.01f)  == false)
		{
			if(Custom_motor_target_Angle[0] < 2.85f ){Custom_motor_target_Angle[0] += 0.002f;}
			else if(Custom_motor_target_Angle[0] > 2.85f ){Custom_motor_target_Angle[0] -= 0.002f;}
		    
		}
		if(Custom_motor_target_Angle[1] >= 1.15f)
		{
			Custom_motor_target_Angle[1] -= 0.002f;
		}
		if(Custom_motor_target_Angle[2] >= 3.8f)
		{
			Custom_motor_target_Angle[2] -= 0.002f;
		}
		if(ZONE(Custom_motor_target_Angle[3], 1.0f, 0.01f) == false)
		{
		    if(Custom_motor_target_Angle[3] < 1.0f ){Custom_motor_target_Angle[3] += 0.002f;}
			else if(Custom_motor_target_Angle[3] > 1.0f ){Custom_motor_target_Angle[3] -= 0.002f;}
		}
		if(ZONE(Custom_motor_target_Angle[4], -1.04f, 0.01f) == false)
		{
		    if(Custom_motor_target_Angle[4] <-1.04f ){Custom_motor_target_Angle[4] += 0.002f;}
			else if(Custom_motor_target_Angle[4] > -1.04f ){Custom_motor_target_Angle[4] -= 0.002f;}
		}



		process = 1;


	}


Custom_motor_target_Angle[5] = 2.0f;


	LZ01.Motor_Pos_Set(Custom_motor_target_Angle[0]); // 设置电机4的电流为0
	LZ02.Motor_Pos_Set(Custom_motor_target_Angle[1]); // 设置电机4的电流为0
	LZ03.Motor_Pos_Set(Custom_motor_target_Angle[2]); // 设置电机4的电流为0
	DM04.Motor_Pos_Set(Custom_motor_target_Angle[3]); // 设置电机4的电流为0
	DM05.Motor_Pos_Set(Custom_motor_target_Angle[4]); // 设置电机4的电流为0
	DM06.Motor_Pos_Set(Custom_motor_target_Angle[5]); // 设置电机4的电流为0





}




void Arm_fall_ready_onEnter_state(StateMachine* fsm, uint8_t prevState)
{


	// if (prevState == 0x02 || prevState == 0x04)
	// {
	//     Custom_motor_target_Angle[0] = 2.85f;
	// 	Custom_motor_target_Angle[1] = 2.25f;
	// 	Custom_motor_target_Angle[2] = 5.0f;
	// 	Custom_motor_target_Angle[3] = 1.0f;
	// 	Custom_motor_target_Angle[4] = 2.05f;
	// 	Custom_motor_target_Angle[5] = 4.15f;

	// }
	// else if(prevState == 0x05)
	// {
	//     Custom_motor_target_Angle[0] = LZ01.get_motordata().Angle;
	// 	Custom_motor_target_Angle[1] = LZ02.get_motordata().Angle;
	// 	Custom_motor_target_Angle[2] = LZ03.get_motordata().Angle;
	// 	Custom_motor_target_Angle[3] = DM04.get_motordata().Angle;
	// 	Custom_motor_target_Angle[4] = DM05.get_motordata().Angle;
	// 	Custom_motor_target_Angle[5] = DM06.get_motordata().Angle;	
	// }
	// else
	// {
	    
	//    	Custom_motor_target_Angle[0] = 2.85f;
	// 	Custom_motor_target_Angle[1] = 1.15f;
	// 	Custom_motor_target_Angle[2] = 3.8f;
	// 	Custom_motor_target_Angle[3] = 1.0f;
	// 	Custom_motor_target_Angle[4] = 2.05f;
	// 	Custom_motor_target_Angle[5] = 4.15f; 


	// }



		process = 0;

		Custom_motor_target_Angle[0] = LZ01.get_motordata().Angle;
		Custom_motor_target_Angle[1] = LZ02.get_motordata().Angle;
		Custom_motor_target_Angle[2] = LZ03.get_motordata().Angle;
		Custom_motor_target_Angle[3] = DM04.get_motordata().Angle;
		Custom_motor_target_Angle[4] = DM05.get_motordata().Angle;
		Custom_motor_target_Angle[5] = DM06.get_motordata().Angle;	

}








void All_stop_onStay_state(StateMachine* fsm, uint32_t elapsedMs)
{
  
	Basis_stop_onStay_state(fsm, elapsedMs);
	Arm_stop_onStay_state(fsm, elapsedMs);


}


void Car_Moving_onStay_state(StateMachine* fsm, uint32_t elapsedMs)
{

    Basis_Move_onStay_state(fsm, elapsedMs);
	Arm_stop_onStay_state(fsm, elapsedMs);
Remote_AnyType_Rx_Data_TypeDef* remote_data_ = Remote_AnyType.GetRemoteData(); // 获取遥控器数据

	if(remote_data_->Chz_Left > 1500)
	{
	    an_d = OPEN;
	}
	else if(remote_data_->Chz_Left < 500)
	{
	    an_d = CLOSE;
	}
}

void Car_arm_rise_ready_onStay_state(StateMachine* fsm, uint32_t elapsedMs)
{


	Basis_Move_onStay_state(fsm, elapsedMs);
    Arm_rise_ready_onStay_state(fsm, elapsedMs);
Remote_AnyType_Rx_Data_TypeDef* remote_data_ = Remote_AnyType.GetRemoteData(); // 获取遥控器数据

	if(remote_data_->Chz_Left > 1500)
	{
	    an_d = OPEN;
	}
	else if(remote_data_->Chz_Left < 500)
	{
	    an_d = CLOSE;
	}

}


void Car_arm_fall_ready_onStay_state(StateMachine* fsm, uint32_t elapsedMs)
{

	Basis_Move_onStay_state(fsm, elapsedMs);
    Arm_fall_ready_onStay_state(fsm, elapsedMs);
	Remote_AnyType_Rx_Data_TypeDef* remote_data_ = Remote_AnyType.GetRemoteData(); // 获取遥控器数据

	if(remote_data_->Chz_Left > 1500)
	{
	    an_d = OPEN;
	}
	else if(remote_data_->Chz_Left < 500)
	{
	    an_d = CLOSE;
	}

}

void Car_custom_control_onStay_state(StateMachine* fsm, uint32_t elapsedMs)
{


	Basis_Move_onStay_state(fsm, elapsedMs);
    Arm_custom_control_onStay_state(fsm, elapsedMs);

Remote_AnyType_Rx_Data_TypeDef* remote_data_ = Remote_AnyType.GetRemoteData(); // 获取遥控器数据

	// if(remote_data_->Chz_Left > 1500)
	// {
	//     an_d = 45;
	// }
	// else if(remote_data_->Chz_Left < 500)
	// {
	//     an_d = 120;
	// }

}







void Only_arm_control_onEnter_state(StateMachine* fsm, uint8_t prevState)
{


	
	    Custom_motor_target_Angle[0] = LZ01.get_motordata().Angle;
		Custom_motor_target_Angle[1] = LZ02.get_motordata().Angle;
		Custom_motor_target_Angle[2] = LZ03.get_motordata().Angle;
		Custom_motor_target_Angle[3] = DM04.get_motordata().Angle;
		Custom_motor_target_Angle[4] = DM05.get_motordata().Angle;
		Custom_motor_target_Angle[5] = DM06.get_motordata().Angle;
	




}



void Only_arm_control_onStay_state(StateMachine* fsm, uint32_t elapsedMs)
{


	Remote_AnyType_Rx_Data_TypeDef* remote_data_ = Remote_AnyType.GetRemoteData(); // 获取遥控器数据

    Custom_motor_target_Angle[0] += remote_data_->Chx_Left  * 0.002;
	Custom_motor_target_Angle[1] += remote_data_->Chy_Left  * 0.002;
	Custom_motor_target_Angle[2] += remote_data_->Chy_Right * 0.002;
	Custom_motor_target_Angle[3] += remote_data_->Chx_Right * 0.002;
	Custom_motor_target_Angle[4] += (remote_data_->Chz_Left - 1024)  * 0.00001;
	if(Custom_motor_target_Angle[4] > 0.3f)
	{
		Custom_motor_target_Angle[4] = 0.3f;
	}
	else if(Custom_motor_target_Angle[4] < -2.5f)
	{
	    Custom_motor_target_Angle[4] = -2.5f;	
	}
Custom_motor_target_Angle[5] = 2.0f;


	LZ01.Motor_Pos_Set(Custom_motor_target_Angle[0]); // 设置电机4的电流为0
	LZ02.Motor_Pos_Set(Custom_motor_target_Angle[1]); // 设置电机4的电流为0
	LZ03.Motor_Pos_Set(Custom_motor_target_Angle[2]); // 设置电机4的电流为0
	DM04.Motor_Pos_Set(Custom_motor_target_Angle[3]); // 设置电机4的电流为0
	DM05.Motor_Pos_Set(Custom_motor_target_Angle[4]); // 设置电机4的电流为0
	DM06.Motor_Pos_Set(Custom_motor_target_Angle[5]); // 设置电机4的电流为0



}





void Only_basis_control_onStay_state(StateMachine* fsm, uint32_t elapsedMs)
{

	Basis_Move_onStay_state(fsm, elapsedMs);
	Remote_AnyType_Rx_Data_TypeDef* remote_data_ = Remote_AnyType.GetRemoteData(); // 获取遥控器数据

	if(remote_data_->Chz_Left > 1500)
	{
	    an_d = OPEN;
	}
	else if(remote_data_->Chz_Left < 500)
	{
	    an_d = CLOSE;
	}

	LZ01.Motor_Pos_Set(Custom_motor_target_Angle[0]); // 设置电机4的电流为0
	LZ02.Motor_Pos_Set(Custom_motor_target_Angle[1]); // 设置电机4的电流为0
	LZ03.Motor_Pos_Set(Custom_motor_target_Angle[2]); // 设置电机4的电流为0
	DM04.Motor_Pos_Set(Custom_motor_target_Angle[3]); // 设置电机4的电流为0
	DM05.Motor_Pos_Set(Custom_motor_target_Angle[4]); // 设置电机4的电流为0
	DM06.Motor_Pos_Set(Custom_motor_target_Angle[5]); // 设置电机4的电流为0



}


//启动定时器PWM通道3


//设置角度函数
//angle:0~180度
void Servo_SetAngle(uint16_t angle)
{
    uint16_t pulse;
    if(angle>180) angle=180;
    //角度映射：0°→500，180°→2500
    pulse = 500 + (angle * 2000)/180;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
//	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, angle);
}







