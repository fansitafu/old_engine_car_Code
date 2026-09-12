#include "car_basis_control.h"


#define DJ_MOTOR_KP 0.25f                               //Kp
#define DJ_MOTOR_KI 0.0f								//Ki
#define DJ_MOTOR_KD 0.01f								//Kd
#define DJ_MOTOR_OUTPUT_MAX 5.0f						//输出最大值
#define DJ_MOTOR_ERROR_MAX 1000.0f						//误差最大值
#define DJ_MOTOR_INTEGRAL_MAX 500.0f					//积分最大值
#define DJ_MOTOR_DEAD_ZONE 5.0f							//死区
#define DJ_MOTOR_INTEGRAL_SEPARATION_THRESHOLD 6.0f		//积分分离阈值


CanMsgQueue        Engine_Basis_CAN1_Queue; // CAN消息队列
IFR_FDCAN_ClassDef Engine_Basis_FDCAN1; // FDCAN对象
IFR_DJI_Speed_Motor DJ_Motor1(0x201, DJI_3508, &Engine_Basis_CAN1_Queue), 
                    DJ_Motor2(0x202, DJI_3508, &Engine_Basis_CAN1_Queue), 
                    DJ_Motor3(0x203, DJI_3508, &Engine_Basis_CAN1_Queue), 
                    DJ_Motor4(0x204, DJI_3508, &Engine_Basis_CAN1_Queue); // 电机对象
Move_Coordinate Basis_Move_Coordinate; // 车辆坐标结构体


StateConfig_Extend Car_Move_State = {NOT_RUNNING, HIGH_PRIORITY,{0x00, NULL, NULL, Basis_Move_onStay_state}}; // 车体状态
StateConfig_Extend Car_Stop_State = {NOT_RUNNING, HIGH_PRIORITY,{0x01, NULL, NULL, Basis_stop_onStay_state}}; // 车体状态

float DJ_Motor1_Speed_Tar = 0.0f; // 电机目标速度
float DJ_Motor2_Speed_Tar = 0.0f; // 电机目标速度
float DJ_Motor3_Speed_Tar = 0.0f; // 电机目标速度
float DJ_Motor4_Speed_Tar = 0.0f; // 电机目标速度





void Init_car_basis(void)
{


	Engine_Basis_FDCAN1.RegisterMotor(&DJ_Motor1); // 注册电机对象到FDCAN控制器
	Engine_Basis_FDCAN1.RegisterMotor(&DJ_Motor2);
	Engine_Basis_FDCAN1.RegisterMotor(&DJ_Motor3);
	Engine_Basis_FDCAN1.RegisterMotor(&DJ_Motor4);
    Engine_Basis_FDCAN1.FDCAN_Init(&hfdcan1, &Engine_Basis_CAN1_Queue); // 初始化FDCAN1，绑定消息队列

	DJ_Motor1.Speed_PID.PID_Init(DJ_MOTOR_KP, DJ_MOTOR_KI, DJ_MOTOR_KD, DJ_MOTOR_OUTPUT_MAX, DJ_MOTOR_ERROR_MAX, DJ_MOTOR_INTEGRAL_MAX, DJ_MOTOR_DEAD_ZONE, DJ_MOTOR_INTEGRAL_SEPARATION_THRESHOLD, 0.3f);
	DJ_Motor2.Speed_PID.PID_Init(DJ_MOTOR_KP, DJ_MOTOR_KI, DJ_MOTOR_KD, DJ_MOTOR_OUTPUT_MAX, DJ_MOTOR_ERROR_MAX, DJ_MOTOR_INTEGRAL_MAX, DJ_MOTOR_DEAD_ZONE, DJ_MOTOR_INTEGRAL_SEPARATION_THRESHOLD, 0.3f);
	DJ_Motor3.Speed_PID.PID_Init(DJ_MOTOR_KP, DJ_MOTOR_KI, DJ_MOTOR_KD, DJ_MOTOR_OUTPUT_MAX, DJ_MOTOR_ERROR_MAX, DJ_MOTOR_INTEGRAL_MAX, DJ_MOTOR_DEAD_ZONE, DJ_MOTOR_INTEGRAL_SEPARATION_THRESHOLD, 0.3f);
	DJ_Motor4.Speed_PID.PID_Init(DJ_MOTOR_KP, DJ_MOTOR_KI, DJ_MOTOR_KD, DJ_MOTOR_OUTPUT_MAX, DJ_MOTOR_ERROR_MAX, DJ_MOTOR_INTEGRAL_MAX, DJ_MOTOR_DEAD_ZONE, DJ_MOTOR_INTEGRAL_SEPARATION_THRESHOLD, 0.3f);
	
	Direction_angle_PID.PID_Init(5.0f, 0.01f, 0.01f, 1000.0f, 1000.0f, 500.0f, 3.0f, 6.0f, 0.3f);

	Direction_angle_tar = IMU_Info.Angle.Yaw;

	Car_StateMachine.registerState_extend(&Car_Move_State); // 注册状态机状态1
	Car_StateMachine.registerState_extend(&Car_Stop_State); // 注册状态机状态2

}




void Basis_Move_onStay_state(StateMachine* fsm, uint32_t elapsedMs)
{
    
	Basis_Motor_Date_Updating();
/*

	试一下目标速度更新的频率和电机电流更新的频率的关系，看看需不需要将目标速度更新的频率和电机电流更新的频率一致

*/


    DJ_Motor1.Motor_Speed_Set(DJ_Motor1_Speed_Tar); // 设置电机1的目标速度为0
    DJ_Motor2.Motor_Speed_Set(DJ_Motor2_Speed_Tar); // 设置电机2的目标速度为0
    DJ_Motor3.Motor_Speed_Set(DJ_Motor3_Speed_Tar); // 设置电机3的目标速度为0
    DJ_Motor4.Motor_Speed_Set(DJ_Motor4_Speed_Tar); // 设置电机4的目标速度为0
	

}





void Basis_stop_onStay_state(StateMachine* fsm, uint32_t elapsedMs)
{

	// 在停止模式下，将所有电机的电流设置为0
	DJ_Motor1.set_electric(0); // 设置电机1的电流为0
	DJ_Motor2.set_electric(0); // 设置电机2的电流为0
	DJ_Motor3.set_electric(0); // 设置电机3的电流为0
	DJ_Motor4.set_electric(0); // 设置电机4的电流为0

}








void Basis_Motor_Date_Updating()
{
	
	Remote_AnyType_Rx_Data_TypeDef* remote_data_ = Remote_AnyType.GetRemoteData(); // 获取遥控器数据

    Basis_Move_Coordinate.dir_x = remote_data_->Chx_Left  * REMOTE_TO_SPEED; // 从遥控器输入获取车辆前后方向
    Basis_Move_Coordinate.dir_y = remote_data_->Chy_Left  * REMOTE_TO_SPEED; // 
	Basis_Move_Coordinate.yaw   = remote_data_->Chx_Right * REMOTE_TO_SPEED ;
    // Half_judge();
    DJ_Motor1_Speed_Tar = - Basis_Move_Coordinate.dir_x - Basis_Move_Coordinate.dir_y + Basis_Move_Coordinate.yaw; // 根据遥控器输入计算电机1的目标速度
    DJ_Motor2_Speed_Tar = - Basis_Move_Coordinate.dir_x + Basis_Move_Coordinate.dir_y + Basis_Move_Coordinate.yaw;
    DJ_Motor3_Speed_Tar =   Basis_Move_Coordinate.dir_x - Basis_Move_Coordinate.dir_y + Basis_Move_Coordinate.yaw;
    DJ_Motor4_Speed_Tar =   Basis_Move_Coordinate.dir_x + Basis_Move_Coordinate.dir_y + Basis_Move_Coordinate.yaw;


}




float Direction_angle_tar;
float an_tar_;
IFR_PID Direction_angle_PID;

void Half_judge(void)
{
	Remote_AnyType_Rx_Data_TypeDef* remote_data_ = Remote_AnyType.GetRemoteData(); // 获取遥控器数据
    Direction_angle_tar += remote_data_->Chx_Right * 0.4f;


	while(Direction_angle_tar > 180.0f)
	{
		Direction_angle_tar -= 360.0f;
	}
	while(Direction_angle_tar < -180.0f)
	{
	    Direction_angle_tar += 360.0f;
	}


	if(Direction_angle_tar > IMU_Info.Angle.Yaw)
	{

		if(Direction_angle_tar - IMU_Info.Angle.Yaw > 180.0f)
		{
		    Direction_angle_tar -= 360.0f;
		}

	}
	else
	{
		if(Direction_angle_tar - IMU_Info.Angle.Yaw < -180.0f)
		{
		    Direction_angle_tar += 360.0f;
		}
	}

	an_tar_ = Direction_angle_tar;

	Basis_Move_Coordinate.yaw = Direction_angle_PID.Positional_PID(Direction_angle_tar, IMU_Info.Angle.Yaw);


}




