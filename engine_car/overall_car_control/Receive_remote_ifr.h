#ifndef RECEIVE_REMOTE_IFR_H
#define RECEIVE_REMOTE_IFR_H



#include "ifr_usart.h"
#include "ifr_remote.h"
#include "ifr_usart_queue.h"
#include "Transition_type.h"



typedef enum Remote_Type
{
	REMOTE_TYPE_UNKNOWN = 0,
	REMOTE_TYPE_DT7 ,
	REMOTE_TYPE_VT13,
	REMOTE_TYPE_WBUS,
	
} Remote_Type_enum;











typedef struct
{
	Remote_Type_enum Type;
	float Chx_Left;
	float Chy_Left;
	float Chx_Right;
	float Chy_Right;
	float Chz_Left;				// DT7 遥控器数据
	uint8_t  Switch_Left;
	uint8_t  Switch_Left_extra;		// 额外的左拨杆状态，如果有就是左侧偏左的拨杆
	uint8_t  Switch_Right;
	uint8_t  Switch_Right_extra;	// 额外的右拨杆状态，如果有就是右侧偏右的拨杆
	float Dial_Left;	// 天地飞 左侧旋钮
	float Dial_Right;	// 天地飞 右侧旋钮


	struct
	{
		int16_t X;
		int16_t Y;
		int16_t Z;
		uint8_t Press_L;	//鼠标左键
		uint8_t Press_R;	//鼠标右键
		uint8_t Press_M;	// 额外的鼠标按键状态，如果有就是中键的按键
	}Mouse;


	struct
	{

		float joint_angle[6];	// 自定义控制器关节角度数据
		uint8_t Keyboard_press[9];	// 自定义控制器键盘按键数据

	}Custom_Controller;	// 自定义控制器数据结构体，可以根据需要添加自定义控制器的数据成员

	uint8_t Updata;
	uint32_t Last_Updata_Time;	
	


} Remote_AnyType_Rx_Data_TypeDef;





/**
 * @brief 远程USART类定义，继承自IFR_USART_ClassDef
 * 
 * 这个类实现了远程USART通信功能，主要用于处理遥控器数据的接收和解析。
 */
class REMOTE_USART_ClassDef : public IFR_USART_ClassDef
{


public:
    /**
     * @brief 构造函数
     * 
     * 初始化Remote_AnyType_Data成员变量
     */
//	REMOTE_USART_ClassDef() : Remote_AnyType_Data(){}



	void Remote_Init(UART_HandleTypeDef *huart, Remote_Type_enum remote_type);
	Remote_AnyType_Rx_Data_TypeDef* GetRemoteData();

private:
	

	Remote_AnyType_Rx_Data_TypeDef Remote_AnyType_Data;	// 用于存储解析后的遥控器数据



};













void DT7_Remote_To_AnyType_Remote(Remote_AnyType_Rx_Data_TypeDef *Remote_data_);
void VT13_Remote_To_AnyType_Remote(Remote_AnyType_Rx_Data_TypeDef *Remote_data_);
void WBUS_Remote_To_AnyType_Remote(Remote_AnyType_Rx_Data_TypeDef *Remote_data_);
void Remote_AnyType_Analysis(uint8_t *pData, uint8_t len);

void Custom_Remote_Controller_Analysis(uint8_t *pData, uint8_t len);




extern REMOTE_USART_ClassDef Remote_AnyType;

extern float Custom_joint_Angle[6];
extern float Custom_joint_Radian[6];





















#endif // RECEIVE_REMOTE_IFR_H