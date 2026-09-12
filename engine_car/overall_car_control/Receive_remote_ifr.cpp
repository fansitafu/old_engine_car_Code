#include "Receive_remote_ifr.h"



REMOTE_USART_ClassDef Remote_AnyType;

/*
DT7遥控器的波特率是100000，数据格式是9位数据位，1位停止位，无校验位，
*/




void REMOTE_USART_ClassDef::Remote_Init(UART_HandleTypeDef *huart, Remote_Type_enum remote_type)
{


    Remote_AnyType_Data.Type = remote_type;

    ifr_usart_Init(huart, Remote_AnyType_Analysis);

}




Remote_AnyType_Rx_Data_TypeDef*  REMOTE_USART_ClassDef:: GetRemoteData()
{
	return &Remote_AnyType_Data;
}




void Remote_AnyType_Analysis(uint8_t *pData, uint8_t len)
{
    Remote_AnyType_Rx_Data_TypeDef *Remote_data_;
    Remote_data_ = Remote_AnyType.GetRemoteData();	// 获取当前存储的遥控器数据


    switch (Remote_data_->Type)
    {
        case REMOTE_TYPE_DT7:
            DT7_RemoteAndKeyboard_Analysis(pData, len);
            DT7_Remote_To_AnyType_Remote(Remote_data_);

            break;
        case REMOTE_TYPE_VT13:
            VT13_Remote_Analysis(pData, len);
            VT13_Remote_To_AnyType_Remote(Remote_data_);
            break;
        case REMOTE_TYPE_WBUS:
            WBUS_Remote_Analysis(pData, len);
            WBUS_Remote_To_AnyType_Remote(Remote_data_);
            break;
        default:
            // 未知类型，可能需要处理
            break;
    }



    // Custom_Remote_Controller_Analysis(pData, len); // 调用自定义控制器解析函数




}






void DT7_Remote_To_AnyType_Remote(Remote_AnyType_Rx_Data_TypeDef *Remote_data_)
{
    

    
    Remote_data_->Chx_Right = DT7_Remote.Chx_Right;
    if(Remote_data_->Chx_Right < 0.01 && Remote_data_->Chx_Right > -0.01)Remote_data_->Chx_Right = 0;
    Remote_data_->Chy_Right = DT7_Remote.Chy_Right;
    if(Remote_data_->Chy_Right < 0.01 && Remote_data_->Chy_Right > -0.01)Remote_data_->Chy_Right = 0;
    Remote_data_->Chx_Left = DT7_Remote.Chx_Left;
    if(Remote_data_->Chx_Left < 0.01 && Remote_data_->Chx_Left > -0.01)Remote_data_->Chx_Left = 0;
    Remote_data_->Chy_Left = DT7_Remote.Chy_Left;
    if(Remote_data_->Chy_Left < 0.01 && Remote_data_->Chy_Left > -0.01)Remote_data_->Chy_Left = 0;
    Remote_data_->Switch_Right = DT7_Remote.Switch_Right;
    Remote_data_->Switch_Left = DT7_Remote.Switch_Left;
    Remote_data_->Chz_Left = DT7_Remote.Chz_Left;

    Remote_data_->Mouse.X = DT7_Remote.Mouse.X;
    Remote_data_->Mouse.Y = DT7_Remote.Mouse.Y;
    Remote_data_->Mouse.Z = DT7_Remote.Mouse.Z;
    Remote_data_->Mouse.Press_L = DT7_Remote.Mouse.Press_L;
    Remote_data_->Mouse.Press_R = DT7_Remote.Mouse.Press_R;

    Remote_data_->Updata = DT7_Remote.Updata;

    Remote_data_->Last_Updata_Time = HAL_GetTick();
}


void VT13_Remote_To_AnyType_Remote(Remote_AnyType_Rx_Data_TypeDef *Remote_data_)
{

    Remote_data_->Chx_Right = VT13_Remote.Chx_Right;
    Remote_data_->Chy_Right = VT13_Remote.Chy_Right;
    Remote_data_->Chx_Left = VT13_Remote.Chx_Left;
    Remote_data_->Chy_Left = VT13_Remote.Chy_Left;

    Remote_data_->Switch_Right = VT13_Remote.mode_sw;
    Remote_data_->Switch_Left = VT13_Remote.go_home;
    Remote_data_->Switch_Right_extra = VT13_Remote.button;
    Remote_data_->Switch_Left_extra = VT13_Remote.fn;

    Remote_data_->Dial_Left = VT13_Remote.wheel;
    Remote_data_->Dial_Right = VT13_Remote.shutter;


    Remote_data_->Updata = VT13_Remote.Updata;

    Remote_data_->Last_Updata_Time = HAL_GetTick();


}






void WBUS_Remote_To_AnyType_Remote(Remote_AnyType_Rx_Data_TypeDef *Remote_data_)
{
    
    Remote_data_->Chx_Right = WBUS_Remote.Chx_Right;
    Remote_data_->Chy_Right = WBUS_Remote.Chy_Right;
    Remote_data_->Chx_Left = WBUS_Remote.Chx_Left;
    Remote_data_->Chy_Left = WBUS_Remote.Chy_Left;
    Remote_data_->Switch_Right = WBUS_Remote.Switch_C;
    Remote_data_->Switch_Right_extra = WBUS_Remote.Switch_D;
    Remote_data_->Switch_Left = WBUS_Remote.Switch_B;
    Remote_data_->Switch_Left_extra = WBUS_Remote.Switch_A;
    Remote_data_->Dial_Left = WBUS_Remote.Dial_Left;
    Remote_data_->Dial_Right = WBUS_Remote.Dial_Right;


}



    uint16_t data[6];

    float Angle_[6];
    float Radian_[6];
    float Custom_joint_Angle[6];
    float Custom_joint_Radian[6];

    uint8_t LEN;
////////////////////////////  以下为自定义控制器数据解析函数  //////////////////////////////
void Custom_Remote_Controller_Analysis(uint8_t *pData, uint8_t len)
{
    // 在这里实现自定义控制器的数据解析逻辑
    // 解析后的数据可以存储到 Remote_AnyType_Data 中
    LEN = len;
    float sign;
    if(pData[5] == 0x02 && pData[6] == 0x03)
    {
        data[0] = ((uint16_t) pData[8] << 8) | pData[7] ;
        data[1] = ((uint16_t) pData[10] << 8) | pData[9] ;
        data[2] = ((uint16_t) pData[12] << 8) | pData[11] ;
        data[3] = ((uint16_t) pData[14] << 8) | pData[13] ;
        data[4] = ((uint16_t) pData[16] << 8) | pData[15] ;
        data[5] = ((uint16_t) pData[18] << 8) | pData[17] ;
    }

    for (int i = 0; i < 6; i++)//data中
    {
    // 正数第一位:置1负数第一位:置0
    if ((data[i] & 0x8000) == 0x8000)sign = 1;
    else sign = -1;
    data[i] = data[i] & 0x7fff;
    Angle_[i] = sign * data[i] / 100.0f;
    Radian_[i] = sign * data[i] * (3.1415926f / 18000.0f);

    // +180°*100 -> +180°
    // +180°*100 -> +pi

    }
/* 数组1对应轴1,2 to 2，4 to 3, 3 to 4，5 to 5 */

//Custom_joint_Angle是-180°~180°

    Custom_joint_Angle[0] = angle_equal_range_offset(Angle_[1], 130.0f);
    Custom_joint_Angle[1] = angle_equal_range_offset(Angle_[2], 230.0f);
    Custom_joint_Angle[2] = angle_equal_range_offset(-Angle_[4], 205.0f);
    Custom_joint_Angle[3] = angle_equal_range_offset(-Angle_[3], -88.0f);
    Custom_joint_Angle[4] = angle_equal_range_offset(-Angle_[5], -160.0f);
    Custom_joint_Angle[5] = angle_equal_range_offset(Angle_[0], 0.0f);

    Custom_joint_Radian[0] = rad_cycle_offset(Radian_[1], DEGREE_TO_RADIAN(130.0f));
    Custom_joint_Radian[1] = rad_cycle_offset(Radian_[2], DEGREE_TO_RADIAN(230.0f));
    Custom_joint_Radian[2] = rad_cycle_offset(-Radian_[4], DEGREE_TO_RADIAN(205.0f));
    Custom_joint_Radian[3] = rad_cycle_offset(-Radian_[3], DEGREE_TO_RADIAN(-88.0f));
    Custom_joint_Radian[4] = rad_cycle_offset(-Radian_[5], DEGREE_TO_RADIAN(-160.0f));
    Custom_joint_Radian[5] = rad_cycle_offset(Radian_[0], DEGREE_TO_RADIAN(0.0f));



}














