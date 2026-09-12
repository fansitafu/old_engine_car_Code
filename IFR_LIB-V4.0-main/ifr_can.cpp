/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_can.cpp
  * Version		: v3.0
  * Author		: LiuHao Lijiawei Albert PanJiajun 
  * Date		: 2025-10-10
  * Description	: IFR CAN库中的 驱动层，用于初始化CAN设备和注册解析回调函数、注册电机、调用中间层来发送接收数据
	*
  *********************************************************************
  */ 
#include "ifr_can.h"
#ifdef HAL_CAN_MODULE_ENABLED	//如果底下是虚的说明你没使用任何CAN口
#if USE_HAL_CAN_REGISTER_CALLBACKS	//如果底下是虚的说明你没使能Register Callback CAN
// 全局CAN指针初始化
IFR_CAN_ClassDef* CAN_Pointer[3] = { NULL };
/*******************************************************************************
* @功能     	: 初始化函数
* @参数1        : None
* @返回值 		: void
* @概述  		: 用于将硬件层和中间层和驱动层进行绑定，以及CAN的过滤器配置等初始化
*******************************************************************************/
void IFR_CAN_ClassDef::CAN_Init(CAN_HandleTypeDef *hcan, CanMsgQueue* msgQueue) 
{
	// 检查参数有效性
	if (NULL == hcan || NULL == msgQueue) 
		return;
	// 绑定CAN硬件和队列（队列固定16容量）
	_hcan = hcan;
	_msgQueue = msgQueue;
	uint8_t canIdx = IFR_Can_ID_Get(hcan);
	// 避免重复初始化
	if (CAN_Pointer[canIdx] == this) return;
	CAN_Pointer[canIdx] = this;
	// 配置过滤器和中断
	_ConfigCANFilter();
	// 注册发送/接收回调
	HAL_CAN_RegisterCallback(_hcan, HAL_CAN_TX_MAILBOX0_COMPLETE_CB_ID, IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(_hcan, HAL_CAN_TX_MAILBOX1_COMPLETE_CB_ID, IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(_hcan, HAL_CAN_TX_MAILBOX2_COMPLETE_CB_ID, IFR_CAN_Transmit_Callback);
	HAL_CAN_RegisterCallback(_hcan, HAL_CAN_RX_FIFO0_MSG_PENDING_CB_ID, IFR_CAN_Recevice_Callback);
	// 启动CAN并使能接收中断
	HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(_hcan);
}
/*******************************************************************************
* @功能     	: 过滤器配置函数（不对外开放）
* @参数1        : None
* @返回值 		: void
* @概述  		: 过滤器配置
*******************************************************************************/
void IFR_CAN_ClassDef::_ConfigCANFilter() 
{
	CAN_FilterTypeDef sFilterConfig = {0};
	// 分配过滤器组（CAN1用0~13，CAN2用14~27）
	if (_hcan->Instance == CAN1) 
	{
		sFilterConfig.FilterBank = 0;
		sFilterConfig.SlaveStartFilterBank = 14;
	} 
	else 
	{
		sFilterConfig.FilterBank = 14;
		sFilterConfig.SlaveStartFilterBank = 14;
	}
	// 接收所有消息（32位掩码模式）
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(_hcan, &sFilterConfig);
}
/*******************************************************************************
* @功能     	: 电机注册函数
* @参数1        : 要注册的电机指针
* @返回值 		: bool，是否注册成功
* @概述  		: 用于注册以后，将接收到的CAN数据转向调用电机对应的协议解析函数
*******************************************************************************/
bool IFR_CAN_ClassDef::RegisterMotor(IFR_Basic_Motor* motor) 
{
    if (NULL == motor || _motorCount >= MAX_REGISTERED_MOTORS) 
        return false;
    // 检查是否已注册
    for (uint8_t i = 0; i < _motorCount; i++) {
        if (_motorList[i] == motor) 
			return true;
    }
    _motorList[_motorCount++] = motor;
    return true;
}
/*******************************************************************************
* @功能     	: 自定义消息协议注册函数
* @参数1        : 自定义消息协议解析函数指针
* @返回值 		: void
* @概述  		: 用于在接收数据时可以选择使用自定义的消息协议解析函数
*******************************************************************************/
bool IFR_CAN_ClassDef::RegisterCustomFun(CustomCanFunc Custom_Func)
{
    if (NULL == Custom_Func || _custom_func_Count >= MAX_REGISTERED_FUNCS) 
        return false;
    // 检查是否已注册
    for (uint8_t i = 0; i < _custom_func_Count; i++) 
	{
        if (_custom_func_list[i] == Custom_Func) 
			return true;
    }
    _custom_func_list[_custom_func_Count] = Custom_Func;
	_custom_func_Count++;
    return true;
}
/*******************************************************************************
* @功能     	: 单个CAN消息发送函数
* @参数1        : CAN发送结构体
* @参数2        : 需要发送到CAN线上的数据
* @返回值 		: void
* @概述  		: 用于发送自定义的CAN数据，若CAN空闲邮箱不足则使能邮箱空闲中断
*******************************************************************************/
void IFR_CAN_ClassDef::CAN_Transmit(CAN_TxHeaderTypeDef *pHeader, uint8_t *pData)
{
	uint32_t TxMailbox;
	if(HAL_CAN_GetTxMailboxesFreeLevel(_hcan) > 0)
	{
		HAL_CAN_AddTxMessage(_hcan, pHeader, pData, &TxMailbox);
	}
	else if (NULL != _msgQueue)	// 检查是否有队列，并且队列未满
	{
		CanMsg_t msg;
		msg.can_identifier_type = (pHeader->IDE == CAN_ID_STD) ? CAN_STD : CAN_EXT;
		msg.id = (msg.can_identifier_type == CAN_STD) ? pHeader->StdId : pHeader->ExtId;
		memcpy(msg.data, pData, 8);
		_msgQueue->enqueue(msg);
		HAL_CAN_ActivateNotification(_hcan, CAN_IT_TX_MAILBOX_EMPTY);
	}
	if (NULL == _msgQueue)	// 若队列为空，则关闭邮箱空闲中断
		HAL_CAN_DeactivateNotification(_hcan, CAN_IT_TX_MAILBOX_EMPTY);
}
/*******************************************************************************
* @功能     	: 队列消息发送函数
* @参数1        : None
* @返回值 		: void
* @概述  		: 发送当前CAN队列中的所有数据，若CAN空闲邮箱不足则使能邮箱空闲中断
*******************************************************************************/
void IFR_CAN_ClassDef::SendQueuedMsgs() 
{
    if (NULL == _hcan || NULL == _msgQueue || _msgQueue->is_empty()) 
        return;
	__disable_irq();	// 关闭所有中断，创建临界区
    CanMsg_t msg;
    uint32_t txMailbox;
    // 循环发送队列消息
    while (_msgQueue->dequeue(msg) && HAL_CAN_GetTxMailboxesFreeLevel(_hcan) > 0) 
	{
		CAN_TxHeaderTypeDef txHeader = {0};
		txHeader.IDE = (msg.can_identifier_type == CAN_STD) ? CAN_ID_STD : CAN_ID_EXT;
		txHeader.StdId = (txHeader.IDE == CAN_ID_STD) ? msg.id : 0;
		txHeader.ExtId = (txHeader.IDE == CAN_ID_EXT) ? msg.id : 0;
		txHeader.RTR = CAN_RTR_DATA;
		txHeader.DLC = 8;
		HAL_CAN_AddTxMessage(_hcan, &txHeader, msg.data, &txMailbox);
    }
    // 如果退出了循环，但队列非空时，说明当前邮箱满，但还有数据没有发送，使能发送邮箱空闲中断
    if (!_msgQueue->is_empty()) 
        HAL_CAN_ActivateNotification(_hcan, CAN_IT_TX_MAILBOX_EMPTY);
	else // 如果队列空了，则关闭发送邮箱空闲中断
	{
		HAL_CAN_DeactivateNotification(_hcan, CAN_IT_TX_MAILBOX_EMPTY);
	}  
	__enable_irq();	// 重启所有中断
}
/*******************************************************************************
* @功能     	: 该CAN线上所有电机全部使能函数（自动清除错误标志位）
* @参数1        : None
* @返回值 		: void
* @概述  		: 放在主控循环中用于自动使能电机并清除故障，重启电机
*******************************************************************************/
void IFR_CAN_ClassDef::Motor_All_Enable()
{
	for (int index = 0; index < this->get_motor_count(); index++)
	{
		if (this->get_motor(index)->get_motordata().error_code != 0)
			this->get_motor(index)->Disable(1);
		else if (this->get_motor(index)->get_motordata().motor_state == MOTOR_DISABLE)
			this->get_motor(index)->Enable();
	}
}
/*******************************************************************************
* @功能     	: 接收消息处理函数
* @参数1        : None
* @返回值 		: void
* @概述  		: 用于给中断回调函数调用，将接收到的CAN消息分发给各个注册电机和自定义协议函数
*******************************************************************************/
void IFR_CAN_ClassDef::CAN_Recevice() 
{
    if (NULL == _hcan || (0 == _motorCount && 0 == _custom_func_Count)) 
		return;
    // 读取CAN消息
    if (HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &_rxHeader, _rxData) != HAL_OK) 
        return;
    // 转换为统一CanMsg格式
    CanMsg_t recvMsg;
    recvMsg.can_identifier_type = (CAN_ID_STD == _rxHeader.IDE) ? CAN_STD : CAN_EXT;
    recvMsg.id = (CAN_STD == recvMsg.can_identifier_type) ? _rxHeader.StdId : _rxHeader.ExtId;
    memcpy(recvMsg.data, _rxData, 8);
    // 分发消息给注册电机
    for (uint8_t i = 0; i < _motorCount; i++) 
	{
		if (NULL != _motorList[i]) 
		{
			_motorList[i]->parse_can_msg(recvMsg);
		}
    }
	// 分发消息给注册自定义协议
	for (uint8_t i = 0; i < _custom_func_Count; i++)
	{
		if (NULL != _custom_func_list[i])
			_custom_func_list[i](_rxHeader, _rxData);
	}
}
/*******************************************************************************
* @功能     	: 接收中断回调函数
* @参数1        : None
* @返回值 		: void
* @概述  		: 自定义的中断回调函数（重定向），接收中断触发后自动调用，无需外部调用和修改！！！
*******************************************************************************/
void IFR_CAN_Recevice_Callback(CAN_HandleTypeDef *_hcan) 
{
	if (NULL == _hcan) 
		return;
	uint8_t canIdx = IFR_Can_ID_Get(_hcan);
	if (NULL != CAN_Pointer[canIdx]) 
		CAN_Pointer[canIdx]->CAN_Recevice();
}
/*******************************************************************************
* @功能     	: 发送中断回调函数
* @参数1        : None
* @返回值 		: void
* @概述  		: 自定义的中断回调函数（重定向），发送中断触发后自动调用，无需外部调用和修改！！！
*******************************************************************************/
void IFR_CAN_Transmit_Callback(CAN_HandleTypeDef *_hcan) 
{
	if (NULL == _hcan) 
		return;
	uint8_t canIdx = IFR_Can_ID_Get(_hcan);
	if (NULL != CAN_Pointer[canIdx]) 
		CAN_Pointer[canIdx]->SendQueuedMsgs();
}
/*******************************************************************************
* @功能     	: 获取CAN实例化对应的CAN硬件层函数（不对外开放）
* @参数1        : None
* @返回值 		: void
* @概述  		: None
*******************************************************************************/
static uint8_t IFR_Can_ID_Get(CAN_HandleTypeDef *hcan) 
{
	if (NULL == hcan) 
		return 0;
	if (CAN1 == hcan->Instance) 
		return 1;
	else if (CAN2 == hcan->Instance) 
		return 2;
	else return 0;
}

#endif
#endif
