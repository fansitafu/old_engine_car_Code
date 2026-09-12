 #include "ifr_fdcan.h"
/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_fdcan.cpp
  * Version			: v2.0
  * Author			: PanJiajun 
  * Date				: 2025-10-10
  * Description	:	IFR CAN库中的 驱动层，用于初始化CAN设备和注册解析回调函数、注册电机、调用中间层来发送接收数据
	*
  *********************************************************************
  */
#ifdef HAL_FDCAN_MODULE_ENABLED
#if USE_HAL_FDCAN_REGISTER_CALLBACKS  // 需使能FDCAN回调注册
/* 全局FDCAN对象指针初始化（支持3个FDCAN外设，可按需调整） */
IFR_FDCAN_ClassDef* FDCAN_Pointer[4] = {NULL};
/*******************************************************************************
* @功能     	: FDCAN初始化函数
* @参数1        : None
* @返回值 		: void
* @概述  		: 用于将硬件层和中间层和驱动层进行绑定，以及FDCAN的过滤器配置等初始化
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_Init(FDCAN_HandleTypeDef *hfdcan, CanMsgQueue* msgQueue)
{
    // 检查参数有效性
    if (hfdcan == NULL || msgQueue == NULL) return;
    // 绑定FDCAN硬件和消息队列
    _hfdcan = hfdcan;
    _msgQueue = msgQueue;
    // 获取当前FDCAN索引（FDCAN1=1, FDCAN2=2, FDCAN3=3）
    uint8_t fdcanIdx = IFR_FDCAN_ID_Get(hfdcan);
    // 避免重复初始化
    if (FDCAN_Pointer[fdcanIdx] == this) return;
    FDCAN_Pointer[fdcanIdx] = this;
    // 配置FDCAN过滤器（接收所有消息）
    _ConfigFDCANFilter();
    // 注册中断回调（发送空闲+接收新消息）
    HAL_FDCAN_RegisterCallback(_hfdcan, HAL_FDCAN_TX_FIFO_EMPTY_CB_ID, IFR_FDCAN_Transmit_Callback);
	//将IFR_FDCAN_Recevice_Callback 注册为接收fifo0中断
	HAL_FDCAN_RegisterRxFifo0Callback(hfdcan, IFR_FDCAN_Recevice_Callback);			
    // 启动FDCAN并使能接收中断
    HAL_FDCAN_ActivateNotification(_hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(_hfdcan);
}
/*******************************************************************************
* @功能     	: 过滤器配置函数（不对外开放）
* @参数1        : None
* @返回值 		: void
* @概述  		:过滤器配置
*******************************************************************************/
void IFR_FDCAN_ClassDef::_ConfigFDCANFilter()
{
    FDCAN_FilterTypeDef sFilterConfig = {0};
    // 过滤器组分配（FDCAN1用0~7，FDCAN2用8~15，FDCAN3用16~23，避免冲突）
    uint8_t fdcanIdx = IFR_FDCAN_ID_Get(_hfdcan);
    sFilterConfig.FilterIndex = (fdcanIdx - 1) * 8;  // 按外设分配连续过滤器组
    // 接收所有消息（32位掩码模式，ID=0x0000，掩码=0x0000）
    sFilterConfig.IdType = FDCAN_STANDARD_ID;       // 支持标准帧
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;    // 掩码过滤模式
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  // 消息入FIFO0
    sFilterConfig.FilterID1 = 0x0000;               // 过滤ID（无限制）
    sFilterConfig.FilterID2 = 0x0000;               // 过滤掩码（无限制）
    // 配置过滤器
    HAL_FDCAN_ConfigFilter(_hfdcan, &sFilterConfig);
    // 全局过滤配置：拒绝远程帧，接受所有数据帧
    HAL_FDCAN_ConfigGlobalFilter(_hfdcan, 
                                 FDCAN_ACCEPT_IN_RX_FIFO0,  // 标准帧不匹配入FIFO0
                                 FDCAN_ACCEPT_IN_RX_FIFO0,  // 扩展帧不匹配入FIFO0
                                 FDCAN_REJECT_REMOTE,        // 拒绝远程帧
                                 FDCAN_REJECT_REMOTE);       // 拒绝扩展远程帧
}
/*******************************************************************************
* @功能     	: 电机注册函数
* @参数1        : 要注册的电机指针
* @返回值 		: bool，是否注册成功
* @概述  		: 用于注册以后，将接收到的CAN数据转向调用电机对应的协议解析函数
*******************************************************************************/
bool IFR_FDCAN_ClassDef::RegisterMotor(IFR_Basic_Motor* motor)
{
    // 检查参数：电机非空 + 未超过最大注册数
    if (motor == NULL || _motorCount >= MAX_REGISTERED_MOTORS)
        return false;
    // 检查是否已注册（避免重复）
    for (uint8_t i = 0; i < _motorCount; i++) 
		{
        if (_motorList[i] == motor)
            return true;  // 已注册，返回成功
    }
    // 新增注册
    _motorList[_motorCount++] = motor;
    return true;
}
/*******************************************************************************
* @功能     	: 自定义消息协议注册函数
* @参数1        : 自定义消息协议解析函数指针
* @返回值 		: void
* @概述  		: 用于在接收数据时可以选择使用自定义的消息协议解析函数
*******************************************************************************/
bool IFR_FDCAN_ClassDef::RegisterCustomFun(CustomCanFunc Custom_Func)
{
    if (Custom_Func == NULL || _custom_func_Count >= MAX_REGISTERED_FUNCS) 
        return false;
    // 检查是否已注册
    for (uint8_t i = 0; i < _custom_func_Count; i++) 
	{
        if (_custom_func_list[i] == Custom_Func) 
			return true;
    }
    _custom_func_list[_custom_func_Count++] = Custom_Func;
    return true;
}
/*******************************************************************************
* @功能     	: 单个CAN消息发送函数
* @参数1        : CAN发送结构体
* @参数2        : 需要发送到CAN线上的数据
* @返回值 		: void
* @概述  		: 用于发送自定义的CAN数据，若CAN空闲邮箱不足则使能邮箱空闲中断
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_Transmit(FDCAN_TxHeaderTypeDef *pHeader, uint8_t *pData)
{
    // 检查FDCAN句柄和参数有效性
    if (_hfdcan == NULL || pHeader == NULL || pData == NULL) 
        return;
    // 发送FIFO未满：直接发送
    if (HAL_FDCAN_GetTxFifoFreeLevel(_hfdcan) > 0) 
	{
        HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, pHeader, pData);
    }
    else if (NULL != _msgQueue)	// 发送FIFO满，检查是否有队列，有则入队
	{
		CanMsg_t msg;
		msg.can_identifier_type = (pHeader->IdType == FDCAN_STANDARD_ID) ? CAN_STD : CAN_EXT;
		msg.id = (msg.can_identifier_type == CAN_STD) ? pHeader->Identifier : pHeader->Identifier;
		memcpy(msg.data, pData, 8);
		_msgQueue->enqueue(msg);
		HAL_FDCAN_ActivateNotification(_hfdcan, FDCAN_IT_TX_FIFO_EMPTY, 0);
	}
	if (NULL == _msgQueue)	// 若队列为空，则关闭邮箱空闲中断
		HAL_FDCAN_DeactivateNotification(_hfdcan, FDCAN_IT_TX_FIFO_EMPTY);
}
/*******************************************************************************
* @功能     	: 队列消息发送函数
* @参数1        : None
* @返回值 		: void
* @概述  		: 发送当前CAN队列中的所有数据，若CAN空闲邮箱不足则使能邮箱空闲中断
*******************************************************************************/
void IFR_FDCAN_ClassDef::SendQueuedMsgs()
{
    // 检查FDCAN句柄和队列有效性，队列空则返回
    if (_hfdcan == NULL || _msgQueue == NULL || _msgQueue->is_empty())
        return;
    __disable_irq();	// 关闭所有中断，创建临界区
    CanMsg_t msg;
    FDCAN_TxHeaderTypeDef txHeader = {0};
    // 循环发送：队列非空 + 发送FIFO未满
    while (_msgQueue->dequeue(msg) && HAL_FDCAN_GetTxFifoFreeLevel(_hfdcan) > 0) 
	{
        // 通用CanMsg转换为FDCAN发送头
        txHeader.IdType = (msg.can_identifier_type == CAN_STD) ? FDCAN_STANDARD_ID : FDCAN_EXTENDED_ID;
        txHeader.Identifier = msg.id;                  // 标准/扩展ID赋值
        txHeader.TxFrameType = FDCAN_DATA_FRAME;       // 数据帧
        txHeader.DataLength = FDCAN_DLC_BYTES_8;       // 固定8字节长度
        txHeader.BitRateSwitch = FDCAN_BRS_OFF;        // 关闭位速率切换（兼容经典CAN）
        txHeader.FDFormat = FDCAN_CLASSIC_CAN;         // 经典CAN格式
        txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;  // 不记录发送事件
        // 发送消息
        HAL_FDCAN_AddMessageToTxFifoQ(_hfdcan, &txHeader, msg.data);
    }
    // 如果退出了循环，但队列非空时，说明当前邮箱满，但还有数据没有发送，使能发送邮箱空闲中断
    if (!_msgQueue->is_empty()) 
        HAL_FDCAN_ActivateNotification(_hfdcan, FDCAN_IT_TX_FIFO_EMPTY, 0);
    else
        HAL_FDCAN_DeactivateNotification(_hfdcan, FDCAN_IT_TX_FIFO_EMPTY);
    __enable_irq();	// 打开所有中断，退出临界区
}
/*******************************************************************************
* @功能     	: 该FDCAN线上所有电机全部使能函数（自动清除错误标志位）
* @参数1        : 需要使能的FDCAN对象
* @返回值 		: void
* @概述  		:	放在主循环中用于自动使能电机并清除故障，重启电机
*******************************************************************************/
void IFR_FDCAN_ClassDef::Motor_All_Enable()
{
	for (int index = 0; index < this->get_motor_count(); index++)
	{
		if (this->get_motor(index)->get_motordata().error_code != 0)
			this->get_motor(index)->Disable(1);
		else if (MOTOR_DISABLE == this->get_motor(index)->get_motordata().motor_state)
			this->get_motor(index)->Enable();
	}
}
/*******************************************************************************
* @功能     		: 接收消息处理函数
* @参数1        : None
* @返回值 			: void
* @概述  				:	用于给中断回调函数调用，将接收到的CAN消息分发给各个注册电机和自定义协议函数
*******************************************************************************/
void IFR_FDCAN_ClassDef::FDCAN_Recevice()
{
    // 检查FDCAN句柄和注册电机数量
    if (NULL == _hfdcan || (0 == _motorList && 0 == _custom_func_list))
        return;
    // 读取FIFO0中的消息（失败则返回）
    if (HAL_FDCAN_GetRxMessage(_hfdcan, FDCAN_RX_FIFO0, &_rxHeader, _rxData) != HAL_OK) 
        return;
    // 转换FDCAN接收消息为通用CanMsg格式
    CanMsg_t recvMsg;
    recvMsg.can_identifier_type = (FDCAN_STANDARD_ID == _rxHeader.IdType) ? CAN_STD : CAN_EXT;
    recvMsg.id = _rxHeader.Identifier;
    memcpy(recvMsg.data, _rxData, 8);
    // 分发给所有注册的电机（调用电机的parse_can_msg）
    for (uint8_t i = 0; i < _motorCount; i++) 
		{
        if (NULL != _motorList[i])
            _motorList[i]->parse_can_msg(recvMsg);
    }
		// 分发消息给注册自定义协议
		for (uint8_t i = 0; i < _custom_func_Count; i++)
		{
			if (NULL != _custom_func_list[i])
				_custom_func_list[i](_rxHeader, _rxData);
		}
}
/*******************************************************************************
* @功能     		: 接收中断回调函数
* @参数1        : None
* @返回值 			: void
* @概述  				:	自定义的中断回调函数（重定向），接收中断触发后自动调用，无需外部调用和修改！！！
*******************************************************************************/
void IFR_FDCAN_Recevice_Callback(FDCAN_HandleTypeDef *_hfdcan, uint32_t RxFifo0ITs)
{
    // 找到对应的FDCAN对象，调用接收处理
    if (NULL != _hfdcan) 
	{
        uint8_t fdcanIdx = IFR_FDCAN_ID_Get(_hfdcan);
        if (NULL != FDCAN_Pointer[fdcanIdx]) 
            FDCAN_Pointer[fdcanIdx]->FDCAN_Recevice();
    }
}
/*******************************************************************************
* @功能     		: 发送中断回调函数
* @参数1        : None
* @返回值 			: void
* @概述  				:	自定义的中断回调函数（重定向），发送中断触发后自动调用，无需外部调用和修改！！！
*******************************************************************************/
void IFR_FDCAN_Transmit_Callback(FDCAN_HandleTypeDef *_hfdcan)
{
    // 找到对应的FDCAN对象，调用队列发送
    if (NULL != _hfdcan) 
		{
        uint8_t fdcanIdx = IFR_FDCAN_ID_Get(_hfdcan);
        if (NULL != FDCAN_Pointer[fdcanIdx])
            FDCAN_Pointer[fdcanIdx]->SendQueuedMsgs();
    }
}
/*******************************************************************************
* @功能     		: 获取CAN实例化对应的CAN硬件层函数（不对外开放）
* @参数1        : None
* @返回值 			: void
* @概述  				:	None
*******************************************************************************/
static uint8_t IFR_FDCAN_ID_Get(FDCAN_HandleTypeDef *hfdcan)
{
    // 区分FDCAN外设（FDCAN1=1, FDCAN2=2, FDCAN3=3，其他=0）
    if (FDCAN1 == hfdcan->Instance) return 1;
    else if (FDCAN2 == hfdcan->Instance) return 2;
    else if (FDCAN3 == hfdcan->Instance) return 3;
    else return 0;
}
#endif  // USE_HAL_FDCAN_REGISTER_CALLBACKS
#endif  // HAL_FDCAN_MODULE_ENABLED
