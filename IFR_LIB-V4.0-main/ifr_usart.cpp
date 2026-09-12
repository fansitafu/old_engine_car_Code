/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_usart.cpp
  * Version		: v3.1
  * Author		: LiuHao Lijiawei Albert panjiajun
  * Date			: 2025-11-10
  * Description	:	IFR串口库中的驱动层，用于初始化串口设备和注册解析回调函数、调用中间层来发送接收数据
  *********************************************************************
  */
/* Includes ---------------------------------------------------------*/
#include "ifr_usart.h"
/* Private variables -------------------------------------------------*/
#if USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
#ifdef HAL_UART_MODULE_ENABLED

void IFR_UART_Tx_Callback(UART_HandleTypeDef *huart);
void IFR_UART_Rx_Callback(UART_HandleTypeDef *huart, uint16_t len);

// 串口类指针数组，用于定向查找解析函数
IFR_USART_ClassDef* USART_Pointers[11] = {NULL};
/**
  * @概述	串口初始化
  * @参数1	串口句柄
  * @参数2	解析函数指针
  * @返回值 void
  */
void IFR_USART_ClassDef::ifr_usart_Init(UART_HandleTypeDef *huart, void(*UART_Analysis_Function)(uint8_t *pData, uint8_t len)) 
{
    _huart = huart;
    uint8_t uart_id = IFR_Uart_ID_Get(huart);
    USART_Pointers[uart_id] = this;
    if (UART_Analysis_Function != NULL)
        rx_AnalysisFunc = UART_Analysis_Function;
    // 注册接收回调函数
    HAL_UART_RegisterRxEventCallback(huart, IFR_UART_Rx_Callback);
    HAL_UART_RegisterCallback(huart, HAL_UART_TX_COMPLETE_CB_ID, IFR_UART_Tx_Callback);
		// 清空双缓冲
    memset(this->rx_buffer[0], 0, USART_RX_BUFFER_SIZE);
    memset(this->rx_buffer[1], 0, USART_RX_BUFFER_SIZE);;
    // 启动DMA接收
    ifr_usart_Start_DMA_Receive();
}
/**
  * @概述	启动DMA接收
  * @返回值 void
  */
void IFR_USART_ClassDef::ifr_usart_Start_DMA_Receive() 
{
    // 启动DMA空闲接收
    HAL_UARTEx_ReceiveToIdle_DMA(_huart, rx_buffer[current_rx_buffer_idx], USART_RX_BUFFER_SIZE);
    // 禁用DMA半传输中断
    __HAL_DMA_DISABLE_IT(_huart->hdmarx, DMA_IT_HT);
}
/**
  * @概述	直接发送数据
  * @参数1	数据指针
  * @参数2	数据长度
  * @返回值 HAL状态
  */
HAL_StatusTypeDef IFR_USART_ClassDef::ifr_usart_transmit_data(uint8_t *pData, uint8_t len) 
{
    if (_huart == NULL) 
			return HAL_ERROR;
    return HAL_UART_Transmit_DMA(_huart, pData, len);
}
/**
  * @概述	将数据加入发送队列
  * @参数1	数据指针
  * @参数2	数据长度
  * @返回值 成功返回true，失败返回false
  */
bool IFR_USART_ClassDef::ifr_usart_entxqueue(const uint8_t *pData, uint8_t len) 
{
    if (_huart == NULL || len == 0 || len > USART_QUEUE_BUFFER_SIZE) 
        return false;
    // 如果队列已满，返回失败
    if (tx_queue_.is_full()) 
        return false;
    // 创建消息并加入队列
    UsartMsg msg;
    memcpy(msg.data, pData, len);
    msg.length = len;
    return tx_queue_.enqueue(msg);
}
/**
  * @概述	处理发送队列，将队列中的数据发送出去
  * @返回值 void
  */
void IFR_USART_ClassDef::ifr_usart_transmit_tx_queue() 
{
    if (_huart == NULL || tx_queue_.is_empty())
		{
			__HAL_DMA_DISABLE_IT(_huart->hdmatx, UART_IT_TC);	// 失能发送完全中断
      return;
		}
    // 检查UART是否空闲
    if (_huart->gState == HAL_UART_STATE_READY)
		{
        UsartMsg msg;
        if (tx_queue_.dequeue(msg)) 
            ifr_usart_transmit_data(msg.data, msg.length);
    }
		else 
			__HAL_UART_ENABLE_IT(_huart, UART_IT_TC);	// 使能发送完全中断,等发送完成以后再进入此函数发送
}

/**
  * @概述	接收数据双缓冲处理
  * @参数1	接收数据长度
  * @返回值 void
  */
void IFR_USART_ClassDef::ifr_usart_doubleBuffer_recevice(uint16_t len) 
{
	if (len <= 0 || len > USART_RX_BUFFER_SIZE)	// 当数据过长或者过短，直接放弃
		return;
	uint8_t processed_buf_idx = this->current_rx_buffer_idx;  // 刚接收完的缓冲区索引
	this->current_rx_buffer_idx = !this->current_rx_buffer_idx;  // 切换为另一个缓冲区
	this->rx_data_len = len;  // 记录本次接收长度
	
	// 2. 清空当前缓冲区未使用的部分（避免脏数据）
	memset(&this->rx_buffer[processed_buf_idx][len], 0, USART_RX_BUFFER_SIZE - len);
	
	// 3. 调用解析函数处理已接收的数据
	if (this->rx_AnalysisFunc != NULL)
			this->rx_AnalysisFunc(this->rx_buffer[processed_buf_idx], len);
	
	// 4. 重启DMA接收（使用新的缓冲区）
	ifr_usart_Start_DMA_Receive();
	
	// 5. 更新最后接收时间戳
	this->last_data_systick = HAL_GetTick();
}

/**
  * @概述	串口错误处理，重启串口
  * @返回值 void
  */
void IFR_USART_ClassDef::ifr_usart_restart() 
{
    if (_huart == NULL) return;
    // 清除错误标志
    _huart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UNLOCK(_huart);
    // 重新初始化底层硬件
    HAL_UART_MspDeInit(_huart);
    HAL_UART_MspInit(_huart);
    // 重新注册回调函数并启动接收
    HAL_UART_RegisterRxEventCallback(_huart, IFR_UART_Rx_Callback);
		HAL_UART_RegisterCallback(_huart, HAL_UART_TX_COMPLETE_CB_ID, IFR_UART_Tx_Callback);
    ifr_usart_Start_DMA_Receive();
}
/**
  * @概述	串口接收回调函数（无需外部调用！！！）
  * @参数1	串口句柄
  * @参数2	接收数据长度
  * @返回值 void
  */
void IFR_UART_Rx_Callback(UART_HandleTypeDef *huart, uint16_t len) 
{
	uint8_t uart_id = IFR_Uart_ID_Get(huart);
	if (USART_Pointers[uart_id] != NULL)
			USART_Pointers[uart_id]->ifr_usart_doubleBuffer_recevice(len);
}
/**
  * @概述	串口发送完成回调函数（无需外部调用！！！）
  * @参数1	串口句柄
  * @返回值 void
  */
void IFR_UART_Tx_Callback(UART_HandleTypeDef *huart)
{
	uint8_t uart_id = IFR_Uart_ID_Get(huart);
	if (USART_Pointers[uart_id] != NULL)
			USART_Pointers[uart_id]->ifr_usart_transmit_tx_queue();
}
/**
  * @概述	UART错误回调函数（自动重启串口）（无需外部调用！！！）
  * @参数1	串口句柄
  * @返回值 void
  */
uint32_t UART_Error_Count[11] = {0}; // 错误计数
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) 
{
	uint8_t uart_id = IFR_Uart_ID_Get(huart);
	if (USART_Pointers[uart_id] != NULL) 
	{
			USART_Pointers[uart_id]->ifr_usart_restart();
			UART_Error_Count[uart_id]++;
	}
}

/**
  * @概述	获取串口ID
  * @参数1	串口句柄
  * @返回值 串口ID
  */
static uint8_t IFR_Uart_ID_Get(UART_HandleTypeDef *huart) 
{
	#ifdef USART1
	if (huart->Instance == USART1)       return 1;
	#endif

	#ifdef USART2
	if (huart->Instance == USART2)       return 2;
	#endif

	#ifdef USART3
	if (huart->Instance == USART3)       return 3;
	#endif
	
	#ifdef USART4
	if (huart->Instance == USART4)       return 4;
	#endif

	#ifdef USART5
	if (huart->Instance == USART5)       return 5;
	#endif

	#ifdef USART6
	if (huart->Instance == USART6)       return 6;
	#endif
	
	#ifdef USART7
	if (huart->Instance == USART7)       return 7;
	#endif

	#ifdef USART8
	if (huart->Instance == USART8)       return 8;
	#endif

	#ifdef USART9
	if (huart->Instance == USART9)       return 9;
	#endif
	
	#ifdef USART10
	if (huart->Instance == USART10)       return 10;
	#endif
	
	#ifdef UART1
	if (huart->Instance == UART1)        return 1;
	#endif

	#ifdef UART2
	if (huart->Instance == UART2)        return 2;
	#endif

	#ifdef UART3
	if (huart->Instance == UART3)       return 3;
	#endif

	#ifdef UART4
	if (huart->Instance == UART4)        return 4;
	#endif

	#ifdef UART5
	if (huart->Instance == UART5)        return 5;
	#endif

	#ifdef UART6
	if (huart->Instance == UART6)        return 6;
	#endif

	#ifdef UART7
	if (huart->Instance == UART7)       return 7;
	#endif
	
	#ifdef UART8
	if (huart->Instance == UART8)        return 8;
	#endif

	#ifdef UART9
	if (huart->Instance == UART9)        return 9;
	#endif

	#ifdef UART10
	if (huart->Instance == UART10)       return 10;
	#endif


	return 0;
}
#endif // HAL_UART_MODULE_ENABLED
#endif // USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
