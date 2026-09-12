/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_usart.h
  * Version		: v3.0
  * Author		: LiuHao Lijiawei Albert panjiajun
  * Date			: 2025-11-10
  * Description	:	IFR串口库中的驱动层，用于初始化串口设备和注册解析回调函数、调用中间层来发送接收数据
  *********************************************************************
  */
#ifndef __IFR_USART_H_
#define __IFR_USART_H_
/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#endif
#ifdef __cplusplus
}
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#ifdef HAL_UART_MODULE_ENABLED
#if USE_HAL_UART_REGISTER_CALLBACKS && USE_HAL_USART_REGISTER_CALLBACKS
#include "ifr_usart_queue.h"
#define USART_TX_QUEUE_SIZE 8        	// 发送队列大小
#define USART_RX_BUFFER_SIZE 64				// 双缓存接收数组大小
/**
 * @brief 串口通信类
 */
class IFR_USART_ClassDef {
public:
    // 构造函数
    IFR_USART_ClassDef() : _huart(NULL), rx_AnalysisFunc(NULL), current_rx_buffer_idx(0), last_data_systick(0){}
    // 串口初始化
    void ifr_usart_Init(UART_HandleTypeDef *huart, void(*UART_Analysis_Function)(uint8_t *pData, uint8_t len) = NULL);

    // 发送数据（直接发送）
    HAL_StatusTypeDef ifr_usart_transmit_data(uint8_t *pData, uint8_t len);
    // 发送数据（加入队列）
    bool ifr_usart_entxqueue(const uint8_t *pData, uint8_t len);
    // 处理发送队列
    void ifr_usart_transmit_tx_queue();
		// 处理双缓存接收
		void	ifr_usart_doubleBuffer_recevice(uint16_t len);
    // 错误处理：重启串口
    void ifr_usart_restart();
	
    // 获取状态
    uint16_t ifr_usart_tx_queue_size() { return tx_queue_.size(); }
    bool ifr_usart_tx_queue_is_empty() { return tx_queue_.is_empty(); }
		uint32_t ifr_usart_get_last_systick() { return last_data_systick; }
		
private:
    UART_HandleTypeDef *_huart;                          // UART句柄
    void(*rx_AnalysisFunc)(uint8_t *pData, uint8_t len);   // 解析函数指针
    GenericQueue <USART_TX_QUEUE_SIZE> tx_queue_; 		// 发送队列缓冲区 USART_TX_QUEUE_SIZE个
		uint8_t current_rx_buffer_idx;										// 当前双缓存接收区id
		uint8_t rx_buffer[2][USART_RX_BUFFER_SIZE];    		// 双缓存接收缓冲区 USART_RX_BUFFER_SIZE个
		uint8_t rx_data_len;															// 当前数据长度
    uint32_t last_data_systick;  // 最后更新时间戳

    // 启动DMA接收
    void ifr_usart_Start_DMA_Receive();

};
static uint8_t IFR_Uart_ID_Get(UART_HandleTypeDef *huart);
extern IFR_USART_ClassDef* USART_Pointers[11];

#endif
#endif
#endif // IFR_USART_H
