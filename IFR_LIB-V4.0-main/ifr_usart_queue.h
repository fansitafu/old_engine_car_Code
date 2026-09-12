/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_usart_queue.h
  * Version		: v1.0
  * Author		: panjiajun
  * Date			: 2025-11-10
  * Description	: IFR串口库中的中间层，用于数据缓冲、应用层和驱动层的解耦
  *********************************************************************
  */
#ifndef __IFR_USART_QUEUE_H_
#define __IFR_USART_QUEUE_H_
#include "main.h"
#include "string.h"
#ifdef __cplusplus
 extern "C" {
#endif
#ifdef __cplusplus
}
#endif
// 缓冲区大小配置

#define USART_QUEUE_BUFFER_SIZE 64 				// 数据缓冲区大小
// 串口消息结构体
typedef struct 
{
	uint8_t data[USART_QUEUE_BUFFER_SIZE];  // 数据缓冲区
	uint16_t length;                      // 数据长度
} UsartMsg;

/**
 * @brief 通用环形队列类（模板版，支持自定义队列元素数量）
 * @tparam MAX_SIZE 队列最大可存储的消息数量
 */
template <uint8_t MAX_SIZE>  // 模板参数：队列最大元素数量
class GenericQueue 
{
public:
    // 构造函数（编译期确定大小，无需动态内存）
    GenericQueue() : front_(0), rear_(0), count_(0) {}

    // 入队：成功返回true，队列满返回false
    bool enqueue(const UsartMsg& item) {
        if (is_full()) {
            return false;
        }
        buffer_[rear_] = item;
        rear_ = (rear_ + 1) % MAX_SIZE;  // 用模板参数MAX_SIZE替代固定宏
        count_++;
        return true;
    }

    // 出队：成功返回true，队列空返回false
    bool dequeue(UsartMsg& item) {
        if (is_empty()) {
            return false;
        }
        item = buffer_[front_];
        front_ = (front_ + 1) % MAX_SIZE;  // 用模板参数MAX_SIZE替代固定宏
        count_--;
        return true;
    }

    // 队列状态查询
    bool is_empty() const { return count_ == 0; }
    bool is_full() const { return count_ == MAX_SIZE; }
    uint8_t size() const { return count_; }
    uint8_t max_size() const { return MAX_SIZE; }  // 返回模板参数指定的最大大小

    // 清空队列
    void clear() {
        front_ = 0;
        rear_ = 0;
        count_ = 0;
    }

private:
    UsartMsg buffer_[MAX_SIZE];  // 队列缓冲区（大小由模板参数决定）
    uint8_t front_;  // 队头索引（待取出的元素）
    uint8_t rear_;   // 队尾索引（待插入的位置）
    uint8_t count_;  // 当前队列中的元素数量
};

#endif
