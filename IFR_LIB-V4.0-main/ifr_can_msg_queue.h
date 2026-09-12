#ifndef CAN_MSG_QUEUE_H
#define CAN_MSG_QUEUE_H
/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_can_msg_queue.h
  * Version			: v1.0
  * Author			: PanJiajun 
  * Date				: 2025-10-10
  * Description	:	IFR CAN库中的 中间层，用于数据缓冲，实现统一发送、驱动层和应用层的解耦
	*
  *********************************************************************
  */
#include "ifr_basic_motor.h"  
#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif
#define CanMsgQueue_count 16
// 固定容量为16的CAN 循环消息队列
class CanMsgQueue {
public:
    // 构造函数：初始化队列（固定容量16）
    CanMsgQueue() : front_(0), rear_(0), count_(0) {}
    // 禁止拷贝（C++03兼容）
    CanMsgQueue(const CanMsgQueue&);
    CanMsgQueue& operator=(const CanMsgQueue&);
    // 入队：成功返回true，队列满返回false
    bool enqueue(const CanMsg_t& msg) 
		{
				for (int i = 0; i < count_; i++)
				{
					// 计算队列中第i个元素的索引（循环队列索引计算）
					int index = (front_ + i) % CanMsgQueue_count;
					if ((buffer_[index].can_identifier_type == CAN_EXT && buffer_[index].id  >> 24 == msg.id >> 24 && (uint8_t)msg.id == (uint8_t)buffer_[index].id) // 检查是否为扩展帧，扩展帧则适用于灵足协议，指示符（前5位）相同，并且电机ID（后8位）相同
						|| (buffer_[index].can_identifier_type == CAN_STD && buffer_[index].id == msg.id))						// 检查是否为标准帧且CANID相同
					{
						buffer_[index] = msg;// 找到相同指示符，覆盖该位置的值
						return true;
					}				
				}
        if (is_full()) 
					Error_Handler();// 如果队列满则进入Error_Handler // 若不想要这个队列满的判断就改成后面这个： return false;
				// 队列未满，在队尾新增元素
				buffer_[rear_] = msg;
        rear_ = (rear_ + 1) % CanMsgQueue_count;  // 固定16取模
        count_++;
        return true;
    }
    // 出队：成功返回true，队列空返回false
    bool dequeue(CanMsg_t& msg) 
		{
			if (is_empty()) 
				return false;
			msg = buffer_[front_];
			front_ = (front_ + 1) % CanMsgQueue_count;  // 固定16取模
			count_--;
			return true;
    }
    // 队列状态查询
    bool is_empty() const { return count_ == 0; }
    bool is_full() const { return count_ == CanMsgQueue_count; }  // 判断数量是否等于16
    uint16_t size() const { return count_; }
    uint16_t max_size() const { return CanMsgQueue_count; }       // 固定返回16
		uint16_t get_front() { return front_; };											// 返回队头索引
		uint16_t get_rear() { return rear_; };												// 返回队尾索引
		uint32_t get_index_id(uint8_t index) {return buffer_[index].id;}			// 返回对应位置的CANid
		uint8_t* get_index_data(uint8_t index) {return buffer_[index].data;}	// 返回对应位置的数据数组指针
		CAN_identifier_type_e get_index_type(uint8_t index) {return buffer_[index].can_identifier_type;}	// 返回对应位置的CAN数据帧类型
    // 清空队列
    void clear() 
		{
			front_ = 0;
			rear_ = 0;
			count_ = 0;
    }
private:
    CanMsg_t buffer_[CanMsgQueue_count];  // 固定容量16的静态数组
    uint16_t front_;     // 队头索引
    uint16_t rear_;      // 队尾索引
    uint16_t count_;     // 当前消息数量
};
#endif // CAN_MSG_QUEUE_H
