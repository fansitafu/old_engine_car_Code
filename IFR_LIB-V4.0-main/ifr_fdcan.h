#ifndef __IFR_FDCAN_H_
#define __IFR_FDCAN_H_
/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_fdcan.h
  * Version			: v2.0
  * Author			: PanJiajun 
  * Date				: 2025-10-10
  * Description	:	IFR CAN库中的 驱动层，负责CAN通信管理
	*
  *********************************************************************
  */
/* 防止递归包含 */
#ifdef __cplusplus
extern "C" {
#endif
#ifdef __cplusplus
}
#endif
#include "main.h"
#ifdef HAL_FDCAN_MODULE_ENABLED
/* 统一CAN消息格式 */
#include "ifr_can_msg_queue.h"  // 引入消息队列头文件
#include "ifr_basic_motor.h" 
#include "string.h"
#define MAX_REGISTERED_MOTORS 8  // 最大注册电机数量
#define MAX_REGISTERED_FUNCS 4 	 // 最大注册自定义协议数
// 定义函数指针类型别名
typedef void (*CustomCanFunc)(FDCAN_RxHeaderTypeDef rxHeader, uint8_t * rxData);
class IFR_FDCAN_ClassDef
{
public:
    // 构造函数：初始化成员变量
    IFR_FDCAN_ClassDef() : _hfdcan(NULL), _msgQueue(NULL), _motorCount(0) 
		{
        // 初始化电机列表为NULL
        for (uint8_t i = 0; i < MAX_REGISTERED_MOTORS; i++) 
            _motorList[i] = NULL;
    }
    // 析构函数
    ~IFR_FDCAN_ClassDef() {}
    /* 核心功能：初始化（绑定FDCAN硬件和消息队列） */
    void FDCAN_Init(FDCAN_HandleTypeDef *hfdcan, CanMsgQueue* msgQueue);
    /* 核心功能：注册电机（支持所有继承IFR_Basic_Motor的电机） */
    bool RegisterMotor(IFR_Basic_Motor* motor);
		/* 核心功能：注册自定义消息协议（支持所有继承IFR_Basic_Motor的电机） */
		bool RegisterCustomFun(CustomCanFunc Custom_Func); // 使用定义的函数指针类型
    /* 核心功能：发送单个FDCAN消息（满则入队） */
    void FDCAN_Transmit(FDCAN_TxHeaderTypeDef *pHeader, uint8_t *pData);
    /* 核心功能：批量发送队列中的消息 */
    void SendQueuedMsgs();
    /* 核心功能：接收消息并分发给注册电机 */
    void FDCAN_Recevice();
    /* 辅助功能：获取FDCAN句柄（供中断回调） */
    FDCAN_HandleTypeDef* GetFdcanHandle() const { return _hfdcan; }
		// 获取CAN上的对应位置的电机类对象（如果索引是正确的）
		IFR_Basic_Motor* get_motor(uint8_t motor_index) 
		const {
			if (motor_index < _motorCount)	
				return _motorList[motor_index]; 
			return 0;
		}
		// 获取当前CAN上注册的电机数量
		uint8_t get_motor_count() const {	return _motorCount;}
		// 使能CAN上所有电机
		void Motor_All_Enable();
private:
    // 禁止拷贝（私有不实现）
    IFR_FDCAN_ClassDef(const IFR_FDCAN_ClassDef&);
    IFR_FDCAN_ClassDef& operator=(const IFR_FDCAN_ClassDef&);
    /* 私有功能：配置FDCAN过滤器（接收所有消息） */
    void _ConfigFDCANFilter();
    /* 成员变量 */
    FDCAN_HandleTypeDef* _hfdcan;                  // FDCAN硬件句柄
    CanMsgQueue* _msgQueue;                        // 消息队列指针
    IFR_Basic_Motor* _motorList[MAX_REGISTERED_MOTORS];  // 通用电机列表（支持所有Basic_Motor子类）
    uint8_t _motorCount;                           // 已注册电机数量
		CustomCanFunc _custom_func_list[MAX_REGISTERED_FUNCS];  		// 自定义协议列表
		uint8_t _custom_func_Count;                       // 已注册自定义协议数
    FDCAN_RxHeaderTypeDef _rxHeader;               // 接收头缓存
    uint8_t _rxData[8];                            // 接收数据缓存
};
/* 全局FDCAN对象指针（支持多FDCAN外设） */
extern IFR_FDCAN_ClassDef* FDCAN_Pointer[4];
/* 中断回调函数声明*/
void IFR_FDCAN_Recevice_Callback(FDCAN_HandleTypeDef *_hfdcan, uint32_t RxFifo0ITs);
void IFR_FDCAN_Transmit_Callback(FDCAN_HandleTypeDef *_hfdcan);
/* 辅助函数：获取FDCAN索引（区分FDCAN1/FDCAN2/FDCAN3） */
static uint8_t IFR_FDCAN_ID_Get(FDCAN_HandleTypeDef *hfdcan);
#endif  // HAL_FDCAN_MODULE_ENABLED
#endif  // __IFR_FDCAN_H_
