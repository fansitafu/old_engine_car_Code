#ifndef IFR_CAN_H
#define IFR_CAN_H
/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_can.cpp
  * Version			: v3.0
  * Author			: LiuHao Lijiawei Albert PanJiajun 
  * Date				: 2025-10-10
  * Description	:	IFR CAN库中的 驱动层，用于初始化CAN设备和注册解析回调函数、注册电机、调用中间层来发送接收数据
	*
  *********************************************************************
  */
#ifdef __cplusplus
 extern "C" {
#endif
#ifdef __cplusplus
 }
#endif
 #include "main.h"
#ifdef HAL_CAN_MODULE_ENABLED
#include "string.h"
#include "ifr_basic_motor.h"
#include "ifr_can_msg_queue.h"  // 引入固定容量队列
#define MAX_REGISTERED_MOTORS 8  // 最大注册电机数
#define MAX_REGISTERED_FUNCS 4 	 // 最大注册自定义协议数
// 定义函数指针类型别名
typedef void (*CustomCanFunc)(CAN_RxHeaderTypeDef rxHeader, uint8_t * rxData);
class IFR_CAN_ClassDef {
public:
    explicit IFR_CAN_ClassDef() : _hcan(NULL), _msgQueue(NULL), _motorCount(0) // 列表化初始化CAN类
		{
			// 初始化电机列表为NULL
			for (uint8_t i = 0; i < MAX_REGISTERED_MOTORS; i++) 
					_motorList[i] = NULL;
    }
    ~IFR_CAN_ClassDef() {}
    // 初始化：绑定CAN硬件和固定容量队列（16）
    void CAN_Init(CAN_HandleTypeDef *hcan, CanMsgQueue* msgQueue);
    // 注册电机（支持所有继承Basic_Motor的电机）
    bool RegisterMotor(IFR_Basic_Motor* motor);
		// 注册自定义消息协议
		bool RegisterCustomFun(CustomCanFunc Custom_Func); // 使用定义的函数指针类型
		// 发送单个消息
		void CAN_Transmit(CAN_TxHeaderTypeDef *pHeader, uint8_t *pData);
    // 批量发送队列中的消息
    void SendQueuedMsgs();
    // 接收消息并分发
    void CAN_Recevice();
    // 获取CAN句柄（供中断回调）
    CAN_HandleTypeDef* GetCanHandle() const { return _hcan; }
		// 获取CAN上的对应位置的电机类对象（如果索引是正确的）
		IFR_Basic_Motor* get_motor(uint8_t motor_index) 
		const {
			if (motor_index < _motorCount)	
				return _motorList[motor_index]; 
			return 0;
		}
		// 使能CAN上的所有电机
		void Motor_All_Enable();
		// 获取当前CAN上注册的电机数量
		uint8_t get_motor_count() const {	return _motorCount;}
private:
    // 禁止拷贝（私有不实现）
    IFR_CAN_ClassDef(const IFR_CAN_ClassDef&);
    IFR_CAN_ClassDef& operator=(const IFR_CAN_ClassDef&);
    // 配置CAN过滤器
    void _ConfigCANFilter();
    CAN_HandleTypeDef* _hcan;                  // CAN硬件句柄
    CanMsgQueue* _msgQueue;                    // 固定容量16的队列指针
    IFR_Basic_Motor* _motorList[MAX_REGISTERED_MOTORS];  // 电机列表
		uint8_t _motorCount;                       // 已注册电机数
		CustomCanFunc _custom_func_list[MAX_REGISTERED_FUNCS];  		// 自定义协议列表
		uint8_t _custom_func_Count;                       // 已注册自定义协议数
    CAN_RxHeaderTypeDef _rxHeader;             // 接收头缓存
    uint8_t _rxData[8];                        // 接收数据缓存
};
// 全局CAN对象指针
extern IFR_CAN_ClassDef* CAN_Pointer[3];
// 中断回调函数
void IFR_CAN_Recevice_Callback(CAN_HandleTypeDef *_hcan);
void IFR_CAN_Transmit_Callback(CAN_HandleTypeDef *_hcan);
// 辅助函数：获取CAN索引
static uint8_t IFR_Can_ID_Get(CAN_HandleTypeDef *hcan);

#endif // IFR_CAN_H
#endif
