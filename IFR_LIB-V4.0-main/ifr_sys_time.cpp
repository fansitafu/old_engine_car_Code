/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2025, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  * FileName 		: ifr_sys_time.cpp
  * Version			: v1.0 
  * Author			: PanJiajun
  * Date			: 2025-10-10
  * Description	:	IFR 库中的 系统计时器 （基于中科大的开源）
  *
  *********************************************************************
  */

/**
 * 要求: 使能并绑定一个32位定时器, 开中断, PSC分频到1MHz, ARR为3600000000 - 1（8个0）
 * 保证arr计数器1us增一次, 1h触发一次中断
 */
//以下为示例代码
 // 初始化系统时间
/*
    ifr_sys_timestamp.Init(&htim2);

    // 测试1: 计算程序耗时
    uint64_t start_time = ifr_sys_timestamp.Get_Current_Timestamp();

    // 模拟耗时操作 (100ms)
    for (volatile uint32_t i = 0; i < 100000; i++) {
    __NOP();  // 空操作
    }

    uint64_t elapsed = ifr_sys_timestamp.Get_Current_Timestamp() - start_time;
    printf("耗时: %llu us\n", elapsed); // 实际项目中可移除printf，此处仅演示

    // 测试2: 延时1秒 (通过LED闪烁验证)
    HAL_GPIO_Toggle(LED_PORT, LED_PIN);  // 开始延时前点亮LED
    Delay_Second(1);                    // 延时1秒
    HAL_GPIO_Toggle(LED_PORT, LED_PIN);  // 延时结束后熄灭LED

    // 测试3: 检测当前时间 (微秒级)
    uint64_t current_time = ifr_sys_timestamp.Get_Current_Timestamp();
    printf("当前时间: %llu us\n", current_time); // 实际项目中可移除

    // 测试4: 延时500ms (通过LED闪烁验证)
    HAL_GPIO_Toggle(LED_PORT, LED_PIN);  // 点亮LED
    Delay_Millisecond(500);              // 延时500ms
    HAL_GPIO_Toggle(LED_PORT, LED_PIN);  // 熄灭LED

    // 测试5: 延时250us (微秒级)
    HAL_GPIO_Toggle(LED_PORT, LED_PIN);  // 点亮LED
    Delay_Microsecond(250);              // 延时250us
    HAL_GPIO_Toggle(LED_PORT, LED_PIN);  // 熄灭LED
*/

/* Includes ------------------------------------------------------------------*/

#include "ifr_sys_time.h"
#ifdef HAL_TIM_MODULE_ENABLED//如果底下是虚的说明你没使用任何定时器
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

ifr_Timestamp_ClassDef ifr_sys_timestamp;

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/
/**
 * @brief TIM定时器回调函数, 每3600s调用一次
 *
 */
void TIM_3600s_PeriodElapsedCallback()
{
    uint32_t TIM_Overflow_Count_Temp = ifr_sys_timestamp.Get_Overflow_Count();
    ifr_sys_timestamp.Set_Overflow_Count(TIM_Overflow_Count_Temp + 1);
}

/**
 * @brief 初始化时间戳
 *
 * @param __TIM_Manage_Object 绑定的定时器, psc后为1MHz, arr要求是3 600 000 000, arr至少32位的寄存器
 */
void ifr_Timestamp_ClassDef::Init(TIM_HandleTypeDef *htim)
{
    if (NULL == htim)
    {
        return;
    }
    TIM_Overflow_Count = 0;
    // 初始清零计数器
    IFR_TIM_Handler.TIM_ITStart(htim, TIM_3600s_PeriodElapsedCallback);
    htim->Instance->CNT = 0;
}

/**
 * @brief 计算当前时间戳, 单位微秒
 *
 * @return uint64_t 当前时间戳
 */
uint64_t ifr_Timestamp_ClassDef::Calculate_Timestamp() const
{
    // 当前时间
    uint64_t timestamp;
    // arr计数
    uint32_t arr_counter = IFR_TIM_Handler.Get_TIM_Handle()->Instance->CNT;

    timestamp = (uint64_t)(TIM_Overflow_Count) * 3600000000ULL + (uint64_t) arr_counter;

    return (timestamp);
}

/**
 * @brief 延迟指定秒数
 *
 * @param Second 延迟秒数
 */
void Delay_Second(const uint32_t &Second)
{
    volatile uint64_t start_time = ifr_sys_timestamp.Get_Current_Timestamp();

    while ((uint64_t)(Second) * 1000000ULL + start_time > ifr_sys_timestamp.Get_Current_Timestamp())
    {
    }
}

/**
 * @brief 延迟指定毫秒数
 *
 * @param Millisecond 延迟毫秒数
 */
void Delay_Millisecond(const uint32_t &Millisecond)
{
    volatile uint64_t start_time = ifr_sys_timestamp.Get_Current_Timestamp();

    while ((uint64_t)(Millisecond) * 1000ULL + start_time > ifr_sys_timestamp.Get_Current_Timestamp())
    {
    }
}

/**
 * @brief 延迟指定微秒数
 *
 * @param Microsecond 延迟微秒数
 */
void Delay_Microsecond(const uint32_t &Microsecond)
{
    volatile uint64_t start_time = ifr_sys_timestamp.Get_Current_Timestamp();

    while ((uint64_t)(Microsecond) + start_time > ifr_sys_timestamp.Get_Current_Timestamp())
    {
    }
}
#endif // HAL_TIM_MODULE_ENABLED
