/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 	: ifr_gpio.h
  * Version		: v2.1
  * Author		: LiuHao Lijiawei Albert
  * Date		: 2022-09-27
  * Description	:
  *********************************************************************
  */
#ifndef __IFR_GPIO_H_
#define __IFR_GPIO_H_
/* Define to prevent recursive inclusion -------------------------------------*/
#ifdef __cplusplus
 extern "C" {
#ifdef __cplusplus
}
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */
class IFR_GPIO_ClassDef
{
	public:
		IFR_GPIO_ClassDef(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
		void High(void);
		void Low(void);
		void Turn(void);
		GPIO_PinState Read(void);
	private:
		GPIO_TypeDef* GPIOX;
		uint16_t GPIO_PIN;
		GPIO_PinState PinState;
};
// 快速设定GPIO的高低电平
#define H  1
#define L  0
// 极简GPIO控制宏
#define IFR_GPIO_SetPin(GPIO_name, GPIO_PIN_name, H_L)  \
  if ((H_L) == H) {                                     \
    HAL_GPIO_WritePin(GPIO_##GPIO_name, GPIO_PIN_##GPIO_PIN_name, GPIO_PIN_SET); \
  } else {                                              \
    HAL_GPIO_WritePin(GPIO_##GPIO_name, GPIO_PIN_##GPIO_PIN_name, GPIO_PIN_RESET); \
  }                                                     \

// 快速反转GPIO的高低电平
#define IFR_GPIO_TogglePin(GPIO_name, GPIO_PIN_name)	\
  HAL_GPIO_TogglePin(GPIO_##GPIO_name, GPIO_PIN_##GPIO_PIN_name)

// 快速读取GPIO的高低电平
#define IFR_GPIO_ReadPin(GPIO_name, GPIO_PIN_name)	\
  HAL_GPIO_ReadPin(GPIO_##GPIO_name, GPIO_PIN_##GPIO_PIN_name)
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */
/* USER CODE BEGIN Prototypes */
/* USER CODE END Prototypes */
#endif
#endif
