#ifndef __IFR_CRC_H_
#define __IFR_CRC_H_
/**
  **************************** Copyright ******************************
  *
  *       (C) Copyright 2022, China, IFR Laboratory,DreamTeam.
  *                         All Rights Reserved
  *
  *
  * FileName 		: ifr_crc.h
  * Version			: v1.1
  * Author			: Albert YangSengZong panjiajun
  * Date				: 2025-11-10
  * Description	: Compute crc8 and crc16
	*
  *********************************************************************
  */
#include "main.h"
#ifdef __cplusplus
 extern "C" {
#endif
#define IFR_CRC8_INIT 0xFF
#define IFR_CRC16_INIT 0xFFFF
uint8_t IFR_Get_CRC8_Check(uint8_t *pchMessage,uint32_t DataLength,uint8_t crc8_init);
uint16_t IFR_Get_CRC16_Check(uint8_t *pchMessage,uint32_t DataLength,uint16_t crc16_init);
uint8_t verify_crc16_check_sum(uint8_t *p_msg, uint16_t len);
#ifdef __cplusplus
 }
#endif
#endif //#ifndef __IFR_CRC_H_
