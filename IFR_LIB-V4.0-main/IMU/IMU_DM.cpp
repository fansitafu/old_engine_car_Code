#include "IMU_DM.h"
IMU_DM_TypeDef IMU_DM_Info;
IMU_DM_TypeDef IMU_DM_Info_Head;
/**
  * @概述	达妙IMU解析函数
  * @参数1	解析数据包头指针
  * @参数2  数据包长度
  * @返回值 void
  */
void IFR_IMU_DM_Analysis(uint8_t *pData,uint8_t len)
{
	uint8_t i = 0;
//	uint8_t found = 0; // 标记是否找到有效数据包
	uint16_t crc_calc = 0;
	uint16_t crc_recv = 0;
	while (i <= len - 58) 
	{  // 确保剩余长度足够
			// 检查帧头和帧尾
		if (pData[i] == 0x55 && pData[i + 1] == 0xAA && pData[i + 2] == 0x01 && pData[i + 18] == 0x0A)//对帧头，标志号，从机ID，帧尾
		{
			if(pData[i + 3] == 0x01)//找到加速度包
			{
				// 计算 CRC16 校验
				crc_calc = Get_CRC16(&pData[i], 16);
				crc_recv = (uint16_t)(pData[i + 16] | (pData[i + 17] << 8));
				if (crc_calc == crc_recv) 
				{
					memcpy(&IMU_DM_Info.Accel.x, &pData[i + 4], 4);
					memcpy(&IMU_DM_Info.Accel.y, &pData[i + 8], 4);
					memcpy(&IMU_DM_Info.Accel.z, &pData[i + 12], 4);
					IMU_DM_Info.IMU_state = data_ok;
					i += 19;
				} 
				else 
				{
					IMU_DM_Info.IMU_state = data_crc_error;// CRC 错误，继续查找后续数据包
				}
			}
			if(pData[i + 3] == 0x02)//找到角速度包
			{
				// 计算 CRC16 校验
				crc_calc = Get_CRC16(&pData[i], 16);
				crc_recv = (uint16_t)(pData[i + 16] | (pData[i + 17] << 8));
				if (crc_calc == crc_recv) 
				{
					memcpy(&IMU_DM_Info.Gyro.x, &pData[i + 4], 4);
					memcpy(&IMU_DM_Info.Gyro.y, &pData[i + 8], 4);
					memcpy(&IMU_DM_Info.Gyro.z, &pData[i + 12], 4);
					IMU_DM_Info.IMU_state = data_ok;
					i += 19;
				} 
				else 
				{
					IMU_DM_Info.IMU_state = data_crc_error;// CRC 错误，继续查找后续数据包
				}
			}
			if(pData[i + 3] == 0x03)//找到角度包
			{
				// 计算 CRC16 校验
				crc_calc = Get_CRC16(&pData[i], 16);
				crc_recv = (uint16_t)(pData[i + 16] | (pData[i + 17] << 8));
				if (crc_calc == crc_recv) 
				{
					memcpy(&IMU_DM_Info.Angle.roll,  &pData[i + 4],  4);  // Roll
					memcpy(&IMU_DM_Info.Angle.pitch, &pData[i + 8],  4); // Pitch
					memcpy(&IMU_DM_Info.Angle.yaw,   &pData[i + 12], 4); // Yaw
					IMU_DM_Info.IMU_state = data_ok;
					i += 19;
				} 
				else 
				{
					IMU_DM_Info.IMU_state = data_crc_error;// CRC 错误，继续查找后续数据包
				}
			}	
		}
		else
		{
			IMU_DM_Info.IMU_state = data_frame_error;
			i++;
		}
	}
	return; 
}

uint16_t Get_CRC16(uint8_t *ptr, uint16_t len)
{
	 uint16_t crc = 0xFFFF;
	for (size_t i = 0; i < len; ++i)
	{
	uint8_t index = (crc >> 8 ^ ptr[i]);
	crc = ((crc << 1) ^ CRC16_table[index]);
	}
	return crc;
}

//如果你要使用两个DM陀螺仪，你需要现在加上这个
/**
  * @概述	达妙IMU解析函数
  * @参数1	解析数据包头指针
  * @参数2  数据包长度
  * @返回值 void
  */
void IFR_IMU_DM_Analysis_Head(uint8_t *pData,uint8_t len)
{
	uint8_t i = 0;
//	uint8_t found = 0; // 标记是否找到有效数据包
	uint16_t crc_calc = 0;
	uint16_t crc_recv = 0;
	while (i <= len - 58) 
	{  // 确保剩余长度足够
			// 检查帧头和帧尾
		if (pData[i] == 0x55 && pData[i + 1] == 0xAA && pData[i + 2] == 0x01 && pData[i + 18] == 0x0A)//对帧头，标志号，从机ID，帧尾
		{
			if(pData[i + 3] == 0x01)//找到加速度包
			{
				// 计算 CRC16 校验
				crc_calc = Get_CRC16(&pData[i], 16);
				crc_recv = (uint16_t)(pData[i + 16] | (pData[i + 17] << 8));
				if (crc_calc == crc_recv) 
				{
					memcpy(&IMU_DM_Info_Head.Accel.x, &pData[i + 4], 4);
					memcpy(&IMU_DM_Info_Head.Accel.y, &pData[i + 8], 4);
					memcpy(&IMU_DM_Info_Head.Accel.z, &pData[i + 12], 4);
					IMU_DM_Info_Head.IMU_state = data_ok;
					i += 19;
				} 
				else 
				{
					IMU_DM_Info_Head.IMU_state = data_crc_error;// CRC 错误，继续查找后续数据包
				}
			}
			if(pData[i + 3] == 0x02)//找到角速度包
			{
				// 计算 CRC16 校验
				crc_calc = Get_CRC16(&pData[i], 16);
				crc_recv = (uint16_t)(pData[i + 16] | (pData[i + 17] << 8));
				if (crc_calc == crc_recv) 
				{
					memcpy(&IMU_DM_Info_Head.Gyro.x, &pData[i + 4], 4);
					memcpy(&IMU_DM_Info_Head.Gyro.y, &pData[i + 8], 4);
					memcpy(&IMU_DM_Info_Head.Gyro.z, &pData[i + 12], 4);
					IMU_DM_Info_Head.IMU_state = data_ok;
					i += 19;
				} 
				else 
				{
					IMU_DM_Info_Head.IMU_state = data_crc_error;// CRC 错误，继续查找后续数据包
				}
			}
			if(pData[i + 3] == 0x03)//找到角度包
			{
				// 计算 CRC16 校验
				crc_calc = Get_CRC16(&pData[i], 16);
				crc_recv = (uint16_t)(pData[i + 16] | (pData[i + 17] << 8));
				if (crc_calc == crc_recv) 
				{
					memcpy(&IMU_DM_Info_Head.Angle.roll,  &pData[i + 4],  4);  // Roll
					memcpy(&IMU_DM_Info_Head.Angle.pitch, &pData[i + 8],  4); // Pitch
					memcpy(&IMU_DM_Info_Head.Angle.yaw,   &pData[i + 12], 4); // Yaw
					IMU_DM_Info_Head.IMU_state = data_ok;
					i += 19;
				} 
				else 
				{
					IMU_DM_Info_Head.IMU_state = data_crc_error;// CRC 错误，继续查找后续数据包
				}
			}	
		}
		else
		{
			IMU_DM_Info_Head.IMU_state = data_frame_error;
			i++;
		}
	}
	return; 
}












































