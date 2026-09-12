#include "my_judge.h"


robot_status_t robot_status;
power_heat_data_t heat_limit;
Judge_DataTypedef current_all_data;

const unsigned char CRC8_INIT = 0xff; 
const uint16_t CRC16_INIT = 0xffff; 

//电机电流×24就是功率
static uint8_t Judge_Data_TF = 0;//判断crc16校验是否正确
uint32_t my_count_tem_1 = 0;
uint32_t my_count_tem_2 = 0;
uint32_t my_count_tem_3 = 0;
void JudgeData_analysis(uint8_t *pdata, uint8_t len)
{

	uint16_t data_length = 0;
	uint16_t CRC16 = 0;
	uint16_t Cmd_ID = 0;
	int i=0;
	while(pdata[i] != 0xA5)
	{
		i++;
	}

	while(i<len - 5)
	{
		if(IFR_Get_CRC8_Check(&pdata[i],4,CRC8_INIT) == pdata[i+4])
		{
			my_count_tem_1 ++;
			data_length = (uint16_t)((pdata[i + 2] << 8) | pdata[i+1]);
			Cmd_ID = (uint16_t)((pdata[i + 6]<<8) | pdata[i + 5]);
			CRC16 = (uint16_t)((pdata[i + 5 + 2 + data_length + 1]<<8) | pdata[i + 5 + 2 + data_length]);
			if(IFR_Get_CRC16_Check(&pdata[i], 5+2+data_length, CRC16_INIT) == CRC16)
			{
				my_count_tem_2 ++;
				Judge_Data_TF = 1;
				switch(Cmd_ID)
				{
					case robot_state:
						memcpy(&robot_status, &pdata[i+7], data_length);
						break;
					case shooting_heat:
						memcpy(&heat_limit, &pdata[i+7], data_length);
						break;
				}
			}
		}
	}
	data_transmit();
}

void data_transmit(void)
{

	current_all_data.shooter_42mm_barrel_heat = heat_limit.shooter_42mm_barrel_heat;
	current_all_data.shooter_barrel_cooling_value = robot_status.shooter_barrel_cooling_value;
	current_all_data.shooter_barrel_heat_limit = robot_status.shooter_barrel_heat_limit;
	current_all_data.chassis_power_limit = robot_status.chassis_power_limit;
	current_all_data.buffer_energy = heat_limit.buffer_energy;


}

