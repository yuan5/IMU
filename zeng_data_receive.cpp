#include "zeng_data_receive.h"

#include <windows.h>
#include <winnt.h>
#include <stdio.h>

#include <iostream>
#include <fstream>
#include <math.h>
#include <stdbool.h>

#include "qdebug.h"

using namespace std;

//#define SAVE_DATA

#ifdef SAVE_DATA
ofstream data_save;
#endif

#ifdef SAVE_CAL_DATA
ofstream data_save;
#endif

HANDLE receive_handle = NULL;
HANDLE hComm = NULL;

bool save_flag = false;
const unsigned char Header[2] = {'H','D'};
UI_DISPLAY_DATA dis_data;

UINT16 angle = 0;
//the sign "L" transform char to long
//LPCWSTR Com_name[10] = {L"com0",
//						L"com1",
//						L"com2",
//						L"com3",
//						L"com4",
//						L"com5",
//						L"com6",
//						L"com7",
//						L"com8",
//						L"com9"};

typedef struct IMU_SAVE_DATA_TYPE_{
	float gyro[3];
	float acc[3];
	float mag[3];
	float euler[3];
	float temp;
}IMU_SAVE_DATA_TYPE;

INT16 Zeng_input_usart_init(char *port_name, UINT32 baudrate)
{
	INT16 ret = 0;
	DCB dcb;

	WCHAR uart_name[64];
	int com_number = 0;
	sscanf(port_name, "COM%d", &com_number);
	if(com_number >= 10)
	{
		swprintf(uart_name, L"\\\\.\\%s", port_name);
	}else{
		swprintf(uart_name, L"%s", port_name);
	}

	hComm = CreateFile(uart_name/*TEXT("com5")*/, \
		GENERIC_READ | GENERIC_WRITE, \
		0, \
		NULL, \
		OPEN_EXISTING, \
		FILE_ATTRIBUTE_NORMAL/*|FILE_FLAG_OVERLAPPED*/, \
		NULL);
	if (INVALID_HANDLE_VALUE == hComm)
	{
		ret = -1;
	} else {
		COMMTIMEOUTS TimeOuts;
		TimeOuts.ReadIntervalTimeout = 0;
		TimeOuts.ReadTotalTimeoutMultiplier = 1;
		TimeOuts.ReadTotalTimeoutConstant = 1;
		TimeOuts.WriteTotalTimeoutMultiplier = 500;
		TimeOuts.WriteTotalTimeoutConstant = 2000;
		SetCommTimeouts(hComm, &TimeOuts);

		GetCommState(hComm, &dcb);
		dcb.BaudRate = baudrate;
		dcb.ByteSize = 8;
		dcb.Parity = NOPARITY;
		dcb.StopBits = ONESTOPBIT;

		SetCommState(hComm, &dcb);

		PurgeComm(hComm, PURGE_TXCLEAR|PURGE_RXCLEAR);

#ifdef SAVE_DATA
		data_save.open("data/data.csv");
#endif
#ifdef SAVE_CAL_DATA
		data_save.open("data/data.csv");
#endif
	}

	return ret;
}

int Zeng_uart_read_data(UINT16 cnt, char *buff)
{
	int ret = 0;
	DWORD read_cnt;

	ReadFile(hComm, buff, cnt, &read_cnt, NULL);
	cnt -= read_cnt;
	while(cnt>0)
	{
		ReadFile(hComm, buff, cnt, &read_cnt, NULL);
		cnt -= read_cnt;
	}
	ret = read_cnt;
	return ret;
}

int Zeng_uart_write_data(UINT16 cnt, char *buf)
{
	DWORD write_cnt;
	WriteFile(hComm, buf, cnt, &write_cnt, NULL);
	if(write_cnt > 0)
	{
		if(write_cnt == cnt)
		{
			write_cnt = 0;
		}
	}else{
		write_cnt = -1;
	}
	return write_cnt;
}

int Zeng_get_data_header(unsigned char *buf)
{
	int i=0;
	int ret = 0;
	int time_out = 60;
	while((i<2)&&((time_out--) >0))
	{
		Zeng_uart_read_data(1, (char*)&buf[i]);
		{
			if(Header[i] == buf[i])
			{
				i++;
			}else{
				i=0;
			}
		}
	}
	if(time_out <= 0)
	{
		ret = 1;
	}else{
		//get length
		Zeng_uart_read_data(2, (char*)&buf[2]);
	}
	return ret;
}

uint8_t Zeng_cal_CheckSum(uint8_t *start, uint32_t len)
{
	uint8_t sum = 0;
	uint8_t i = 0;
	for(i=0; i<len; i++)
	{
		sum += start[i];
	}
	return sum;
}

uint16_t Zeng_cal_CheckSum_uint16(uint16_t *start, uint8_t len)
{
	uint16_t sum = 0;
	uint8_t i = 0;
	for(i=0; i<len; i++)
	{
		sum += start[i];
	}
	return sum;
}


//float save_angle[256] = {0};
//uint8_t save_thumb = 0;
float real_angle = 0.0f;
extern ofstream save_data;
//bool save_flag = false;
float rad_deg_sw = 1.0f;
IMU_SAVE_DATA_TYPE stream_save_data;
DWORD WINAPI Zeng_data_receive_task(LPVOID lpParam)
{
	//uint32_t cnt = 0;
	unsigned char read_buff[256];
	TRANS_HEADER *header_ptr = (TRANS_HEADER*)read_buff;

	TRANS_EULER_FLOAT *euler_ptr = NULL;
	TRANS_3D_FLOAT   *trix_ptr = NULL;
	TRANS_QUATERN_FLOAT *qua_ptr = NULL;
	TRANS_ROTATION_FLOAT *rot_ptr = NULL;

	int16_t data_len = 0;
	uint8_t sub_len = 0;
	uint8_t data_start = 0;
	uint8_t data_type = 0;

	uint16_t Check_sum = 0;
#ifdef SAVE_CAL_DATA
	float sum_angle = 0.0f;
#endif

//	lpParam = lpParam;
    Q_UNUSED(lpParam);

	while(1)
	{
		if(Zeng_get_data_header(read_buff) != 0)
		{
			Sleep(1);
		}else{
			data_len = header_ptr->data_len;
			Zeng_uart_read_data((uint16_t)data_len, (char*)&read_buff[4]);

			Check_sum = Zeng_cal_CheckSum_uint16((uint16_t*)&read_buff[4], ((data_len-2)>>1));
			if(*((uint16_t*)&read_buff[data_len+2]) == Check_sum)
			{
				dis_data.data_type = 0;
				data_start = 4;
				dis_data.update_flag = true;
				data_len -= 2;

				while(data_len>0)
				{
					sub_len = read_buff[data_start++];
					data_type = read_buff[data_start++];
					data_len -= (sub_len+2);
					dis_data.data_type |= (0x01<<data_type);
					switch(data_type){
					case GYRO_FLOAT:
						trix_ptr = (TRANS_3D_FLOAT*)&read_buff[data_start];
						dis_data.gyro[0] = trix_ptr->val[0]*rad_deg_sw;
						dis_data.gyro[1] = trix_ptr->val[1]*rad_deg_sw;
						dis_data.gyro[2] = trix_ptr->val[2]*rad_deg_sw;
						//stream_save_data
						memcpy((void*)stream_save_data.gyro, (void*)trix_ptr->val, 3*sizeof(float));

						break;
					case ACCEL_FLOAT:
						trix_ptr = (TRANS_3D_FLOAT*)&read_buff[data_start];
						dis_data.acc[0] = trix_ptr->val[0];
						dis_data.acc[1] = trix_ptr->val[1];
						dis_data.acc[2] = trix_ptr->val[2];
						dis_data.acceleration = sqrt(dis_data.acc[0]*dis_data.acc[0]+dis_data.acc[1]*dis_data.acc[1]+dis_data.acc[2]*dis_data.acc[2]);
						memcpy((void*)stream_save_data.acc, (void*)trix_ptr->val, 3*sizeof(float));
						break;
					case MAG_FLAOT:
						trix_ptr = (TRANS_3D_FLOAT*)&read_buff[data_start];
						dis_data.mag[0] = trix_ptr->val[0];
						dis_data.mag[1] = trix_ptr->val[1];
						dis_data.mag[2] = trix_ptr->val[2];
						dis_data.magnetic = sqrt(dis_data.mag[0]*dis_data.mag[0]+dis_data.mag[1]*dis_data.mag[1]+dis_data.mag[2]*dis_data.mag[2]);
						memcpy((void*)stream_save_data.mag, (void*)trix_ptr->val, 3*sizeof(float));
						break;
					case EULER_FLOAT:
						euler_ptr = (TRANS_EULER_FLOAT*)&read_buff[data_start];

						dis_data.system_time = euler_ptr->system_time_ms;
						dis_data.voltage = euler_ptr->temperature/132.48f;
						dis_data.euler[0] = euler_ptr->euler[0]*rad_deg_sw;
						dis_data.euler[1] = euler_ptr->euler[1]*rad_deg_sw;
						dis_data.euler[2] = euler_ptr->euler[2]*rad_deg_sw;

						memcpy((void*)stream_save_data.euler, (void*)euler_ptr->euler, 3*sizeof(float));
						stream_save_data.temp = dis_data.voltage;
						break;
					case QUATERN_FLOAT:
						qua_ptr = (TRANS_QUATERN_FLOAT*)&read_buff[data_start];

						dis_data.quatern[0] = qua_ptr->quatern[0];
						dis_data.quatern[1] = qua_ptr->quatern[1];
						dis_data.quatern[2] = qua_ptr->quatern[2];
						dis_data.quatern[3] = qua_ptr->quatern[3];
						break;
					case ROTATION_MATRIX:
						rot_ptr = (TRANS_ROTATION_FLOAT*)&read_buff[data_start];
						for(uint8_t i=0; i<3; i++)
						{
							for(uint8_t j=0; j<3; j++)
							{
								dis_data.Rotation_matrix(j,i) = rot_ptr->M[i][j];
							}
						}
//						qDebug()<<"rotation data"<<endl;
						break;
					case EST_EULER:
						trix_ptr = (TRANS_3D_FLOAT*)&read_buff[data_start];
						dis_data.est_euler[0] = trix_ptr->val[0]*rad_deg_sw;
						dis_data.est_euler[1] = trix_ptr->val[1]*rad_deg_sw;
						dis_data.est_euler[2] = trix_ptr->val[2]*rad_deg_sw;
						break;
					default:
						qDebug()<<"bad type"<<endl;
						break;
					}

					data_start += sub_len;

					if(data_len<0)
					{
						qDebug()<<"len error"<<endl;
					}
				}

				if(save_data)
				{
					save_data	<< stream_save_data.acc[0]	<< ","
								<< stream_save_data.acc[1]  << ","
								<< stream_save_data.acc[2]  << ","
								<< stream_save_data.gyro[0] << ","
								<< stream_save_data.gyro[1] << ","
								<< stream_save_data.gyro[2] << ","
								<< stream_save_data.mag[0]  << ","
								<< stream_save_data.mag[1]  << ","
								<< stream_save_data.mag[2]  << ","
								<< stream_save_data.euler[0]<< ","
								<< stream_save_data.euler[1]<< ","
								<< stream_save_data.euler[2]<< ","
								<< stream_save_data.temp << endl;

				}

			}else{
//				qDebug()<<"check:"<<Check_sum<<",real:"<<*((uint16_t*)&read_buff[data_len+2])<<endl;
			}
		}
	}
}

int Zeng_data_suspend_task(void)
{
	int ret = 0;
	if(NULL == receive_handle)
	{
		ret = 1;
	}else{
		if(CloseHandle(hComm))
		{
			ret = 0;
			//free(hComm);
			hComm = NULL;
#ifdef SAVE_DATA
			data_save.close();
#endif

#ifdef SAVE_CAL_DATA
			data_save.close();
#endif
		}else{
			ret = 2;
		}
		SuspendThread(receive_handle);
	}
	return ret;
}

int Zeng_data_ready_task(char* port_num, uint32_t baudrate)
{
	int ret = 0;
	if(NULL == receive_handle)
	{
		ret = 1;
	}else{
		//Port init
		ret = Zeng_input_usart_init(port_num, baudrate);
		if(0 == ret)
		{
			//qDebug()<<"com" <<(short)port_num << " baudrate:" << baudrate << " Zeng_input_usart_init successful!" << endl;
			ResumeThread(receive_handle);
		}else{
			qDebug()<<"Zeng_input_usart_init fail!" << endl;
		}

	}
	return ret;
}

int Zeng_is_usart_open(void)
{
	return (hComm==NULL?0:1);
}

void Zeng_data_receive_init(void)
{

	receive_handle = CreateThread(
									NULL,
									0,
									Zeng_data_receive_task,
									NULL,
									0,
									NULL);
	Zeng_data_suspend_task();
}









