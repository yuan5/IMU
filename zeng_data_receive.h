#ifndef ZENG_DATA_RECEIVE_H
#define ZENG_DATA_RECEIVE_H

#include<Eigen/Eigen>

typedef unsigned char  uint8_t;
typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef signed short int16_t;

#define adc_to_voltage		(41.5f*1.5f/4096.0f)
#define adc_to_degree		(360.0f/65536.0f)
//#define adc_to_speed		(adc_to_degree*10000)
#define adc_to_speed		(adc_to_degree*10000/360.0f)
#define adc_to_current		(3.3/(4096.0f*10.0f*0.0005f))//(3.3/4096.0f)/0.005f
#define DEG_TO_RAD			(3.1415926f/180.0f)

//#define COMMOND_MOTOR               (0x01)
//#define COMMOND_SET_PORT_CONFIG    	(0x02)
#define COMMOND_WRITE_CAN_ID		(0x05)
#define COMMOND_SYS_RESET           (0x08)

using namespace Eigen;

typedef enum UPPER_COMMAND_TYPE_{
    COMMAND_SET_MAG_CAL= 0x00,
    COMMAND_CAL,
    COMMAND_SET_PORT_CONFIG,
}UPPER_COMMAND_TYPE;

typedef enum _SEND_DATA_TYPE{
	SYSTEM_DATA = 0x00,
	GYRO_FLOAT,
	ACCEL_FLOAT,
	MAG_FLAOT,
	EULER_FLOAT,
	QUATERN_FLOAT,
	ROTATION_MATRIX,
	LINAR_ACC,
	EST_EULER
}SEND_DATA_TYPE;

typedef struct _TRANS_HEADER{
	uint8_t magic[2];
	uint8_t data_type;
	uint8_t data_len;
}TRANS_HEADER;

typedef struct _TRANS_MAG_FLOAT{
	float val[3];
}TRANS_3D_FLOAT;

typedef struct _TRANS_EULER_FLOAT{
	uint32_t system_time_ms;
	float euler[3];
	int16_t temperature;
	uint8_t reserve;
	uint8_t CheckSum;
}TRANS_EULER_FLOAT;

typedef struct _TRANS_QUATERN_FLOAT{
	float quatern[4];
}TRANS_QUATERN_FLOAT;

typedef struct _TRANS_ROTATION_FLOAT{
	float M[3][3];
}TRANS_ROTATION_FLOAT;

//current detect result
typedef struct _CUR_DETECT_RESULT{
	TRANS_HEADER header;
	uint16_t cur_offset[2];
	uint8_t reserve[3];
	uint8_t CheckSum;
}CUR_DETECT_RESULT;

typedef struct _UI_DISPLAY_DATA{
	uint32_t system_time;
	float voltage;
	float test_voltage;
	float euler[3];
	float est_euler[3];
	float mag[3];
	float magnetic;
	float acc[3];
	float acceleration;
	float gyro[3];
	float quatern[4];
	Matrix3f Rotation_matrix;
	uint8_t pole[2];
	uint16_t encoder_offset[2];
	uint16_t current_offset[2][2];
	char direct[2];

	uint32_t data_type;

	bool update_flag;
}UI_DISPLAY_DATA;

//from upper to device
typedef enum __MOTOR_STATUS{
  STOP = 0,
  DETECT_ENCODE,
  DETECT_DCCAL,
  DETECT_RESISTANCE,
  DETECT_INDUCTANCE,
  RUN_CURRENT,
  RUN_SPEED,
  AXIS_STATE_MOTOR_CALIBRATION,
  DRV_FAULT_ERR
}MOTOR_STATUS;

typedef struct _ZENG_UART_RECEIVE_DATA{
  uint8_t magic[2];
  uint8_t common_type;
  uint8_t data_len;
}ZENG_UART_RECEIVE_DATA;

typedef struct _MOTOR_COMMOND{
  ZENG_UART_RECEIVE_DATA header;
  uint8_t motor_status[2];
  uint8_t current[2];
  int16_t speed[2];
}MOTOR_COMMOND;

//typedef struct _CAN_WRITE_ID{
//	ZENG_UART_RECEIVE_DATA header;
//	uint32_t write_id;
//	uint32_t read_id;
//}CAN_WRITE_ID_MSG;


typedef struct HD_IMU_OUTPUT_{
	bool		Usart;
	uint8_t		resrved;
	uint16_t	frequency;
	uint32_t	output_types;
	uint32_t	baudrate;
}HD_IMU_OUTPUT;

typedef struct _IMU_PORT_SET_TYPE{
	ZENG_UART_RECEIVE_DATA header;
	HD_IMU_OUTPUT set;
	uint16_t set_magic;
    uint16_t CheckSum;
}IMU_PORT_SET_TYPE;

typedef struct _IMU_MAG_OFFSET_TYPE{
	ZENG_UART_RECEIVE_DATA header;
	float offset[3];
	uint16_t magic;
	uint16_t CheckSum;
}IMU_MAG_OFFSET_TYPE;

void Zeng_data_receive_init(void);
int Zeng_data_suspend_task(void);
int Zeng_data_ready_task(char* port_num, unsigned int baudrate);
int16_t Zeng_input_usart_init(uint8_t com_num, uint32_t baudrate);
int Zeng_uart_write_data(uint16_t cnt, char *buf);
int Zeng_is_usart_open(void);
uint16_t Zeng_cal_CheckSum_uint16(uint16_t *start, uint8_t len);
uint8_t Zeng_cal_CheckSum(uint8_t *start, uint32_t len);

extern UI_DISPLAY_DATA dis_data;
extern const unsigned char Header[2];
//extern unsigned short angle;

//#define SAVE_CAL_DATA

#ifdef SAVE_CAL_DATA
extern float real_angle;
extern bool save_flag;
#endif



#endif // ZENG_DATA_RECEIVE_H
