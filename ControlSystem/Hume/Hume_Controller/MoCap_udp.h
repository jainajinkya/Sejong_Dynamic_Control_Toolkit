#ifndef MOCAP_UDP_H
#define MOCAP_UDP_H
#ifdef __cplusplus
extern "C"
{
#endif

#define CMD_CALIBRATE				0x01000000
#define CMD_SET_TORQUE				0x02000000
#define CMD_SET_MODE				0x03000000
#define CMD_SET_MODE_RAW_TORQUE		0x03000001
#define CMD_SET_MODE_COMP_TORQUE	0x03000002

#define CMD_DATA_LEN 29

#define NOTIFY_ADDR             "192.168.1.6"
#define NOTIFY_PORT             51123
#define CMD_PORT                51124
#define STT_PORT                51125
#define POS_PORT                51128

#define NUM_LED 9
    
#ifndef __cplusplus
typedef struct
{
	pthread_mutex_t *mutex;
	void *win;
}udp_arg;
#endif

#include <stdint.h>

typedef struct
{
	uint32_t b8 :1;
	uint32_t b7 :1;
	uint32_t b6 :1;
	uint32_t b5 :1;
	uint32_t b4 :1;
	uint32_t b3 :1;
	uint32_t b2 :1;
	uint32_t b1 :1;
	uint32_t b0 :1;
} Int32BitField;

typedef struct
{
	int32_t index;
	uint32_t timeStamp;
	double x[NUM_LED];
	double y[NUM_LED];
	double z[NUM_LED];
	uint32_t validBits;
	uint32_t extraInt;
} message;


#ifdef __cplusplus
}
#endif
#endif
