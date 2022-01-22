#ifndef __REMOTE_CONTROL_H__
#define __REMOTE_CONTROL_H__
#include "main.h"

typedef struct
{
	int x;
	int y;
	float z;
	
	int last_x;
	int last_y;
	float last_z;
	
	int sum_x;
	int sum_y;
	float sum_z;
}GYRO;

extern GYRO gyro;


void GYRO_Resolve(uint32_t StdId, uint8_t *RxData);

#endif
