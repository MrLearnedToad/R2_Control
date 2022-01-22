#include "main.h"
#include "stdlib.h"
#include "math.h"
#include "Remote_Control.h"
GYRO gyro;

uint8_t Yaw[4] = {0};
uint8_t X[4] = {0};
uint8_t Y[4] = {0};

void GYRO_Resolve(uint32_t StdId, uint8_t *RxData)
{
	/*���սǶȺͽ��ٶ���Ϣ*/
	if(StdId == 0x351 || StdId == 0x352)
	{
		/*���սǶ���Ϣ*/
		if(StdId == 0x351)
		{
			gyro.last_z = gyro.z;
			memcpy(&gyro.z, RxData, 4);
		   //gyro.z= *(float*)Yaw;
			gyro.sum_z += gyro.z - gyro.last_z;
            gyro.z=-gyro.z;
//			while(gyro.z >180){gyro.z-=180;}
//			while(gyro.z <-180){gyro.z+=180;}
		}
		
		/*����λ����Ϣ*/
		if(StdId == 0x352)
		{
			gyro.last_x = gyro.x;
			memcpy(X,RxData,4);
			gyro.x = *(int*)X;
			gyro.sum_x += (gyro.x - gyro.last_x)*cos(gyro.z-gyro.last_z);
			
			gyro.last_y = gyro.y;
			memcpy(Y,&RxData[4],4);
			gyro.y = *(int*)Y;
			gyro.sum_y += (gyro.y - gyro.last_y)*cos(gyro.z-gyro.last_z);
            gyro.y=gyro.y;
		}
	}
}