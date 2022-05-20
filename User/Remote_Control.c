#include "main.h"
#include "stdlib.h"
#include "math.h"
#include "Remote_Control.h"
#include "string.h"
GYRO gyro;
extern int debug;
extern float dZ;
extern uint32_t speed_timer;
uint8_t Yaw[4] = {0};
uint8_t X[4] = {0};
uint8_t Y[4] = {0};
extern void speed_cal(void);


void GYRO_Resolve(uint32_t StdId, uint8_t *RxData)
{
    static uint8_t flag_init=0;
	/*接收角度和角速度信息*/
	if(StdId == 0x351 || StdId == 0x352)
	{
		/*接收角度信息*/
		if(StdId == 0x351)
		{
			gyro.last_z = gyro.z;
			memcpy(&gyro.z, RxData, 4);
		   //gyro.z= *(float*)Yaw;
			gyro.sum_z += gyro.z - gyro.last_z;
            gyro.z=-gyro.z;

            if(flag_init==0)
            {
                while(fabs(gyro.z)>180)
                {
                    if(gyro.z>0)
                        gyro.z-=360.0f;
                    else
                        gyro.z+=360.0f;
                }
                dZ=-gyro.z;
                flag_init=1;
            }
		}
		
		/*接收位置信息*/
		if(StdId == 0x352)
		{
			gyro.last_x = gyro.x;
			memcpy(X,RxData,4);
			gyro.x = *(int*)X;
			//gyro.sum_x += (gyro.x - gyro.last_x)*cos(gyro.z-gyro.last_z);
			//debug++;
			gyro.last_y = gyro.y;
			memcpy(Y,&RxData[4],4);
			gyro.y = *(int*)Y;
			//gyro.sum_y += (gyro.y - gyro.last_y)*cos(gyro.z-gyro.last_z);
            gyro.y=gyro.y;
			speed_cal();
		}
        gyro.error_counter=0;
	}
}
