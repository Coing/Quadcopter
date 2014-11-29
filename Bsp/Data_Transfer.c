#include "data_transfer.h"
#include "imu.h"
#include "usart.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

u8 data_to_send[50];


void Data_Exchange(void)
{

	if(Send_Status)
	{
		Send_Status = 0;
		Data_Send_Status();
	}
	else if(Send_Senser)
	{
		Send_Senser = 0;
		Data_Send_Senser();
	}
}


void Data_Send_Status(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x01;
	data_to_send[_cnt++]=0;
	vs16 _temp;
	_temp = (int)(Att_Angle.rol*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Att_Angle.pit*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	_temp = (int)(Att_Angle.yaw*100);
	//_temp = (int)(Mag_Heading*100);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	//vs32 _temp2 = Alt;
//	vs32 _temp2 = sonarAlt;
	_temp = sonarAlt/100;
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	vs32 _temp2 = 6000;
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);
		
	data_to_send[_cnt++]=0xA0;	//Ëø¶¨

	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	Uart1_Put_Buf(data_to_send,_cnt);

}
void Data_Send_Senser(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x02;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Acc.x);
	data_to_send[_cnt++]=BYTE0(Acc.x);
	data_to_send[_cnt++]=BYTE1(Acc.y);
	data_to_send[_cnt++]=BYTE0(Acc.y);
	data_to_send[_cnt++]=BYTE1(Acc.z);
	data_to_send[_cnt++]=BYTE0(Acc.z);
	data_to_send[_cnt++]=BYTE1(Gyr.x);
	data_to_send[_cnt++]=BYTE0(Gyr.x);
	data_to_send[_cnt++]=BYTE1(Gyr.y);
	data_to_send[_cnt++]=BYTE0(Gyr.y);
	data_to_send[_cnt++]=BYTE1(Gyr.z);
	data_to_send[_cnt++]=BYTE0(Gyr.z);
	data_to_send[_cnt++]=BYTE0(magT.x);
	data_to_send[_cnt++]=BYTE0(magT.x);
	data_to_send[_cnt++]=BYTE0(magT.y);
	data_to_send[_cnt++]=BYTE0(magT.y);
	data_to_send[_cnt++]=BYTE0(magT.z);
	data_to_send[_cnt++]=BYTE0(magT.z);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++] = sum;
	

	Uart1_Put_Buf(data_to_send,_cnt);

}
