//通讯协议的处理
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_uart.h"


static uint8_t s_buffer[10];

extern uint16_t hrs_rate;//心率
extern uint32_t step_count;//步数
extern uint8_t bat_percent;//电池

//------------------------------------------------------
//计算校验和
static uint8_t check_sum(uint8_t* data,uint8_t len)
{
	uint8_t temp=0;
	for(int i=0;i<len;i++)
		temp+=data[i];
	return temp;
}
//================================================================
//发送确认命令
//fe 04 01 01 ab b0 ；fe 04 01 01 ae b3
//flag=1 发送成功确认，0-发送失败确认
void send_shakehand(uint8_t flag)
{
	s_buffer[0]=0xfe;
	s_buffer[1]=0x04;
	s_buffer[2]=0x01;//command
	s_buffer[3]=0x01;
	if(flag==1)
	{
		s_buffer[4]=0xab;
		s_buffer[5]=0xb0;
	}
	else
	{
		s_buffer[4]=0xae;
		s_buffer[5]=0xb3;
	}
	eric_uart_send(s_buffer,6);
}
//--------------------------------------
//解析长度命令
uint16_t get_len(uint8_t *data)
{
	if(data[2]==0xaa && (data[5]==check_sum(data,5)) )
	{
		return data[4];
	}
	else
		return 0xff;
}
//-----------------------------------------------------
//处理接收到的数据
//return:1-收到命令正确；0-收到命令错误
void rece_dispatch(uint8_t *data)
{
	uint8_t len=data[0];
	uint8_t *temp;
	if(data[1]!=0xfe || data[2]!=0x05 || (data[len]!=check_sum(&data[1],len-1)) )
	{
		//收到错误命令回复
		send_shakehand(0);
	}
	//解析命令
	switch(data[3])
	{
		case 0xaa://同步数据
			//解析数据,注意是小头
			temp=(uint8_t*)&step_count;
			memcpy(temp,&data[5],3);
			temp=(uint8_t*)&hrs_rate;
			memcpy(temp,&data[8],2);
			bat_percent=data[10];//电池
			send_shakehand(1);
			break;
		default:
			//直接发送数据到手机
			send_data_phone(&data[1],len);
			return;
	};
}
