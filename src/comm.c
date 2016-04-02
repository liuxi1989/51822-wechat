//ͨѶЭ��Ĵ���
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "app_uart.h"


static uint8_t s_buffer[10];

extern uint16_t hrs_rate;//����
extern uint32_t step_count;//����
extern uint8_t bat_percent;//���

//------------------------------------------------------
//����У���
static uint8_t check_sum(uint8_t* data,uint8_t len)
{
	uint8_t temp=0;
	for(int i=0;i<len;i++)
		temp+=data[i];
	return temp;
}
//================================================================
//����ȷ������
//fe 04 01 01 ab b0 ��fe 04 01 01 ae b3
//flag=1 ���ͳɹ�ȷ�ϣ�0-����ʧ��ȷ��
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
//������������
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
//������յ�������
//return:1-�յ�������ȷ��0-�յ��������
void rece_dispatch(uint8_t *data)
{
	uint8_t len=data[0];
	uint8_t *temp;
	if(data[1]!=0xfe || data[2]!=0x05 || (data[len]!=check_sum(&data[1],len-1)) )
	{
		//�յ���������ظ�
		send_shakehand(0);
	}
	//��������
	switch(data[3])
	{
		case 0xaa://ͬ������
			//��������,ע����Сͷ
			temp=(uint8_t*)&step_count;
			memcpy(temp,&data[5],3);
			temp=(uint8_t*)&hrs_rate;
			memcpy(temp,&data[8],2);
			bat_percent=data[10];//���
			send_shakehand(1);
			break;
		default:
			//ֱ�ӷ������ݵ��ֻ�
			send_data_phone(&data[1],len);
			return;
	};
}
