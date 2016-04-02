#include "eric_uart.h"
uint8_t buffer[32];
static uint8_t index=1;//������1��ʼ��¼��0��¼�ܳ���
static uint8_t command_len=5;//���յ��������,������5��
static uint8_t command_start=0;//��ʼ��������=1������ظ�Ϊ0
static uint8_t temp=0;
uint8_t rece_flag=0;//���յ�����ı�־

void UART0_IRQHandler(void)
{
    if (NRF_UART0->EVENTS_RXDRDY != 0)
    {
        // Clear UART RX event flag
        NRF_UART0->EVENTS_RXDRDY = 0;
        temp = (uint8_t)NRF_UART0->RXD;

					if(rece_flag==0 && command_start==0 && temp==0xfe)
					{
						command_start=1;
						command_len=5;
						buffer[index]=temp;
						index++;
					}
					else if(command_start==1)
					{
						if(index==2 && temp==0x05)//ȷ��ʱ����������,����st
						{
							buffer[index]=temp;
							index++;
							command_start=2;
						}
						else
						{
							index=1;
							command_start=0;//�ָ�							
						}
					}
					else  if(command_start==2)//�������ʼ
					{
							buffer[index]=temp;
						//�ж������
							if(index==4)
							{
								command_len+=temp;//���ֽڣ��ܳ����ܴ���255
								index++;
							}
							else
							{
								if(index==command_len)//����������
								{
									index=1;
									command_start=0;//�ָ�
									buffer[0]=command_len;
									rece_flag=1;
								}
								else
									index++;
							}
					}
					else
					{
							index=1;
							command_start=0;//�ָ�
					}
    }
}

void eric_uart_Init()
{
  UART_InitType UART_InitStruct;
	UART_InitStruct.txd_pin_number 		= 3;									// pin 0.9
	UART_InitStruct.hwfc 				= UART_CONFIG_HWFC_Disabled;
	UART_InitStruct.rxd_pin_number 		= 4;									// pin 0.11
	UART_InitStruct.baudrate 			= UART_BAUDRATE_BAUDRATE_Baud115200;
	UART_InitStruct.parity 				= UART_CONFIG_PARITY_Excluded;			// no parity


    /* Power on */
    NRF_UART0->POWER = (UART_POWER_POWER_Enabled << UART_POWER_POWER_Pos);

    /* initialize TXD */
    NRF_UART0->PSELTXD = UART_InitStruct.txd_pin_number;

    /* initialize RXD */
    NRF_UART0->PSELRXD = UART_InitStruct.rxd_pin_number;

    /* initialize hardware flow control */
    if (UART_InitStruct.hwfc == UART_CONFIG_HWFC_Enabled)
    {
        /* initialize RTS */
        NRF_UART0->PSELRTS = UART_InitStruct.rts_pin_number;

        /* initialize CTS */
        NRF_UART0->PSELCTS = UART_InitStruct.cts_pin_number;
        
        /* enable hardware flow control */
        NRF_UART0->CONFIG &= ~(UART_InitStruct.hwfc << UART_CONFIG_HWFC_Pos);
        NRF_UART0->CONFIG |= (UART_InitStruct.hwfc << UART_CONFIG_HWFC_Pos);
    }

    /* set baudrate */
    NRF_UART0->BAUDRATE = (UART_InitStruct.baudrate << UART_BAUDRATE_BAUDRATE_Pos);
    
    /* set parity */
    NRF_UART0->CONFIG &= ~(UART_InitStruct.parity << UART_CONFIG_PARITY_Pos);
    NRF_UART0->CONFIG |= (UART_InitStruct.parity << UART_CONFIG_PARITY_Pos);
    

    NRF_UART0->ENABLE           = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
    NRF_UART0->TASKS_STARTTX    = 1;
    NRF_UART0->TASKS_STARTRX    = 1;
    NRF_UART0->EVENTS_RXDRDY    = 0;

    /* Enable UART interrupt */
    NRF_UART0->INTENCLR = 0xFFFFFFFFUL; // clear all
    NRF_UART0->INTENSET = UART_INTENSET_RXDRDY_Set << UART_INTENSET_RXDRDY_Pos;

    NVIC_ClearPendingIRQ(UART0_IRQn);
    NVIC_SetPriority(UART0_IRQn, 3);
    NVIC_EnableIRQ(UART0_IRQn);

}

void eric_uart_deinit(void)
{
    /* Power off */
    NRF_UART0->POWER = (UART_POWER_POWER_Disabled << UART_POWER_POWER_Pos);

   memset(buffer,0,sizeof(buffer));
}

void eric_uart_send(const uint8_t *buf, int size)
{
    for (int i = 0; i < size; i++)
    {
        NRF_UART0->TXD = buf[i];

        while (NRF_UART0->EVENTS_TXDRDY != 1)
        {
            /* do nothing */
        }
        NRF_UART0->EVENTS_TXDRDY = 0;
    }    
}

