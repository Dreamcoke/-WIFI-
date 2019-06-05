/*
����ͨ��ʵ��

PA2-----USART2_TX
PA3----USART2_RX

����USART���жϽ��н��պͷ��͡�

 �õ��˴��ڵ������֡�

 ע���޸� stm32f10x.h ��HCLK��Ƶ��Ϊ�����徧��Ƶ�ʡ� ���磬�������徧��Ƶ��Ϊ8MHz������Ҫ���壺
 #define HSE_VALUE ((uint32_t)8000000) // �����Լ��������ϵ��ⲿʱ��ԴƵ��

 */
 #include "stm32f10x.h"
#define TxBufferSize_wifi (countof(TxBuffer_wifi) - 1) //����UART1�ķ��ͻ��������ֽ���
#define RxBufferSize_wifi 0x51 //����UART1�Ľ��ջ��������ֽ���
#define countof(a) (sizeof(a) / sizeof(*(a)))
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "misc.h" //����NVIC�Ĺ̼�
//u8 *TxBuffer_wifi;
u8 TxBuffer_wifi[0xff];
u8 RxBuffer_wifi[RxBufferSize_wifi];
u8 NbrOfDataToTransfer_wifi = TxBufferSize_wifi;
u8 NbrOfDataToRead_wifi = RxBufferSize_wifi;
u8 TxCounter_wifi = 0;
u16 RxCounter_wifi = 0;
u8 Tx_Flag_wifi=0;
u8 Rx_Flag_wifi=0;
u8 flag_a_wifi=13;
char transfer_string_wifi[320];
//��ʪ��
u8 temperature_wifi=0;  	    
u8 humidity_wifi=0;
//pm2.5��ֵ
u16 dustValue_wifi;


void RCC_Configuration_wifi(void)//ʱ�������ӳ���
{ 
	SystemInit(); //72MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//ʹ��AFIO��ʱ�ӣ��������ڸ��ù���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//ʹ��GPIOA��ʱ��
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);//ʹ��GPIOE��ʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART1��ʱ�ӣ� USART1�ҽӵ�APB2�ϡ�����USART2-5�ҽӵ�APB1��
	
}
void GPIO_Configuration_wifi(void) //GPIO����
{ /*��������*/
	GPIO_InitTypeDef GPIO_InitStructure;//����GPIO��ʼ���ṹ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //���ùܽ�PA10/USART1_RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA10

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2; //���ùܽ�PA9/USART1_TX
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; //IO������Ϊ���������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//��������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��PA10
}
void USART_Configuration_wifi(void) //USART����
{ /*USART����*/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //USART_WordLength_8b; //8 λ���� //USART_WordLength_9b; //9 λ����
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	//USART_StopBits_1 ;//��֡��β���� 1 ��ֹͣλ
	//USART_StopBits_0.5;//��֡��β���� 0.5 ��ֹͣλ
	//USART_StopBits_2 ;//��֡��β���� 2 ��ֹͣλ
	//USART_StopBits_1.5;//��֡��β���� 1.5 ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;
	//USART_Parity_No ;//��żʧ��
	//USART_Parity_Even;//żģʽ
	//USART_Parity_Odd ;//��ģʽ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	//USART_HardwareFlowControl_None; //Ӳ��������ʧ��
	//USART_HardwareFlowControl_RTS; //�������� RTSʹ��
	//USART_HardwareFlowControl_CTS; //������� CTSʹ��
	//USART_HardwareFlowControl_RTS_CTS;//RTS�� CTSʹ��
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//USART_Mode_Tx;//����ʹ��
	//USART_Mode_Rx;//����ʹ��
	USART_Init(USART2, &USART_InitStructure);//��ʼ������
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //���������ж�,���ռĴ������գ�RXNE=1��ʱ�����ж�
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE); // ���������ж�,���ͼĴ�����(TXE=1)ʱ�ܲ����ж�
	/*���ڵķ����ж����������ֱ��ǣ�
	l�������ݼĴ������жϣ�TXE�� l
	��������жϣ�TC��*/
	USART_Cmd(USART2, ENABLE); //����USART
	}
void NVIC_Configuration_wifi(void) //NVIC����
{ /*����NVIC��Ӧ�����ȼ�λ*/
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //���ȼ�����1��1:3,1λ��ռ���ȼ��� 3λ�����ȼ���
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //���ô���1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ� 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	NVIC_Init(&NVIC_InitStructure);
}
void USART2_IRQHandler(void) //����2 �жϷ������
{
	GPIO_SetBits(GPIOE,GPIO_Pin_1);
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //�жϽ��ռĴ����Ƿ�ǿ�
	{//����⵽�ж϶���
		//RxBuffer_wifi[RxCounter_wifi++]=USART_ReceiveData(USART2); /* Read one byte from the receive data register */
		//if (RxBuffer_wifi[RxCounter_wifi-1]=='\n') //�����⵽���У������ý��ձ�־Ϊ1.
		// \n:���У���굽�������ף�
		// \r:�س�����굽��������
		//{
		//Rx_Flag_wifi=1;
		//TxBuffer_wifi[RxCounter_wifi]=0; //���ͻ�����������
		//}
		//if(RxCounter_wifi == NbrOfDataToRead_wifi) //������ջ��������ˡ�
		//{
		//USART_ITConfig(USART2, USART_IT_RXNE, DISABLE); //�رս����ж�
		//Rx_Flag_wifi=1;
		//TxBuffer_wifi[RxCounter_wifi]=0; //���ͻ�����������
			u8 res = USART_ReceiveData(USART2);
			if(res == '1')
			{
				GPIO_SetBits(GPIOC,GPIO_Pin_5);
			}
			//USART_SendData(USART2, res);
	}
		if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
		{ //����⵽�����ж�
			USART_SendData(USART2, TxBuffer_wifi[TxCounter_wifi++]); //�������ݼĴ���дһ���ֽ�
			if(TxCounter_wifi == NbrOfDataToTransfer_wifi)
			{
			TxCounter_wifi=0;
			USART_ITConfig(USART2, USART_IT_TXE, DISABLE); /* Disable the USART1 Transmit interrupt */
			}
		}
}
void WIFI_UART_INIT(void){
	RCC_Configuration_wifi();
	GPIO_Configuration_wifi();
	USART_Configuration_wifi();
	NVIC_Configuration_wifi();
}
void Send_Server(char *msg){
 // TxBuffer_wifi = (u8*)msg;
	char *str = msg;
	strcpy((char*)TxBuffer_wifi,str);
	NbrOfDataToTransfer_wifi = strlen(msg);
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE); 
}
	

	
	
