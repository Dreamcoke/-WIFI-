
/*
����ͨ��ʵ��

PA9-----USART1_TX
PA10----USART1_RX

����USART���жϽ��н��պͷ��͡�

 �õ��˴��ڵ������֡�

 ע���޸� stm32f10x.h ��HCLK��Ƶ��Ϊ�����徧��Ƶ�ʡ� ���磬�������徧��Ƶ��Ϊ8MHz������Ҫ���壺
 #define HSE_VALUE ((uint32_t)8000000) // �����Լ��������ϵ��ⲿʱ��ԴƵ��

 */


#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "stm32f10x_it.h"
#include "stm32f10x_it.c"
#include "misc.h" //����NVIC�Ĺ̼�
#include "misc.c" //����NVIC�Ĺ̼�
#include "dht11.h"
#include <stdio.h>
#include "lcd1602.h"
#include <string.h>
#include "delay.h"
#include "wifi.h"
#include "BH1750.h"
#include "flash.h"
#include "sds011.h"
#include "arofene.h"
#define TxBufferSize (countof(TxBuffer) - 1) //����UART1�ķ��ͻ��������ֽ���
#define RxBufferSize 0x51 //����UART1�Ľ��ջ��������ֽ���
#define countof(a) (sizeof(a) / sizeof(*(a)))

u8 TxBuffer[0x78];
u8 RxBuffer[RxBufferSize];
u8 NbrOfDataToTransfer = TxBufferSize;
u8 NbrOfDataToRead = RxBufferSize;
u8 TxCounter = 0;
u16 RxCounter = 0;
extern u8 res;
extern u8 pm25_low;
extern u8 pm25_high; 
extern u8 pm10_low;
extern u8 pm10_high;
extern u16 pm25;
extern u16 pm10;
extern u16 aro;
u8 Tx_Flag=0;
u8 Rx_Flag=0;
u8 flag_a=13;
u8 flag_w=1;
u8 flag_r=0;
char in_data[10]={'a','b','c','d','e'};//Ҫд�������  
char out_data[10];//�����  
u8 STATUS=0;
char weather1[10]="rain";
char weather2[10]="sun";
float flag_g;
char transfer_string[0x78];
char transfer_string2[0x78];
char transfer_string3[0x78];
char transfer_stringWifi[0x78];
char temperature_unit_1=0xdf;
char temperature_unit_2=0x43;
//��ʪ��
u8 temperature=0;  	    
u8 humidity=0;
//pm2.5��ֵ
u16 dustValue;
void RCC_Configuration(void); //ʱ������

void GPIO_Configuration(void); //GPIO����


void delay_nms(u16 time);
void WIFI_UART_INIT(void);
void l1602_INIT(void);
void Send_Server(char *msg);
float Get_gy30(void);

unsigned int i;
FILE *fp;


int main(void)
{ 
RCC_Configuration();//����ʱ��

GPIO_Configuration(); //����GPIO
arofene_USART_Init();//���ü�ȩģ��
sds011_USART_Init();
delay_init();	

WIFI_UART_INIT();
l1602_INIT();
GPIOConfig();	//���ù���ģ��
Init_BH1750();       //��ʼ��BH1750
GPIO_SetBits(GPIOC,GPIO_Pin_8);
while(DHT11_Init()){
	GPIO_ResetBits(GPIOE,GPIO_Pin_5);
	delay_nms(1000);
	GPIO_SetBits(GPIOE,GPIO_Pin_5);
	delay_nms(1000);
}
GPIO_ResetBits(GPIOE,GPIO_Pin_6);
while(1)
{  
	
	GPIO_SetBits(GPIOE,GPIO_Pin_0);
	if(temperature == 0)
		GPIO_ResetBits(GPIOE,GPIO_Pin_5);
	else
		GPIO_SetBits(GPIOE,GPIO_Pin_5);
	flag_a = DHT11_Read_Data(&temperature,&humidity);		//��ȡ��ʪ��ֵ	flag_a = DHT11_Read_Data(&temperature,&humidity);		//��ȡ��ʪ��ֵ	
	flag_g=Get_gy30();//��ȡ����ǿ��
	flag_w=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7);
	//fp=fopen("E:\data\data.txt","wt+");
	//fputc(temperature,fp);
	//fclose(fp);
	if(flag_w==0)
	  flag_r=1;//����
	else 
		flag_r=0;//����
	
	 STATUS=Write_Flash(in_data,5);  
   //Delay(0x02FFFF);  
   if(STATUS)  
    {      
            GPIO_SetBits(GPIOC, GPIO_Pin_11);//����led1  
            Read_Flash(out_data,5);  
           // for(i=0;i<5;i++)
			      //    transfer_string[i]=(char)out_data[i];
    }  
    //while(1); 

	
	sprintf(transfer_string,"The temperature is %d,The humidity is %d,The pm2.5 is %.2f,The pm10 is %.2f,flag is %x,The is light %f lx,The weather is %d\r\n\0",temperature,humidity,pm25/10.0,pm10/10.0,flag_a,flag_g,flag_r);
	
	strcpy((char*)TxBuffer,transfer_string);
	sprintf(transfer_stringWifi,"%d,%d,%.2f,%x,%f,%d,%.2f,%d,,,10000002\r\n\0",temperature,humidity,pm25/10.0,flag_a,flag_g,flag_r,pm10/10.0,aro);
	Send_Server(transfer_stringWifi);
	L1602_Clear();
	sprintf(transfer_string2,"   T: %d  %c%c ",temperature,temperature_unit_1,temperature_unit_2);
	L1602_string(1,1,transfer_string2);
	sprintf(transfer_string3,"   H: %d%% RH",humidity);
	L1602_string(2,1,transfer_string3);
	delay_nms(500);
	L1602_Clear();
	//delay_nms(500);
	sprintf(transfer_string3,"PM2.5:%.1fug/m3",pm25/10.0);
	L1602_string(1,1,transfer_string3);
	sprintf(transfer_string3,"PM10 :%.1fug/m3",pm10/10.0);
	L1602_string(2,1,transfer_string3);
	delay_nms(500);
	L1602_Clear();
	sprintf(transfer_string3,"  A: %d  ug/m3",aro);
	L1602_string(1,1,transfer_string3);
	sprintf(transfer_string3,"  L: %.2f lx",flag_g);
	L1602_string(2,1,transfer_string3);	
	delay_nms(500);
	
	if(humidity>=80)
	{
		GPIO_SetBits(GPIOE,GPIO_Pin_1);//������
		GPIO_ResetBits(GPIOE,GPIO_Pin_0);//������
		GPIO_ResetBits(GPIOE,GPIO_Pin_2);//���ȵ͵�ƽ
	  GPIO_ResetBits(GPIOE,GPIO_Pin_4);//�������͵�ƽ
		delay_nms(2000);
	}
	if(humidity<80)
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_1);//������
		GPIO_ResetBits(GPIOE,GPIO_Pin_0);//������
		GPIO_SetBits(GPIOE,GPIO_Pin_2);//���ȸߵ�ƽ
	  GPIO_SetBits(GPIOE,GPIO_Pin_4);//�������ߵ�ƽ
		delay_nms(500);
	}	
	
	if(Rx_Flag==1) //������յ�һ���Ի��н��������ݣ��򽫽��յ��������ٷ��ͳ�ȥ��
	{ //�ж��Ƿ��յ�һ֡��Ч����
	Rx_Flag=0;
	for(i=0; i< NbrOfDataToRead; i++) TxBuffer[i] =0;
	// TxBuffer[0]='\n';
	// TxBuffer[1]='\r';
	for(i=0; i< RxCounter; i++) TxBuffer[i] = RxBuffer[i];
	//TxBuffer[RxCounter]='\r';
	//TxBuffer[RxCounter+1]='\n';
	// TxBuffer[RxCounter+2]='\r';
	//�����ջ�����������ת�����ͻ����������ڻ�����ͷ�ϼӻ��з���׼��ת��
	RxCounter=0;
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //���������ж�,���ռĴ������գ�RXNE=1��ʱ�����ж�
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // ���������ж�
//  TxBuffer="for test\r\n";
//	memcpy(TxBuffer,"for test\r\n",11);
	//delay_nms(1000);
		}
	
	}
  
	
 }
void RCC_Configuration(void)//ʱ�������ӳ���
{ 
	SystemInit(); //72MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//ʹ��AFIO��ʱ�ӣ��������ڸ��ù���
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//ʹ��GPIOA��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);//ʹ��GPIOE��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1��ʱ�ӣ� USART1�ҽӵ�APB2�ϡ�����USART2-5�ҽӵ�APB1��
	
}



void GPIO_Configuration(void) //GPIO����
{ /*��������*/
	GPIO_InitTypeDef GPIO_InitStructure;//����GPIO��ʼ���ṹ������

	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_0|GPIO_Pin_6|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_8|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_SetBits(GPIOE,GPIO_Pin_5);
	GPIO_SetBits(GPIOE,GPIO_Pin_6);
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);	
	GPIO_SetBits(GPIOE,GPIO_Pin_3);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}


void delay_nms(u16 time)//��ʱ�ӳ���
{ 
	u16 i=0;
	while(time--)
	{ i=12000; //�Լ�����
	while(i--) ;
	}
}


