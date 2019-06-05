
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

#define TxBufferSize (countof(TxBuffer) - 1) //����UART1�ķ��ͻ��������ֽ���
#define RxBufferSize 0x51 //����UART1�Ľ��ջ��������ֽ���
#define countof(a) (sizeof(a) / sizeof(*(a)))

u8 TxBuffer[0x78];
u8 RxBuffer[RxBufferSize];
u8 NbrOfDataToTransfer = TxBufferSize;
u8 NbrOfDataToRead = RxBufferSize;
u8 TxCounter = 0;
u16 RxCounter = 0;
u8 Tx_Flag=0;
u8 Rx_Flag=0;
u8 flag_a=13;
u8 flag_w=1;
u8 flag_r=0;
u32 in_data[5]={11,22,33,44,55};//Ҫд�������  
u32 out_data[5];//�����  
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
void NVIC_Configuration(void); //NVIC����
void GPIO_Configuration(void); //GPIO����
void USART_Configuration(void); //USART����
void GP2Y_ADC_init(void);
void delay_nms(u16 time);
void WIFI_UART_INIT(void);
void l1602_INIT(void);
void Send_Server(char *msg);
float Get_gy30(void);
u16 GetGP2YAverageValue(u8 times);
u16 GetGP2YSingleValue(void);
unsigned int i;
FILE *fp;



int main(void)
{ 
RCC_Configuration();//����ʱ��
NVIC_Configuration();//����NVIC
GPIO_Configuration(); //����GPIO
USART_Configuration();//����USART
delay_init();	
GP2Y_ADC_init();
WIFI_UART_INIT();
l1602_INIT();
GPIOConfig();	//���ù���ģ��
Init_BH1750();       //��ʼ��BH1750
while(DHT11_Init()){
	GPIO_ResetBits(GPIOE,GPIO_Pin_5);
	delay_nms(1000);
	GPIO_SetBits(GPIOE,GPIO_Pin_5);
	delay_nms(1000);
}
GPIO_ResetBits(GPIOE,GPIO_Pin_6);
while(1)
{  
	
	dustValue=GetGP2YSingleValue();
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
	
	// STATUS=Write_Flash(in_data,5);  
   //Delay(0x02FFFF);  
  //  if(STATUS)  
  //  {       int i;
     //       GPIO_SetBits(GPIOC, GPIO_Pin_11);//����led1  
     //       Read_Flash(out_data,5);  
     //        for(i=0;i<5;i++)
		//	          transfer_string[i]=out_data[i];
   // }  
    //while(1);  
	dustValue=156;
	flag_g=116.00;
	sprintf(transfer_string,"The temperature is %d,The humidity is %d,The pm2.5 is %d,flag is %x,The is light %f lx,The weather is %d\r\n\0",temperature,humidity,dustValue,flag_a,flag_g,flag_r);
	
	strcpy((char*)TxBuffer,transfer_string);
	sprintf(transfer_stringWifi,"%d,%d,%d,%x,%f,%d\r\n\0",temperature,humidity,dustValue,flag_a,flag_g,flag_r);
	Send_Server(transfer_stringWifi);
	L1602_Clear();
	sprintf(transfer_string2," T:%d%c%c H:%d%%RH",temperature,temperature_unit_1,temperature_unit_2,humidity);
	L1602_string(1,1,transfer_string2);
	sprintf(transfer_string3,"PM2.5:%d",dustValue);
	L1602_string(2,1,transfer_string3);
	delay_nms(1000);
	L1602_Clear();
	//delay_nms(500);
	sprintf(transfer_string3,"L:%0.2f",flag_g);
	L1602_string(1,1,transfer_string3);
	if(humidity>=60)
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_1);
	  GPIO_SetBits(GPIOE,GPIO_Pin_2);
		GPIO_ResetBits(GPIOE,GPIO_Pin_3);
	}
	if(humidity<60)
		GPIO_ResetBits(GPIOE,GPIO_Pin_2);
	//delay(2000);
	//L1602_Clear();
	//L1602_string(1,1,"Hi STM32,");
	//L1602_string(2,1,"Nice to meet you!");
	//delay(2000);
//	snprintf(TxBuffer,sizeof(TxBuffer),"%c\r\n",'c');
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // ��������
	delay_nms(500);
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
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //���������ж�,���ռĴ������գ�RXNE=1��ʱ�����ж�
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // ���������ж�
//  TxBuffer="for test\r\n";
//	memcpy(TxBuffer,"for test\r\n",11);
	delay_nms(1000);
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

void NVIC_Configuration(void) //NVIC����
{ /*����NVIC��Ӧ�����ȼ�λ*/
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //���ȼ�����1��1:3,1λ��ռ���ȼ��� 3λ�����ȼ���
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //���ô���1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //��ռ���ȼ� 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	NVIC_Init(&NVIC_InitStructure);
}
void GPIO_Configuration(void) //GPIO����
{ /*��������*/
	GPIO_InitTypeDef GPIO_InitStructure;//����GPIO��ʼ���ṹ������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //���ùܽ�PA10/USART1_RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA10

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9; //���ùܽ�PA9/USART1_TX
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; //IO������Ϊ���������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9
	//GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_6;
	//GPIO_Init(GPIOB,&GPIO_InitStructure); //��ʼ��PA9
	//GPIO_SetBits (GPIOB,GPIO_Pin_6);
	//GPIO_ResetBits (GPIOB,GPIO_Pin_4);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_0|GPIO_Pin_6|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_8|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	GPIO_SetBits(GPIOE,GPIO_Pin_5);
	GPIO_SetBits(GPIOE,GPIO_Pin_6);
	GPIO_ResetBits(GPIOC,GPIO_Pin_5);	
	GPIO_SetBits(GPIOE,GPIO_Pin_3);
	
}

void USART_Configuration(void) //USART����
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
	USART_Init(USART1, &USART_InitStructure);//��ʼ������
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //���������ж�,���ռĴ������գ�RXNE=1��ʱ�����ж�
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // ���������ж�,���ͼĴ�����(TXE=1)ʱ�ܲ����ж�
	/*���ڵķ����ж����������ֱ��ǣ�
	l�������ݼĴ������жϣ�TXE�� l
	��������жϣ�TC��*/
	USART_Cmd(USART1, ENABLE); //����USART
	}

void delay_nms(u16 time)//��ʱ�ӳ���
{ 
	u16 i=0;
	while(time--)
	{ i=12000; //�Լ�����
	while(i--) ;
	}
}

//��Ҫ���ô��ڽ����ж��жϣ��ж�ʱ��Ϊ1ms
//------------------------------------------------------------------
//��������void USART1_IRQHandler(void)
//���������null
//���ز�����null
//˵�������ڽ����жϷ���
//------------------------------------------------------------------


void USART1_IRQHandler(void) //����1 �жϷ������
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //�жϽ��ռĴ����Ƿ�ǿ�
	{//����⵽�ж϶���
		RxBuffer[RxCounter++]=USART_ReceiveData(USART1); /* Read one byte from the receive data register */
		if (RxBuffer[RxCounter-1]=='\n') //�����⵽���У������ý��ձ�־Ϊ1.
		// \n:���У���굽�������ף�
		// \r:�س�����굽��������
		{
		Rx_Flag=1;
		TxBuffer[RxCounter]=0; //���ͻ�����������
		}
		if(RxCounter == NbrOfDataToRead) //������ջ��������ˡ�
		{
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); //�رս����ж�
		Rx_Flag=1;
		TxBuffer[RxCounter]=0; //���ͻ�����������
			}
			
		} 
		if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
		{ //����⵽�����ж�
			USART_SendData(USART1, TxBuffer[TxCounter++]); //�������ݼĴ���дһ���ֽ�
			if(TxCounter == NbrOfDataToTransfer)
			{
			TxCounter=0;
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE); /* Disable the USART1 Transmit interrupt */
			}
		}

}
void GP2Y_ADC_init()  
{  
    GPIO_InitTypeDef GPIO_InitStructure;  
    ADC_InitTypeDef ADC_InitStructure;  
  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC3,ENABLE);  
  
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  ���14M ����ADCʱ�ӣ�ADCCLK��  
    ADC_DeInit(ADC3);  
  
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;//ADC  
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //ģ������  
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;  
    GPIO_Init(GPIOF,&GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP; //�������  
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;  
    GPIO_Init(GPIOC,&GPIO_InitStructure);
   
  
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;   
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;   
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;   
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;   
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   
    ADC_InitStructure.ADC_NbrOfChannel = 4;   
    ADC_Init(ADC3, &ADC_InitStructure);  
      
    //����ָ��ADC�Ĺ�����ͨ�����������ǵ�ת��˳��Ͳ���ʱ��  
  ADC_RegularChannelConfig(ADC3,ADC_Channel_4,1,ADC_SampleTime_239Cycles5);  
      
    //�ڲ��¶ȴ���������ADC3ͨ��4�ġ�  
  //  ADC_RegularChannelConfig(ADC3,ADC_Channel_4,1,ADC_SampleTime_7Cycles5);  
//  ADC_TempSensorVrefintCmd(ENABLE);//���ڲ��¶ȴ�����ʹ��  
    ADC_Cmd(ADC3,ENABLE);     
  
    ADC_ResetCalibration(ADC3);//����ָ����ADC��У׼�Ĵ���  
    while(ADC_GetResetCalibrationStatus(ADC3));//��ȡADC����У׼�Ĵ�����״̬  
      
    ADC_StartCalibration(ADC3);//��ʼָ��ADC��У׼״̬  
    while(ADC_GetCalibrationStatus(ADC3));//��ȡָ��ADC��У׼����  
  
    ADC_SoftwareStartConvCmd(ADC3, ENABLE);//ʹ�ܻ���ʧ��ָ����ADC�����ת����������  
    GPIO_SetBits(GPIOC, GPIO_Pin_2);//��1  
	}
u16 GetGP2YAverageValue(u8 times)  
{  
    u32 temp_val=0;  
    u8 t;  
    for(t=0;t<times;t++)  
    {  
        temp_val+=GetGP2YSingleValue();  
    }  
    return temp_val/times;  
}  
  
u16 GetGP2YSingleValue(void)  
{  
        int samplingTime = 280;//�ȴ�LED������ʱ����280��s  
        int deltaTime = 40;//�����������ʱ��Ϊ320��s����ˣ����ǻ����ٵȴ�40��s  
      
        uint16_t ADCVal;  
        float dustVal = 0;  
        float Voltage;  
      
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);//��0  �����ڲ�LED  ????????  
        delay_us(samplingTime);                     // ����LED���280us�ĵȴ�ʱ��  
          
        ADC_SoftwareStartConvCmd(ADC3, ENABLE);  
    while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC));  
        ADCVal = ADC_GetConversionValue(ADC3);  
      
        delay_us(deltaTime);  
        GPIO_SetBits(GPIOC, GPIO_Pin_2);//��1  
        delay_us(9680);//��Ҫ�����0.32ms/10ms��PWM�ź������������е�LED  
        Voltage = ADCVal * 3.3 / 4096;  
        dustVal = (Voltage + 1) * 1000 / 10;//*1000�����ǽ���λת��Ϊug/m3  
	//			GPIO_SetBits(GPIOC, GPIO_Pin_2);
//      if(ADCVal > 36.455)  
//          dustVal = ((float)(ADCVal / 1024.0) - 0.0356) * 120000 * 0.035;  
//      dustVal = 0.17 * ADCVal - 0.1;  
//      dustVal = ADCVal;  
//      return dustVal; //      mg/m3  
        return dustVal;  
}  
