#include "sds011.h"
int j=0;
u8 res=1;
u8 pm25_low=1;
u8 pm25_high=1; 
u8 pm10_low=1;
u8 pm10_high=1;
u16 pm25;
u16 pm10;
int flag=0;
void GPIO_Configuration_sds011(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void RCC_Configuration_sds011(void)
{
	SystemInit();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
}

void USART_Configuration_sds011(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate                 =9600;
	USART_InitStructure.USART_HardwareFlowControl      =USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                     =USART_Mode_Rx;
	USART_InitStructure.USART_Parity                   =USART_Parity_No;
	USART_InitStructure.USART_StopBits                 =USART_StopBits_1;
	USART_InitStructure.USART_WordLength               =USART_WordLength_8b;
	USART_Init(USART3,&USART_InitStructure);
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART3,ENABLE);
}

void NVIC_Configuration_sds011(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	NVIC_InitStructure.NVIC_IRQChannel                      =USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd                   =ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    =0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority           =0;
	NVIC_Init(&NVIC_InitStructure);
}

void sds011_USART_Init(void)
{
	GPIO_Configuration_sds011();	
RCC_Configuration_sds011();
USART_Configuration_sds011();
NVIC_Configuration_sds011();
}

void USART3_IRQHandler(void)
{
	
	if(USART_GetITStatus(USART3,USART_IT_RXNE))
	{  
		res=USART_ReceiveData(USART3);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_5);
		if(res==0xAA)
		{
			j=-1;
			flag = 1;
		}
		if(flag){
			j++;
		}
		if(j==2)
		{
			pm25_low=res;
		}
		
		if(j==3)
		{
			pm25_high=res;
		}
		
		if(j==4)
		{
			pm10_low=res;
		}
		
		if(j==5)
		{
			j=0;
			flag =0;
			pm10_high=res;
	//		
			pm25 = ((pm25_high<<8)+pm25_low);
			pm10 = (pm10_high<<8)+pm10_low;
		}




	}
}
