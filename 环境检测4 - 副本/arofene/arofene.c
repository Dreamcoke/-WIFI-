#include "arofene.h"
int a=0;
u8 ares=1;
u8 aro_low=1;
u8 aro_high=1; 
u16 aro;

int aflag=0;
void GPIO_Configuration_arofene(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}

void RCC_Configuration_arofene(void)
{
	SystemInit();
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
}

void USART_Configuration_arofene(void)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate                 =9600;
	USART_InitStructure.USART_HardwareFlowControl      =USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                     =USART_Mode_Rx;
	USART_InitStructure.USART_Parity                   =USART_Parity_No;
	USART_InitStructure.USART_StopBits                 =USART_StopBits_1;
	USART_InitStructure.USART_WordLength               =USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART1,ENABLE);
}

void NVIC_Configuration_arofene(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	
	NVIC_InitStructure.NVIC_IRQChannel                      =USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd                   =ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    =0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority           =0;
	NVIC_Init(&NVIC_InitStructure);
}

void arofene_USART_Init(void)
{
	GPIO_Configuration_arofene();	
RCC_Configuration_arofene();
USART_Configuration_arofene();
NVIC_Configuration_arofene();
}

void USART1_IRQHandler(void)
{
	
	if(USART_GetITStatus(USART1,USART_IT_RXNE))
	{  
		ares=USART_ReceiveData(USART1);
	//	GPIO_SetBits(GPIOA,GPIO_Pin_5);
		if(ares==0xFF)
		{
			a=-1;
			aflag = 1;
		}
		if(aflag){
			a++;
		}
		
		
		if(a==4)
		{
			aro_high=ares;
		}
		
		if(a==5)
		{
			a=0;
			aflag =0;
			aro_low=ares;
	//		
			aro = ((aro_high<<8)+aro_low);
			
		}




	}
}
