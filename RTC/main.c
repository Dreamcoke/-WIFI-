#include "stm32f10x.h"
#include "stdio.h"
#include <string.h>
#include "misc.h" //����NVIC�Ĺ̼�
#include "misc.c" //����NVIC�Ĺ̼�
#include "stm32f10x_bkp.h"
#include "stm32f10x_bkp.c"

#include "delay.h"
//void Delay
#define TxBufferSize (countof(TxBuffer) - 1) //����UART1�ķ��ͻ��������ֽ���
#define RxBufferSize 0x51 //����UART1�Ľ��ջ��������ֽ���
#define countof(a) (sizeof(a) / sizeof(*(a)))
#define RTCClockSource_LSE
u8 TxBuffer[0x78];
u8 RxBuffer[RxBufferSize];
u8 NbrOfDataToTransfer = TxBufferSize;
u8 NbrOfDataToRead = RxBufferSize;
u8 TxCounter = 0;
u16 RxCounter = 0;
u8 Tx_Flag=0;
u8 Rx_Flag=0;

typedef struct 
{
    vu8 hour;
    vu8 min;
    vu8 sec;            
    //������������
    vu16 w_year;
    vu8  w_month;
    vu8  w_date;
    vu8  week;     
}_calendar_obj;
_calendar_obj calendar; //ʱ�ӽṹ��
//ƽ�����·����ڱ�
const u8 mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};
/*rtc�ж���������*/
int i;
u8 t=0;
u8 Is_Leap_Year(u16 pyear);
u8 RTC_Set(u16 year,u8 mon,u8 day,u8 hour,u8 min,u8 sec);
u8 RTC_Get(void);
u32 r;
char transfer_string[0x78];
void RCC_Configuration(void); //ʱ������
void NVIC_Configuration(void); //NVIC����
void GPIO_Configuration(void); //GPIO����
void USART_Configuration(void); //USART����
void RTC_Configuration(void);
void RTC_Init(void);
u8 RTC_Get(void);
void delay_nms(u16 time);


int main(void)
{
	RCC_Configuration();//����ʱ��
	NVIC_Configuration();//����NVIC
	GPIO_Configuration(); //����GPIO
	USART_Configuration();//����USART
	RTC_Configuration();
  RTC_Init();
	
    while(1)
    {
        if(t!=calendar.sec)
        {   
					
            t=calendar.sec;
            //printf("\r\n now is %d �� %d �� %d �� %d ʱ %d �� %d �� \r\n ",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
             //r=RTC_Get();
					 sprintf(transfer_string,"now is %d �� %d �� %d �� %d ʱ %d �� %d �� \r\n ",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
				   //sprintf(transfer_string,"now is %d \r\n ",r-3600*8); 
					strcpy((char*)TxBuffer,transfer_string);
					
				}
        //Delay(0x02FFFF);
				USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // ��������
				
	delay_nms(1000);
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
void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
	  
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	  
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //���ô���1�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
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
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_0|GPIO_Pin_6|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_8|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	//GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	
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
void RTC_Configuration(void)

{
    /* ʹ��PWR��BKPʱ�� */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR|RCC_APB1Periph_BKP,ENABLE);
    /* ʹ�ܶԺ󱸼Ĵ����ķ��� */ 
    PWR_BackupAccessCmd(ENABLE);
    /* ��λBKP�Ĵ��� */ 
    //BKP_DeInit();
    /* ʹ��LSE */ 
    RCC_LSEConfig(RCC_LSE_ON);
    /*�ȴ�������� */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {}
    /* �� RTCʱ������ΪLSE���32.768KHZ�ľ���*/ 
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    /* ʹ��RTC Clock */ 
    RCC_RTCCLKCmd(ENABLE);
    /* �ȴ�ͬ�� */ 
    RTC_WaitForSynchro();
    /* �ȴ���RTC�Ĵ�������д�������*/             
    RTC_WaitForLastTask();
    /* ������Ԥ��Ƶֵ: ����RTCʱ������Ϊ1s */
    RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1)*/
    /* �ȴ���RTC�Ĵ�������д������� */
    RTC_WaitForLastTask();
    /* ʹ��RTC���ж� */ 
    RTC_ITConfig(RTC_IT_SEC, ENABLE);
    /* �ȴ���RTC�Ĵ�������д������� */         
    RTC_WaitForLastTask();
		}
void RTC_Init(void)
{
    /*����ǵ�һ������ʱ�ӣ���ִ��RCC_Configuration()��������*/
    if(BKP_ReadBackupRegister(BKP_DR1)!=0x1016)
    {
            RCC_Configuration();
            RTC_Set(2017,9,02,14,51,00);
            GPIO_SetBits(GPIOE, GPIO_Pin_1);//����D1
            BKP_WriteBackupRegister(BKP_DR1, 0x1016);//��ִ�еĺ󱸼Ĵ�����д���û���������
    }
    else
    {
        RTC_WaitForSynchro();//�ȴ�RTC�Ĵ���ͬ�����
        RTC_ITConfig(RTC_IT_SEC, ENABLE);//ʹ��RTC���ж�
        RTC_WaitForLastTask();//�ȴ����һ�ζ�RTC�Ĵ�����д�������
        GPIO_SetBits(GPIOE, GPIO_Pin_2);//����D2
    }
    NVIC_Configuration();
		
    RTC_Get();//����ʱ��
		
}
u8 Is_Leap_Year(u16 pyear)
{
    if(pyear%4==0)//�������ܱ�4����
    {
        if(pyear%100==0)
        {
            if(pyear%400==0)    return 1;//�����00��β����Ҫ�ܱ�400����
            else    return 0;
        }
        else
            return 1;
    }
    else
        return 0;
}
/*
*����ʱ��
*�������ʱ��ת��Ϊ����
*��1970��1��1��Ϊ��׼
*1970~2099��Ϊ�Ϸ����
����ֵ��0���ɹ�������������
*/
u8 RTC_Set(u16 year,u8 mon,u8 day,u8 hour,u8 min,u8 sec)
{
    u16 t;
    u32 secCount=0;
    if(year<1970||year>2099)
        return 1;//3?�䨪
    for(t=1970;t<year;t++)    //��������ݵ��������
    {
        if(Is_Leap_Year(t))//����
            secCount+=31622400;//�����������
        else
            secCount+=31536000;    
    }
    mon-=1;//�ȼ���һ����������������������5��10�գ���ֻ��Ҫ��ǰ4���µ��������ټ���10�죬Ȼ�����������
    for(t=0;t<mon;t++)
    {
        secCount+=(u32)mon_table[t]*86400;//�·����������
        if(Is_Leap_Year(year)&&t==1)
            secCount+=86400;//���꣬2�·�����һ���������
    }
    
    secCount+=(u32)(day-1)*86400;//��ǰ�����ڵ���������ӣ���һ�컹û���꣬����-1��
    secCount+=(u32)hour*3600;//Сʱ������
    secCount+=(u32)min*60;//����������
    secCount+=sec;
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR    | RCC_APB1Periph_BKP,ENABLE);
//    PWR_BackupAccessCmd(ENABLE);
    RTC_SetCounter(secCount);//����RTC��������ֵ
    RTC_WaitForLastTask();    //�ȴ����һ�ζ�RTC�Ĵ�����д�������
    RTC_Get();//����ʱ��
    return 0;
}

/*
�õ���ǰ��ʱ��
�ɹ�����0�����󷵻�����
*/
u8 RTC_Get(void)
{        
        static u16 dayCount=0;
        u32 secCount=0;
        u32 tmp=0;
        u16 tmp1=0;
        secCount=RTC_GetCounter();
        tmp=secCount/86400;//�õ�����
        if(dayCount!=tmp)//����һ��
        {    
            dayCount=tmp;
            tmp1=1970;//��1970�꿪ʼ
            while(tmp>=365)
            {
                if(Is_Leap_Year(tmp1))//������
                {
                    if(tmp>=366)    
                        tmp-=366;//�������������
                    else
                    {
                    //    tmp1++;
                        break;
                    }
                }
                else
                    tmp-=365;//ƽ��
                tmp1++;
            }
            calendar.w_year=tmp1;//�õ����
            tmp1=0;
            while(tmp>=28)//����һ����
            {
                if(Is_Leap_Year(calendar.w_year)&&tmp1==1)//��������������ѭ��2��
                {
                    if(tmp>=29)    
                        tmp-=29;
                    else
                        break;
                }
                else
                {
                    if(tmp>=mon_table[tmp1])//ƽ��
                        tmp-=mon_table[tmp1];
                    else
                        break;
                }
                tmp1++;
            }
            calendar.w_month=tmp1+1;//�õ��·ݣ�tmp1=0��ʾ1�£�����Ҫ��1
            calendar.w_date=tmp+1;    //�õ����ڣ���Ϊ��һ�컹û���꣬����tmpֻ����ǰһ�죬������ʾ��ʱ��Ҫ��ʾ��������
        }
        tmp=secCount%86400;//�õ�������
        calendar.hour=tmp/3600;//Сʱ
        calendar.min=(tmp%3600)/60;//����
        calendar.sec=(tmp%3600)%60;//��
        return 0;
}
/*
RTCʱ���ж�
ÿ�봥��һ��
*/
void RTC_IRQHandler(void)
{     
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET)//�����ж�
    {                            
        RTC_Get();//����ʱ��
      
     }
    if(RTC_GetITStatus(RTC_IT_ALR)!= RESET)//�����ж�
    {
        RTC_ClearITPendingBit(RTC_IT_ALR);//�������ж�        
  }                                                    
    RTC_ClearITPendingBit(RTC_IT_SEC|RTC_IT_OW);//�������ж�
    RTC_WaitForLastTask();                                                   
}
void delay_nms(u16 time)//��ʱ�ӳ���
{ 
	u16 i=0;
	while(time--)
	{ i=12000; //�Լ�����
	while(i--) ;
	}
}
