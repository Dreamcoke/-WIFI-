
/*
串口通信实例

PA9-----USART1_TX
PA10----USART1_RX

利用USART的中断进行接收和发送。

 用到了串口调试助手。

 注意修改 stm32f10x.h 中HCLK的频率为开发板晶振频率。 例如，若开发板晶振频率为8MHz，则需要定义：
 #define HSE_VALUE ((uint32_t)8000000) // 定义自己开发版上的外部时钟源频率

 */


#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "stm32f10x_it.h"
#include "stm32f10x_it.c"
#include "misc.h" //包含NVIC的固件
#include "misc.c" //包含NVIC的固件
#include "dht11.h"
#include <stdio.h>
#include "lcd1602.h"
#include <string.h>
#include "delay.h"
#include "wifi.h"
#include "BH1750.h"
#include "flash.h"
#include "sds011.h"
#include "rtc.h"
#include "arofene.h"

#define TxBufferSize (countof(TxBuffer) - 1) //定义UART1的发送缓冲器的字节数
#define RxBufferSize 0x51 //定义UART1的接收缓冲器的字节数
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
extern u8 ares;
extern u8 aro_low;
extern u8 aro_high;
extern u16 aro;
u8 Tx_Flag=0;
u8 Rx_Flag=0;
u8 flag_a=13;
u8 flag_w=1;
u8 flag_r=0;
char in_data[10]={'a','b','c','d','e'};//要写入的数据  
char out_data[10];//读存放  
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
//温湿度
u8 temperature=0;  	    
u8 humidity=0;
u8 t=0;
//pm2.5数值
u16 dustValue;
void RCC_Configuration(void); //时钟配置
void NVIC_Configuration(void); //NVIC配置
void GPIO_Configuration(void); //GPIO配置
void USART_Configuration(void); //USART配置
void EXTI_Configuration(void);//配置外部中断源
void delay_nms(u16 time);
void WIFI_UART_INIT(void);
void l1602_INIT(void);
void Send_Server(char *msg);
float Get_gy30(void);
u16 GetGP2YAverageValue(u8 times);
unsigned int i;



int main(void)
{ 
RCC_Configuration();//配置时钟
NVIC_Configuration();//配置NVIC
GPIO_Configuration(); //配置GPIO
EXTI_Configuration();//配置外部中断源
sds011_USART_Init();//配置pm模块
arofene_USART_Init();//配置甲醛模块
delay_init();	
WIFI_UART_INIT();
l1602_INIT();
GPIOConfig();	//配置光照模块
Init_BH1750();       //初始化BH1750
RTC_Init();
//GPIO_SetBits(GPIOC,GPIO_Pin_8);
while(DHT11_Init()){
	GPIO_ResetBits(GPIOE,GPIO_Pin_5);
	delay_nms(1000);
	GPIO_SetBits(GPIOE,GPIO_Pin_5);
	delay_nms(1000);
}
GPIO_ResetBits(GPIOE,GPIO_Pin_6);
while(1)
{  
	if(temperature == 0)
		GPIO_ResetBits(GPIOE,GPIO_Pin_5);
	else
		GPIO_SetBits(GPIOE,GPIO_Pin_5);
	flag_a = DHT11_Read_Data(&temperature,&humidity);		//读取温湿度值	flag_a = DHT11_Read_Data(&temperature,&humidity);		//读取温湿度值	
	flag_g=Get_gy30();//读取光照强度
	flag_w=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7);
	
	if(flag_w==0)
	  flag_r=1;//下雨
	else 
		flag_r=0;//晴天
	
	 STATUS=Write_Flash(in_data,5);  
   //Delay(0x02FFFF);  
   if(STATUS)  
    {      
            GPIO_SetBits(GPIOC, GPIO_Pin_11);//点亮led1  
            Read_Flash(out_data,5);  
           // for(i=0;i<5;i++)
			      //    transfer_string[i]=(char)out_data[i];
    }  
    //while(1);  
	//sprintf(transfer_string,"The temperature is %d,The humidity is %d,The pm2.5 is %.2f,The pm10 is %.2f,flag is %x,The is light %f lx,The weather is %d\r\n\0",temperature,humidity,pm25/10.0,pm10/10.0,flag_a,flag_g,flag_r);
	if(t!=calendar.sec)
        {
            t=calendar.sec;
					  sprintf(transfer_string,"now is %d 年 %d 月 %d 日 %d 时 %d 分 %d 秒 \r\n ",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
            //printf("\r\n now is %d 年 %d 月 %d 日 %d 时 %d 分 %d 秒 \r\n ",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
        }
	strcpy((char*)TxBuffer,transfer_string);
	sprintf(transfer_stringWifi,"%d,%d,%.2f,%x,%f,%d,%.2f,%d\r\n\0",temperature,humidity,pm25/10.0,flag_a,flag_g,flag_r,pm10/10.0,aro);
	Send_Server(transfer_stringWifi);
	L1602_Clear();
	sprintf(transfer_string2,"   T: %d  %c%c ",temperature,temperature_unit_1,temperature_unit_2);
	L1602_string(1,1,transfer_string2);
	sprintf(transfer_string3,"   H: %d%% RH",humidity);
	L1602_string(2,1,transfer_string3);
	delay_nms(1000);
	L1602_Clear();
	//delay_nms(500);
	sprintf(transfer_string3,"PM2.5:%.1fug/m3",pm25/10.0);
	L1602_string(1,1,transfer_string3);
	sprintf(transfer_string3,"PM10 :%.1fug/m3",pm10/10.0);
	L1602_string(2,1,transfer_string3);
	delay_nms(1000);
	L1602_Clear();
	sprintf(transfer_string3,"  A: %d  ug/m3",aro);
	L1602_string(1,1,transfer_string3);
	sprintf(transfer_string3,"  L: %.2f lx",flag_g);
	L1602_string(2,1,transfer_string3);
	
	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0))
	{
		GPIO_SetBits(GPIOC,GPIO_Pin_8);
	}
	
	if(humidity>=50||temperature>20)
	{
		GPIO_ResetBits(GPIOE,GPIO_Pin_1);
	  GPIO_SetBits(GPIOE,GPIO_Pin_2);//风扇高电平
		GPIO_ResetBits(GPIOE,GPIO_Pin_3);//蜂鸣器低电平报警
	}
	//if(humidity<80||temperature<30)
	//{
	//	GPIO_ResetBits(GPIOE,GPIO_Pin_2);//风扇低电平
	//  GPIO_SetBits(GPIOE,GPIO_Pin_3);//蜂鸣器高电平
	//}
	//delay(2000);
	//L1602_Clear();
	//L1602_string(1,1,"Hi STM32,");
	//L1602_string(2,1,"Nice to meet you!");
	//delay(2000);
//	snprintf(TxBuffer,sizeof(TxBuffer),"%c\r\n",'c');
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // 开启发送
	delay_nms(500);
	if(Rx_Flag==1) //如果接收到一组以换行结束的数据，则将接收到的数据再发送出去。
	{ //判断是否收到一帧有效数据
	Rx_Flag=0;
	for(i=0; i< NbrOfDataToRead; i++) TxBuffer[i] =0;
	// TxBuffer[0]='\n';
	// TxBuffer[1]='\r';
	for(i=0; i< RxCounter; i++) TxBuffer[i] = RxBuffer[i];
	//TxBuffer[RxCounter]='\r';
	//TxBuffer[RxCounter+1]='\n';
	// TxBuffer[RxCounter+2]='\r';
	//将接收缓冲器的数据转到发送缓冲区，并在缓冲区头上加换行符，准备转发
	RxCounter=0;
	//USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启接收中断,接收寄存器不空（RXNE=1）时产生中断
	//USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // 开启发送中断
//  TxBuffer="for test\r\n";
//	memcpy(TxBuffer,"for test\r\n",11);
	delay_nms(1000);
		}
	
	}
  
	
 }
void RCC_Configuration(void)//时钟配置子程序
{ 
	SystemInit(); //72MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//使能AFIO的时钟，串口属于复用功能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);//使能GPIOE的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1的时钟， USART1挂接到APB2上。其他USART2-5挂接到APB1上
	
}

void EXTI_Configuration(void)//配置外部中断源
{
	EXTI_InitTypeDef EXTI_InitStructure;
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource0);
	EXTI_InitStructure.EXTI_Line=EXTI_Line0;//外部中断线Line0
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;//选择中断模式
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;//上升沿触发
	EXTI_Init(&EXTI_InitStructure);//初始化外部中断	
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;//使能中断
	
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource1);
	EXTI_InitStructure.EXTI_Line=EXTI_Line1;//外部中断线Line1
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;//选择中断模式
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising;//上升沿触发
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;//使能中断
	EXTI_Init(&EXTI_InitStructure);//初始化外部中断	
	
	
}


void NVIC_Configuration(void) //NVIC配置
{ /*配置NVIC相应的优先级位*/
 NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	
    NVIC_InitStructure.NVIC_IRQChannel = RTC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
    NVIC_Init(&NVIC_InitStructure);
}
void GPIO_Configuration(void) //GPIO配置
{ /*引脚设置*/
	GPIO_InitTypeDef GPIO_InitStructure;//声明GPIO初始化结构变量。
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_0|GPIO_Pin_6|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_8|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_2|GPIO_Pin_8|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOC,&GPIO_InitStructure);

	GPIO_SetBits(GPIOE,GPIO_Pin_3);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
}


void delay_nms(u16 time)//延时子程序
{ 
	u16 i=0;
	while(time--)
	{ i=12000; //自己定义
	while(i--) ;
	}
}

//需要设置串口接收中断中断，中断时间为1ms
//------------------------------------------------------------------
//函数名：void USART1_IRQHandler(void)
//输入参数：null
//返回参数：null
//说明：串口接收中断服务
//------------------------------------------------------------------


//void USART1_IRQHandler(void) //串口1 中断服务程序
//{
//	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //判断接收寄存器是否非空
//	{//当检测到中断读入
	//	RxBuffer[RxCounter++]=USART_ReceiveData(USART1); /* Read one byte from the receive data register */
	//	if (RxBuffer[RxCounter-1]=='\n') //如果检测到换行，则设置接收标志为1.
		// \n:换行，光标到下行行首；
		// \r:回车，光标到本行行首
	//	{
	//	Rx_Flag=1;
	//	TxBuffer[RxCounter]=0; //发送缓冲区结束符
		//}
		//if(RxCounter == NbrOfDataToRead) //如果接收缓冲区满了。
		//{
		//USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); //关闭接收中断
	//	Rx_Flag=1;
	//	TxBuffer[RxCounter]=0; //发送缓冲区结束符
		//	}
			
	//	} 
		//if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
		//{ //当检测到发送中断
		//	USART_SendData(USART1, TxBuffer[TxCounter++]); //向发送数据寄存器写一个字节
		//	if(TxCounter == NbrOfDataToTransfer)
		//	{
		//	TxCounter=0;
		//	USART_ITConfig(USART1, USART_IT_TXE, DISABLE); /* Disable the USART1 Transmit interrupt */
		//	}
		//}

//}

void EXTI0_IRQHandler(void)//外部中断源9-5的中断子程序
{   
	//GPIO_SetBits(GPIOC,GPIO_Pin_8);
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)
	{ 
		char str[15] = "id:detection_01";
		Send_Server(str);
		
		EXTI_ClearFlag(EXTI_Line0);
	}
	
}

void EXTI1_IRQHandler(void)//外部中断源9-5的中断子程序
{
	if(EXTI_GetITStatus(EXTI_Line1)!=RESET)
	{
		
		EXTI_ClearFlag(EXTI_Line1);
	}
	
}
