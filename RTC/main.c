#include "stm32f10x.h"
#include "stdio.h"
#include <string.h>
#include "misc.h" //包含NVIC的固件
#include "misc.c" //包含NVIC的固件
#include "stm32f10x_bkp.h"
#include "stm32f10x_bkp.c"

#include "delay.h"
//void Delay
#define TxBufferSize (countof(TxBuffer) - 1) //定义UART1的发送缓冲器的字节数
#define RxBufferSize 0x51 //定义UART1的接收缓冲器的字节数
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
    //公历年月日周
    vu16 w_year;
    vu8  w_month;
    vu8  w_date;
    vu8  week;     
}_calendar_obj;
_calendar_obj calendar; //时钟结构体
//平均的月份日期表
const u8 mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};
/*rtc中断向量配置*/
int i;
u8 t=0;
u8 Is_Leap_Year(u16 pyear);
u8 RTC_Set(u16 year,u8 mon,u8 day,u8 hour,u8 min,u8 sec);
u8 RTC_Get(void);
u32 r;
char transfer_string[0x78];
void RCC_Configuration(void); //时钟配置
void NVIC_Configuration(void); //NVIC配置
void GPIO_Configuration(void); //GPIO配置
void USART_Configuration(void); //USART配置
void RTC_Configuration(void);
void RTC_Init(void);
u8 RTC_Get(void);
void delay_nms(u16 time);


int main(void)
{
	RCC_Configuration();//配置时钟
	NVIC_Configuration();//配置NVIC
	GPIO_Configuration(); //配置GPIO
	USART_Configuration();//配置USART
	RTC_Configuration();
  RTC_Init();
	
    while(1)
    {
        if(t!=calendar.sec)
        {   
					
            t=calendar.sec;
            //printf("\r\n now is %d 年 %d 月 %d 日 %d 时 %d 分 %d 秒 \r\n ",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
             //r=RTC_Get();
					 sprintf(transfer_string,"now is %d 年 %d 月 %d 日 %d 时 %d 分 %d 秒 \r\n ",calendar.w_year,calendar.w_month,calendar.w_date,calendar.hour,calendar.min,calendar.sec);
				   //sprintf(transfer_string,"now is %d \r\n ",r-3600*8); 
					strcpy((char*)TxBuffer,transfer_string);
					
				}
        //Delay(0x02FFFF);
				USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // 开启发送
				
	delay_nms(1000);
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
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启接收中断,接收寄存器不空（RXNE=1）时产生中断
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // 开启发送中断
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
	  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //设置串口1中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void GPIO_Configuration(void) //GPIO配置
{ /*引脚设置*/
	GPIO_InitTypeDef GPIO_InitStructure;//声明GPIO初始化结构变量。
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //配置管脚PA10/USART1_RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮置输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA10

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9; //配置管脚PA9/USART1_TX
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; //IO口配置为复用输出口
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_0|GPIO_Pin_6|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_8|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	//GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	
}
void RCC_Configuration(void)//时钟配置子程序
{ 
	SystemInit(); //72MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//使能AFIO的时钟，串口属于复用功能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);//使能GPIOE的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1的时钟， USART1挂接到APB2上。其他USART2-5挂接到APB1上
	
}
void USART_Configuration(void) //USART配置
{ /*USART设置*/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //USART_WordLength_8b; //8 位数据 //USART_WordLength_9b; //9 位数据
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	//USART_StopBits_1 ;//在帧结尾传输 1 个停止位
	//USART_StopBits_0.5;//在帧结尾传输 0.5 个停止位
	//USART_StopBits_2 ;//在帧结尾传输 2 个停止位
	//USART_StopBits_1.5;//在帧结尾传输 1.5 个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;
	//USART_Parity_No ;//奇偶失能
	//USART_Parity_Even;//偶模式
	//USART_Parity_Odd ;//奇模式
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	//USART_HardwareFlowControl_None; //硬件流控制失能
	//USART_HardwareFlowControl_RTS; //发送请求 RTS使能
	//USART_HardwareFlowControl_CTS; //清除发送 CTS使能
	//USART_HardwareFlowControl_RTS_CTS;//RTS和 CTS使能
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	//USART_Mode_Tx;//发送使能
	//USART_Mode_Rx;//接收使能
	USART_Init(USART1, &USART_InitStructure);//初始化串口
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启接收中断,接收寄存器不空（RXNE=1）时产生中断
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // 开启发送中断,发送寄存器空(TXE=1)时能产生中断
	/*串口的发送中断有两个，分别是：
	l发送数据寄存器空中断（TXE） l
	发送完成中断（TC）*/
	USART_Cmd(USART1, ENABLE); //启动USART
	}
void USART1_IRQHandler(void) //串口1 中断服务程序
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //判断接收寄存器是否非空
	{//当检测到中断读入
		RxBuffer[RxCounter++]=USART_ReceiveData(USART1); /* Read one byte from the receive data register */
		if (RxBuffer[RxCounter-1]=='\n') //如果检测到换行，则设置接收标志为1.
		// \n:换行，光标到下行行首；
		// \r:回车，光标到本行行首
		{
		Rx_Flag=1;
		TxBuffer[RxCounter]=0; //发送缓冲区结束符
		}
		if(RxCounter == NbrOfDataToRead) //如果接收缓冲区满了。
		{
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE); //关闭接收中断
		Rx_Flag=1;
		TxBuffer[RxCounter]=0; //发送缓冲区结束符
			}
			
		} 
		if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
		{ //当检测到发送中断
			USART_SendData(USART1, TxBuffer[TxCounter++]); //向发送数据寄存器写一个字节
			if(TxCounter == NbrOfDataToTransfer)
			{
			TxCounter=0;
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE); /* Disable the USART1 Transmit interrupt */
			}
		}

}
void RTC_Configuration(void)

{
    /* 使能PWR和BKP时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR|RCC_APB1Periph_BKP,ENABLE);
    /* 使能对后备寄存器的访问 */ 
    PWR_BackupAccessCmd(ENABLE);
    /* 复位BKP寄存器 */ 
    //BKP_DeInit();
    /* 使能LSE */ 
    RCC_LSEConfig(RCC_LSE_ON);
    /*等待启动完成 */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) {}
    /* 将 RTC时钟设置为LSE这个32.768KHZ的晶振*/ 
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
    /* 使能RTC Clock */ 
    RCC_RTCCLKCmd(ENABLE);
    /* 等待同步 */ 
    RTC_WaitForSynchro();
    /* 等待对RTC寄存器最后的写操作完成*/             
    RTC_WaitForLastTask();
    /* 配置了预分频值: 设置RTC时钟周期为1s */
    RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1)*/
    /* 等待对RTC寄存器最后的写操作完成 */
    RTC_WaitForLastTask();
    /* 使能RTC秒中断 */ 
    RTC_ITConfig(RTC_IT_SEC, ENABLE);
    /* 等待对RTC寄存器最后的写操作完成 */         
    RTC_WaitForLastTask();
		}
void RTC_Init(void)
{
    /*如果是第一次配置时钟，则执行RCC_Configuration()进行配置*/
    if(BKP_ReadBackupRegister(BKP_DR1)!=0x1016)
    {
            RCC_Configuration();
            RTC_Set(2017,9,02,14,51,00);
            GPIO_SetBits(GPIOE, GPIO_Pin_1);//点亮D1
            BKP_WriteBackupRegister(BKP_DR1, 0x1016);//向执行的后备寄存器中写入用户程序数据
    }
    else
    {
        RTC_WaitForSynchro();//等待RTC寄存器同步完成
        RTC_ITConfig(RTC_IT_SEC, ENABLE);//使能RTC秒中断
        RTC_WaitForLastTask();//等待最近一次对RTC寄存器的写操作完成
        GPIO_SetBits(GPIOE, GPIO_Pin_2);//点亮D2
    }
    NVIC_Configuration();
		
    RTC_Get();//更新时间
		
}
u8 Is_Leap_Year(u16 pyear)
{
    if(pyear%4==0)//首先需能被4整除
    {
        if(pyear%100==0)
        {
            if(pyear%400==0)    return 1;//如果以00结尾，还要能被400整除
            else    return 0;
        }
        else
            return 1;
    }
    else
        return 0;
}
/*
*设置时钟
*把输入的时钟转换为秒钟
*以1970年1月1日为基准
*1970~2099年为合法年份
返回值：0，成功；其它：错误
*/
u8 RTC_Set(u16 year,u8 mon,u8 day,u8 hour,u8 min,u8 sec)
{
    u16 t;
    u32 secCount=0;
    if(year<1970||year>2099)
        return 1;//3?′í
    for(t=1970;t<year;t++)    //把所有年份的秒钟相加
    {
        if(Is_Leap_Year(t))//闰年
            secCount+=31622400;//闰年的秒钟数
        else
            secCount+=31536000;    
    }
    mon-=1;//先减掉一个月再算秒数（如现在是5月10日，则只需要算前4个月的天数，再加上10天，然后计算秒数）
    for(t=0;t<mon;t++)
    {
        secCount+=(u32)mon_table[t]*86400;//月份秒钟数相加
        if(Is_Leap_Year(year)&&t==1)
            secCount+=86400;//闰年，2月份增加一天的秒钟数
    }
    
    secCount+=(u32)(day-1)*86400;//把前面日期的秒钟数相加（这一天还没过完，所以-1）
    secCount+=(u32)hour*3600;//小时秒钟数
    secCount+=(u32)min*60;//分钟秒钟数
    secCount+=sec;
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR    | RCC_APB1Periph_BKP,ENABLE);
//    PWR_BackupAccessCmd(ENABLE);
    RTC_SetCounter(secCount);//设置RTC计数器的值
    RTC_WaitForLastTask();    //等待最近一次对RTC寄存器的写操作完成
    RTC_Get();//更新时间
    return 0;
}

/*
得到当前的时间
成功返回0，错误返回其它
*/
u8 RTC_Get(void)
{        
        static u16 dayCount=0;
        u32 secCount=0;
        u32 tmp=0;
        u16 tmp1=0;
        secCount=RTC_GetCounter();
        tmp=secCount/86400;//得到天数
        if(dayCount!=tmp)//超过一天
        {    
            dayCount=tmp;
            tmp1=1970;//从1970年开始
            while(tmp>=365)
            {
                if(Is_Leap_Year(tmp1))//是闰年
                {
                    if(tmp>=366)    
                        tmp-=366;//减掉闰年的天数
                    else
                    {
                    //    tmp1++;
                        break;
                    }
                }
                else
                    tmp-=365;//平年
                tmp1++;
            }
            calendar.w_year=tmp1;//得到年份
            tmp1=0;
            while(tmp>=28)//超过一个月
            {
                if(Is_Leap_Year(calendar.w_year)&&tmp1==1)//当年是闰年且轮循到2月
                {
                    if(tmp>=29)    
                        tmp-=29;
                    else
                        break;
                }
                else
                {
                    if(tmp>=mon_table[tmp1])//平年
                        tmp-=mon_table[tmp1];
                    else
                        break;
                }
                tmp1++;
            }
            calendar.w_month=tmp1+1;//得到月份，tmp1=0表示1月，所以要加1
            calendar.w_date=tmp+1;    //得到日期，因为这一天还没过完，所以tmp只到其前一天，但是显示的时候要显示正常日期
        }
        tmp=secCount%86400;//得到秒钟数
        calendar.hour=tmp/3600;//小时
        calendar.min=(tmp%3600)/60;//分钟
        calendar.sec=(tmp%3600)%60;//秒
        return 0;
}
/*
RTC时钟中断
每秒触发一次
*/
void RTC_IRQHandler(void)
{     
    if (RTC_GetITStatus(RTC_IT_SEC) != RESET)//秒钟中断
    {                            
        RTC_Get();//更新时间
      
     }
    if(RTC_GetITStatus(RTC_IT_ALR)!= RESET)//闹钟中断
    {
        RTC_ClearITPendingBit(RTC_IT_ALR);//清闹钟中断        
  }                                                    
    RTC_ClearITPendingBit(RTC_IT_SEC|RTC_IT_OW);//清闹钟中断
    RTC_WaitForLastTask();                                                   
}
void delay_nms(u16 time)//延时子程序
{ 
	u16 i=0;
	while(time--)
	{ i=12000; //自己定义
	while(i--) ;
	}
}
