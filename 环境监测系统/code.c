
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

#define TxBufferSize (countof(TxBuffer) - 1) //定义UART1的发送缓冲器的字节数
#define RxBufferSize 0x51 //定义UART1的接收缓冲器的字节数
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
u32 in_data[5]={11,22,33,44,55};//要写入的数据  
u32 out_data[5];//读存放  
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
//pm2.5数值
u16 dustValue;
void RCC_Configuration(void); //时钟配置
void NVIC_Configuration(void); //NVIC配置
void GPIO_Configuration(void); //GPIO配置
void USART_Configuration(void); //USART配置
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
RCC_Configuration();//配置时钟
NVIC_Configuration();//配置NVIC
GPIO_Configuration(); //配置GPIO
USART_Configuration();//配置USART
delay_init();	
GP2Y_ADC_init();
WIFI_UART_INIT();
l1602_INIT();
GPIOConfig();	//配置光照模块
Init_BH1750();       //初始化BH1750
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
	flag_a = DHT11_Read_Data(&temperature,&humidity);		//读取温湿度值	flag_a = DHT11_Read_Data(&temperature,&humidity);		//读取温湿度值	
	flag_g=Get_gy30();//读取光照强度
	flag_w=GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7);
	//fp=fopen("E:\data\data.txt","wt+");
	//fputc(temperature,fp);
	//fclose(fp);
	if(flag_w==0)
	  flag_r=1;//下雨
	else 
		flag_r=0;//晴天
	
	// STATUS=Write_Flash(in_data,5);  
   //Delay(0x02FFFF);  
  //  if(STATUS)  
  //  {       int i;
     //       GPIO_SetBits(GPIOC, GPIO_Pin_11);//点亮led1  
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
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // 开启发送
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
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启接收中断,接收寄存器不空（RXNE=1）时产生中断
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE); // 开启发送中断
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1的时钟， USART1挂接到APB2上。其他USART2-5挂接到APB1上
	
}

void NVIC_Configuration(void) //NVIC配置
{ /*配置NVIC相应的优先级位*/
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //优先级分组1（1:3,1位抢占优先级、 3位子优先级）
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; //设置串口1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
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
	//GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_6;
	//GPIO_Init(GPIOB,&GPIO_InitStructure); //初始化PA9
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
void GP2Y_ADC_init()  
{  
    GPIO_InitTypeDef GPIO_InitStructure;  
    ADC_InitTypeDef ADC_InitStructure;  
  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO|RCC_APB2Periph_ADC3,ENABLE);  
  
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);//12M  最大14M 设置ADC时钟（ADCCLK）  
    ADC_DeInit(ADC3);  
  
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;//ADC  
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AIN; //模拟输入  
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;  
    GPIO_Init(GPIOF,&GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;//
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP; //推挽输出  
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;  
    GPIO_Init(GPIOC,&GPIO_InitStructure);
   
  
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;   
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;   
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;   
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;   
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;   
    ADC_InitStructure.ADC_NbrOfChannel = 4;   
    ADC_Init(ADC3, &ADC_InitStructure);  
      
    //设置指定ADC的规则组通道，设置它们的转化顺序和采样时间  
  ADC_RegularChannelConfig(ADC3,ADC_Channel_4,1,ADC_SampleTime_239Cycles5);  
      
    //内部温度传感器是在ADC3通道4的。  
  //  ADC_RegularChannelConfig(ADC3,ADC_Channel_4,1,ADC_SampleTime_7Cycles5);  
//  ADC_TempSensorVrefintCmd(ENABLE);//打开内部温度传感器使能  
    ADC_Cmd(ADC3,ENABLE);     
  
    ADC_ResetCalibration(ADC3);//重置指定的ADC的校准寄存器  
    while(ADC_GetResetCalibrationStatus(ADC3));//获取ADC重置校准寄存器的状态  
      
    ADC_StartCalibration(ADC3);//开始指定ADC的校准状态  
    while(ADC_GetCalibrationStatus(ADC3));//获取指定ADC的校准程序  
  
    ADC_SoftwareStartConvCmd(ADC3, ENABLE);//使能或者失能指定的ADC的软件转换启动功能  
    GPIO_SetBits(GPIOC, GPIO_Pin_2);//置1  
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
        int samplingTime = 280;//等待LED开启的时间是280μs  
        int deltaTime = 40;//整个脉冲持续时间为320μs。因此，我们还需再等待40μs  
      
        uint16_t ADCVal;  
        float dustVal = 0;  
        float Voltage;  
      
        GPIO_ResetBits(GPIOC, GPIO_Pin_2);//置0  开启内部LED  ????????  
        delay_us(samplingTime);                     // 开启LED后的280us的等待时间  
          
        ADC_SoftwareStartConvCmd(ADC3, ENABLE);  
    while(!ADC_GetFlagStatus(ADC3, ADC_FLAG_EOC));  
        ADCVal = ADC_GetConversionValue(ADC3);  
      
        delay_us(deltaTime);  
        GPIO_SetBits(GPIOC, GPIO_Pin_2);//置1  
        delay_us(9680);//需要脉宽比0.32ms/10ms的PWM信号驱动传感器中的LED  
        Voltage = ADCVal * 3.3 / 4096;  
        dustVal = (Voltage + 1) * 1000 / 10;//*1000作用是将单位转换为ug/m3  
	//			GPIO_SetBits(GPIOC, GPIO_Pin_2);
//      if(ADCVal > 36.455)  
//          dustVal = ((float)(ADCVal / 1024.0) - 0.0356) * 120000 * 0.035;  
//      dustVal = 0.17 * ADCVal - 0.1;  
//      dustVal = ADCVal;  
//      return dustVal; //      mg/m3  
        return dustVal;  
}  
