/*
串口通信实例

PA2-----USART2_TX
PA3----USART2_RX

利用USART的中断进行接收和发送。

 用到了串口调试助手。

 注意修改 stm32f10x.h 中HCLK的频率为开发板晶振频率。 例如，若开发板晶振频率为8MHz，则需要定义：
 #define HSE_VALUE ((uint32_t)8000000) // 定义自己开发版上的外部时钟源频率

 */
 #include "stm32f10x.h"
#define TxBufferSize_wifi (countof(TxBuffer_wifi) - 1) //定义UART1的发送缓冲器的字节数
#define RxBufferSize_wifi 0x51 //定义UART1的接收缓冲器的字节数
#define countof(a) (sizeof(a) / sizeof(*(a)))
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "misc.h" //包含NVIC的固件
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
//温湿度
u8 temperature_wifi=0;  	    
u8 humidity_wifi=0;
//pm2.5数值
u16 dustValue_wifi;


void RCC_Configuration_wifi(void)//时钟配置子程序
{ 
	SystemInit(); //72MHz
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//使能AFIO的时钟，串口属于复用功能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//使能GPIOA的时钟
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);//使能GPIOE的时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//使能USART1的时钟， USART1挂接到APB2上。其他USART2-5挂接到APB1上
	
}
void GPIO_Configuration_wifi(void) //GPIO配置
{ /*引脚设置*/
	GPIO_InitTypeDef GPIO_InitStructure;//声明GPIO初始化结构变量。
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //配置管脚PA10/USART1_RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮置输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA10

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2; //配置管脚PA9/USART1_TX
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; //IO口配置为复用输出口
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;//浮置输入
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化PA10
}
void USART_Configuration_wifi(void) //USART配置
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
	USART_Init(USART2, &USART_InitStructure);//初始化串口
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //开启接收中断,接收寄存器不空（RXNE=1）时产生中断
	USART_ITConfig(USART2, USART_IT_TXE, ENABLE); // 开启发送中断,发送寄存器空(TXE=1)时能产生中断
	/*串口的发送中断有两个，分别是：
	l发送数据寄存器空中断（TXE） l
	发送完成中断（TC）*/
	USART_Cmd(USART2, ENABLE); //启动USART
	}
void NVIC_Configuration_wifi(void) //NVIC配置
{ /*配置NVIC相应的优先级位*/
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); //优先级分组1（1:3,1位抢占优先级、 3位子优先级）
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn; //设置串口1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //抢占优先级 0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	NVIC_Init(&NVIC_InitStructure);
}
void USART2_IRQHandler(void) //串口2 中断服务程序
{
	GPIO_SetBits(GPIOE,GPIO_Pin_1);
	if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //判断接收寄存器是否非空
	{//当检测到中断读入
		//RxBuffer_wifi[RxCounter_wifi++]=USART_ReceiveData(USART2); /* Read one byte from the receive data register */
		//if (RxBuffer_wifi[RxCounter_wifi-1]=='\n') //如果检测到换行，则设置接收标志为1.
		// \n:换行，光标到下行行首；
		// \r:回车，光标到本行行首
		//{
		//Rx_Flag_wifi=1;
		//TxBuffer_wifi[RxCounter_wifi]=0; //发送缓冲区结束符
		//}
		//if(RxCounter_wifi == NbrOfDataToRead_wifi) //如果接收缓冲区满了。
		//{
		//USART_ITConfig(USART2, USART_IT_RXNE, DISABLE); //关闭接收中断
		//Rx_Flag_wifi=1;
		//TxBuffer_wifi[RxCounter_wifi]=0; //发送缓冲区结束符
			u8 res = USART_ReceiveData(USART2);
			if(res == '1')
			{
				GPIO_SetBits(GPIOC,GPIO_Pin_5);
			}
			//USART_SendData(USART2, res);
	}
		if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
		{ //当检测到发送中断
			USART_SendData(USART2, TxBuffer_wifi[TxCounter_wifi++]); //向发送数据寄存器写一个字节
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
	

	
	
