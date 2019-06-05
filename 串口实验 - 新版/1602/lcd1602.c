#include "lcd1602.h"
#include "stm32f10x_gpio.h"
#include "stdio.h"

#define lcm_ce_LOW()    GPIO_ResetBits(GPIOB,GPIO_Pin_0)//E------LCD-Pin6----STM32-PB0
#define lcm_ce_HIGH()   GPIO_SetBits(GPIOB,GPIO_Pin_0)

#define lcm_rst_LOW()   GPIO_ResetBits(GPIOB,GPIO_Pin_1)//RS----LCD-Pin4-----STM32-PB1
#define lcm_rst_HIGH()  GPIO_SetBits(GPIOB,GPIO_Pin_1)

#define lcm_wr_LOW()    GPIO_ResetBits(GPIOB,GPIO_Pin_2)//WR---LCD-Pin5-----STM32-PB2
#define lcm_wr_HIGH()   GPIO_SetBits(GPIOB,GPIO_Pin_2)






//以下是复位或置位数据总线相应位

#define DATA_0_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_8)//复位总线第0位
#define DATA_0_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_8)//置数据总线第0位

#define DATA_1_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_9)//复位总线第1位
#define DATA_1_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_9)//置数据总线第1位

#define DATA_2_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_10)//复位总线第2位
#define DATA_2_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_10)//置数据总线第2位

#define DATA_3_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_4)//复位总线第3位
#define DATA_3_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_4)//置数据总线第3位

#define DATA_4_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_12)//复位总线第4位
#define DATA_4_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_12)//置数据总线第4位

#define DATA_5_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_13)//复位总线第5位
#define DATA_5_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_13)//置数据总线第5位

#define DATA_6_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_14)//复位总线第6位
#define DATA_6_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_14)//置数据总线第6位

#define DATA_7_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_15)//复位总线第7位
#define DATA_7_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_15)//置数据总线第7位
//以上是8位数据总线复位置位

void DATA(unsigned int d)
	//根据数据d的8位二进制来确定对数据总线上哪一位置位或清零
{
      if(d&0x01)  {DATA_0_HIGH();}    else {DATA_0_LOW();}
			if(d&0x02)  {DATA_1_HIGH();}    else {DATA_1_LOW();}
			if(d&0x04)  {DATA_2_HIGH();}    else {DATA_2_LOW();}
			if(d&0x08)  {DATA_3_HIGH();}    else {DATA_3_LOW();}
			if(d&0x10)  {DATA_4_HIGH();}    else {DATA_4_LOW();}
			if(d&0x20)  {DATA_5_HIGH();}    else {DATA_5_LOW();}
			if(d&0x40)  {DATA_6_HIGH();}    else {DATA_6_LOW();}
			if(d&0x80)  {DATA_7_HIGH();}    else {DATA_7_LOW();}
}

void E(unsigned char i)//使能LCD
{
      if(i)    {lcm_ce_HIGH();}       else {lcm_ce_LOW();} 
}

void RS(unsigned char i)//根据参数i确定是置位RS，还是清零RS
{
      if(i)     {lcm_rst_HIGH();}     else  {lcm_rst_LOW();}
}

void RW(unsigned char i)//根据参数i确定是置位RW，还是清零RW
{
      if(i)   {lcm_wr_HIGH();}        else   {lcm_wr_LOW();}
}

void delay(uint a)//毫秒级延时
{
      u16 i=0;
	    while(a--)
			{i=12000;
				while(i--);
			}
}



void enable(uchar del)//1602命令函数，输入的命令值，根据时序来使能LCD
{//根据命令集，按照时序使能LCD
	DATA(del);
   RS(0);//RS=0，选择写指令寄存器；RS=1，选择写数据寄存器
	RW(0);//RW=0,进行写操作；RW=1，进行读操作
	E(0);//E使能位
	delay_ms(10);
	E(1);//使能
	delay_ms(10);
	E(0);
	
}

void write(uchar del)//1602写数据函数，输入的是需要写入1602的数据
{ //根据命令集，按时序来使能LCD
   DATA(del);
   RS(1);//RS=0，选择写指令寄存器；RS=1，选择写数据寄存器
	RW(0);//RW=0,进行写操作；RW=1，进行读操作
	E(0);//E使能位
	delay_ms(10);
	E(1);//使能
	delay_ms(10);
	E(0);
}

void L1602_init(void)//1602初始化
{
     delay_ms(10);
	   enable(0x38);//设置16x2显示，5x7点阵，8位数据接口
	   delay_ms(10);
	   enable(0x06);//地址加1，当写入数据时光标右移
	   delay_ms(10);
	   enable(0x0C);//开显示，不显示光标
	   delay_ms(10);
	   enable(0x01);//清屏
	   delay_ms(10);
	
}

void L1602_Clear(void)//1602清屏
{
   delay_ms(10);
	enable(0x01);//清屏
	delay_ms(10);
}

void L1602_char(uchar row,uchar column,char sign)//按行列位置写字符
{
    uchar a;
	  if(row==1) a=0x80;//在第一行显示
	  if(row==2) a=0xc0;//在第二行显示
	  a=a+column-1;
	  enable(a);
	  write(sign);//写字符sign
}

void L1602_string(uchar row,uchar column,char *p)//从行列位置写字符串
{
   uchar a;
	  if(row==1) a=0x80;//在第一行显示
	  if(row==2) a=0xc0;//在第二行显示
	  a=a+column-1;
	  enable(a);
	  while(*p!='\0')//如果未遇到字符串尾
		{
		    write(*p);
			  p++;
		}
}

void L1602_DispNum(uchar row,uchar column,uint num)
{
   char str[32];
	 sprintf(str,"%d",num);//将整数转换为字符串
	  
	 L1602_string(row,column,str);
}

void L1602_DispFloatNum(uchar row,uchar column,double num)
{
  char str[32];
	 sprintf(str,"%.2f",num);//将浮点数转化为字符串，保留两位小数，第三位四舍五入
	 
	L1602_string(row,column,str);
}

void RCC_Configuration_1602(void)//配置系统时钟
{
  SystemInit();
	//下面给各模块开启时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	//TIM2时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
}

void Port_Init_1602(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	delay(100);
}

void l1602_INIT(void)
{
	  RCC_Configuration_1602();
	  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);//禁用SDJ和JTAG，释放口线用作IO输出
	  Port_Init_1602();
	  L1602_init();
}








