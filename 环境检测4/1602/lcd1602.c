#include "lcd1602.h"
#include "stm32f10x_gpio.h"
#include "stdio.h"

#define lcm_ce_LOW()    GPIO_ResetBits(GPIOB,GPIO_Pin_0)//E------LCD-Pin6----STM32-PB0
#define lcm_ce_HIGH()   GPIO_SetBits(GPIOB,GPIO_Pin_0)

#define lcm_rst_LOW()   GPIO_ResetBits(GPIOB,GPIO_Pin_1)//RS----LCD-Pin4-----STM32-PB1
#define lcm_rst_HIGH()  GPIO_SetBits(GPIOB,GPIO_Pin_1)

#define lcm_wr_LOW()    GPIO_ResetBits(GPIOB,GPIO_Pin_2)//WR---LCD-Pin5-----STM32-PB2
#define lcm_wr_HIGH()   GPIO_SetBits(GPIOB,GPIO_Pin_2)






//�����Ǹ�λ����λ����������Ӧλ

#define DATA_0_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_8)//��λ���ߵ�0λ
#define DATA_0_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_8)//���������ߵ�0λ

#define DATA_1_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_9)//��λ���ߵ�1λ
#define DATA_1_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_9)//���������ߵ�1λ

#define DATA_2_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_10)//��λ���ߵ�2λ
#define DATA_2_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_10)//���������ߵ�2λ

#define DATA_3_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_4)//��λ���ߵ�3λ
#define DATA_3_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_4)//���������ߵ�3λ

#define DATA_4_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_12)//��λ���ߵ�4λ
#define DATA_4_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_12)//���������ߵ�4λ

#define DATA_5_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_13)//��λ���ߵ�5λ
#define DATA_5_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_13)//���������ߵ�5λ

#define DATA_6_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_14)//��λ���ߵ�6λ
#define DATA_6_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_14)//���������ߵ�6λ

#define DATA_7_LOW()      GPIO_ResetBits(GPIOB,GPIO_Pin_15)//��λ���ߵ�7λ
#define DATA_7_HIGH()     GPIO_SetBits(GPIOB,GPIO_Pin_15)//���������ߵ�7λ
//������8λ�������߸�λ��λ

void DATA(unsigned int d)
	//��������d��8λ��������ȷ����������������һλ��λ������
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

void E(unsigned char i)//ʹ��LCD
{
      if(i)    {lcm_ce_HIGH();}       else {lcm_ce_LOW();} 
}

void RS(unsigned char i)//���ݲ���iȷ������λRS����������RS
{
      if(i)     {lcm_rst_HIGH();}     else  {lcm_rst_LOW();}
}

void RW(unsigned char i)//���ݲ���iȷ������λRW����������RW
{
      if(i)   {lcm_wr_HIGH();}        else   {lcm_wr_LOW();}
}

void delay(uint a)//���뼶��ʱ
{
      u16 i=0;
	    while(a--)
			{i=12000;
				while(i--);
			}
}



void enable(uchar del)//1602����������������ֵ������ʱ����ʹ��LCD
{//�������������ʱ��ʹ��LCD
	DATA(del);
   RS(0);//RS=0��ѡ��дָ��Ĵ�����RS=1��ѡ��д���ݼĴ���
	RW(0);//RW=0,����д������RW=1�����ж�����
	E(0);//Eʹ��λ
	delay_ms(10);
	E(1);//ʹ��
	delay_ms(10);
	E(0);
	
}

void write(uchar del)//1602д���ݺ��������������Ҫд��1602������
{ //�����������ʱ����ʹ��LCD
   DATA(del);
   RS(1);//RS=0��ѡ��дָ��Ĵ�����RS=1��ѡ��д���ݼĴ���
	RW(0);//RW=0,����д������RW=1�����ж�����
	E(0);//Eʹ��λ
	delay_ms(10);
	E(1);//ʹ��
	delay_ms(10);
	E(0);
}

void L1602_init(void)//1602��ʼ��
{
     delay_ms(10);
	   enable(0x38);//����16x2��ʾ��5x7����8λ���ݽӿ�
	   delay_ms(10);
	   enable(0x06);//��ַ��1����д������ʱ�������
	   delay_ms(10);
	   enable(0x0C);//����ʾ������ʾ���
	   delay_ms(10);
	   enable(0x01);//����
	   delay_ms(10);
	
}

void L1602_Clear(void)//1602����
{
   delay_ms(10);
	enable(0x01);//����
	delay_ms(10);
}

void L1602_char(uchar row,uchar column,char sign)//������λ��д�ַ�
{
    uchar a;
	  if(row==1) a=0x80;//�ڵ�һ����ʾ
	  if(row==2) a=0xc0;//�ڵڶ�����ʾ
	  a=a+column-1;
	  enable(a);
	  write(sign);//д�ַ�sign
}

void L1602_string(uchar row,uchar column,char *p)//������λ��д�ַ���
{
   uchar a;
	  if(row==1) a=0x80;//�ڵ�һ����ʾ
	  if(row==2) a=0xc0;//�ڵڶ�����ʾ
	  a=a+column-1;
	  enable(a);
	  while(*p!='\0')//���δ�����ַ���β
		{
		    write(*p);
			  p++;
		}
}

void L1602_DispNum(uchar row,uchar column,uint num)
{
   char str[32];
	 sprintf(str,"%d",num);//������ת��Ϊ�ַ���
	  
	 L1602_string(row,column,str);
}

void L1602_DispFloatNum(uchar row,uchar column,double num)
{
  char str[32];
	 sprintf(str,"%.2f",num);//��������ת��Ϊ�ַ�����������λС��������λ��������
	 
	L1602_string(row,column,str);
}

void RCC_Configuration_1602(void)//����ϵͳʱ��
{
  SystemInit();
	//�������ģ�鿪��ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);
	//TIM2ʱ��ʹ��
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
	  GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);//����SDJ��JTAG���ͷſ�������IO���
	  Port_Init_1602();
	  L1602_init();
}








