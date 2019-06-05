#include "stm32f10x.h"
#include "flash.h"
void GPIO_Configuration(void);
void delay(u16 time);
int main(void)
{    
	 //GPIO_Configuration();//GPIO配置，用于点亮led
  //GPIO_SetBits(GPIOB, GPIO_Pin_7);//点亮led1
	
    u32 in_data[5]={11,22,33,44,55};//要写入的数据
    u32 out_data[5];//读存放
    int i;
    u8 STATUS=0;
     //   USART1_Config();//串口1配置
    GPIO_Configuration();//GPIO配置，用于点亮led
    STATUS=Write_Flash(in_data,5);
		 
    delay(1000);
    if(STATUS)
    {
           GPIO_SetBits(GPIOE, GPIO_Pin_2);//点亮led1
            Read_Flash(out_data,5);
//            printf("\r\n The Five Data Is : \r\n");
            for(i=0;i<5;i++)
            {
   //                 printf("\r %d \r",out_data[i]);
            }
    }
    while(1);
}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE); 
 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13 ;            
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;   

  GPIO_Init(GPIOB,&GPIO_InitStructure);
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE); 
 
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;            
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;  
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;   

  GPIO_Init(GPIOA,&GPIO_InitStructure);
}

void delay(u16 time)//延时子程序
{ 
	u16 i=0;
	while(time--)
	{ i=12000; //自己定义
	while(i--) ;
	}
	
}
