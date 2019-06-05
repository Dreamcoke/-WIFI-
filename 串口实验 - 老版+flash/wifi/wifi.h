#ifndef __WIFI_H__
#define __WIFI_H__
 
void USART_Configuration_wifi(void); //USART配置
void GPIO_Configuration_wifi(void); //GPIO配置
void RCC_Configuration_wifi(void); //时钟配置
void USART2_IRQHandler_wifi(void); //串口2 中断服务程序
#endif
