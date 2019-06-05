#ifndef __FLASH_H  
#define __FLASH_H  
#include "stm32f10x.h"  
u8 Write_Flash(char *buff, u8 len);  
void Read_Flash(char *buff, u8 len);  
#endif 
