#ifndef __FLASH_H  
#define __FLASH_H  
#include "stm32f10x.h"  
u8 Write_Flash(u32 *buff, u8 len);  
void Read_Flash(u32 *buff, u8 len);  
#endif 
