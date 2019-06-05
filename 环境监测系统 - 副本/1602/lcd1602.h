/*  LCD1602.h  */

#ifndef __LCD1602_H
#define __LCD1602_H

#define uchar unsigned char
#define uint unsigned int 

	
void delay(uint a);//延时函数，空循环函数
void delay_ms(uint time);//毫秒级延时
void delay_ums(uint time);//微秒级延时
void enable(uchar del);//使能LCD，按时序的底层驱动
void write(uchar del);//写LCD，按时序的底层驱动
void L1602_init(void );//1602初始化
void L1602_char(uchar row,uchar column,char sign);//按行列位置写字符
void L1602_string(uchar row,uchar column,char *p);//从行列位置写字符串

void L1602_Clear(void);//清屏
void L1602_DispNum(uchar row,uchar column,uint num);
void Ll602_DispFloatNum(uchar row,uchar column,double num);

#endif
