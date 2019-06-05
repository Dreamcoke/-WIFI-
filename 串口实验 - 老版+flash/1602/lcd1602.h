/*  LCD1602.h  */

#ifndef __LCD1602_H
#define __LCD1602_H

#define uchar unsigned char
#define uint unsigned int 

	
void delay(uint a);//��ʱ��������ѭ������
void delay_ms(uint time);//���뼶��ʱ
void delay_ums(uint time);//΢�뼶��ʱ
void enable(uchar del);//ʹ��LCD����ʱ��ĵײ�����
void write(uchar del);//дLCD����ʱ��ĵײ�����
void L1602_init(void );//1602��ʼ��
void L1602_char(uchar row,uchar column,char sign);//������λ��д�ַ�
void L1602_string(uchar row,uchar column,char *p);//������λ��д�ַ���

void L1602_Clear(void);//����
void L1602_DispNum(uchar row,uchar column,uint num);
void Ll602_DispFloatNum(uchar row,uchar column,double num);

#endif
