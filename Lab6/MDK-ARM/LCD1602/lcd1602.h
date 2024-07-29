/************************************************************************
 * lcd1602.h
 * Header file for the LCD Driver
 * Created by Xiaoguang Jiang for LAB3
 *DB0~DB7  
 *PA0~PA7= LCD_PORT
 ************************************************************************/



#ifndef __lcd1602_H
#define __lcd1602_H
#include "stm32f1xx_hal.h"


#define LCD_PORT GPIOA->ODR
typedef unsigned char uchar;

/* Function prototypes */

void LCD_init(void);     //LCD初始化相关参数函数
void LCD_Clear(void);    // LCD清屏函数

void LCD_Write_Command(uchar Com); //LCD 写命令函数
void LCD_Write_Data(uchar dat); //LCD 写数据函数

uchar LCD_Read_State(void); //LCD 读状态函数

void LCD_Set_Position(uchar x,uchar y); //LCD 写数据函数

void LCD_ShowNum(unsigned char Line,unsigned char Column,unsigned int Number,unsigned char Length) ;


void LCD_Display_Char(uchar x,uchar y,uchar Char); //LCD显示一个字符函数
void LCD_Display_String(uchar x,uchar y,uchar *str); //LCD显示一个字符串函数

//void Lcd1602_Display_Char(uchar row,uchar col,uchar ch); // LCD显示一个字符函数

#endif

/*****************end of LCD.h**********************/
