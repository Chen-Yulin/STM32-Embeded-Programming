/************************************************************************
 * lcd1602.c
 *
 ************************************************************************/


#include "lcd1602.h"
#include "gpio.h"
#include "Delay.h"

/* initialize the LCD module */
/* define macros for LCD instructions*/
#define LCD_IDLE				0x33
#define LCD_2_LINE_4_BITS		0x28    /*4位显示两行显示模式*/
#define LCD_2_LINE_8_BITS		0x38   /*8位接口两行显示模式*/
#define LCD_DSP_CSR			0x0c     /*开显示，关光标*/
#define LCD_CLR_DSP			0x01   /*清屏，地址指针指向00H*/
#define LCD_CSR_INC			0x06    /*完成一个字符码传送后，光标右移，地址指针自动加1*/
#define LCD_SFT_MOV			0x14	/*光标移动方向,光标向右移动，AC自动加1*/

 /*
* LCD1602初始化
*/

void LCD_init(void)     //LCD初始化函数
{
	LCD_Write_Command(LCD_2_LINE_8_BITS);//显示模式设置,8位显示模式
	Delay_ms(1);//显示模式设置,8位显示模式，两行数据
	LCD_Write_Command(LCD_2_LINE_8_BITS);//显示模式设置,8位显示模式
	LCD_Write_Command(LCD_CLR_DSP);//显示清屏 ，清屏，地址指针指向00H
	//Delay_us(100);
	Delay_ms(1);
	LCD_Write_Command(LCD_CSR_INC);//显示光标右移
	//Delay_us(100);
	Delay_ms(1);
	LCD_Write_Command(LCD_DSP_CSR);//显示开,光标关，光标闪烁关

}
 /*****
* RS:  高电平，表示写的是数据data（LCD屏幕显示数据）; 
	   低电平： 表示写的是命令command（向LCD内部寄存器中数据）;
* R/~W : 低电平， 向LCD1602 写； 
         高电平， 从LCD1602 读
* E : 高电平使能信号
*DB0~DB7 数据总线
*  LCD  RS  RW		意思
*		0	0		写命令
*		0   1       读命令
*		1   0		写数据
*		1   1       读数据
*		
LCD_RS_Pin    数据/命令选择
LCD_RW_Pin    读写选择
LCD_E_Pin;    使能选择

*/

void Lcd1602_Check_Busy(void)
{
	uchar signal;
	HAL_GPIO_WritePin(GPIOB,LCD_RS_Pin, GPIO_PIN_RESET); //LCD_RS = 0;
	HAL_GPIO_WritePin(GPIOB,LCD_RW_Pin, GPIO_PIN_SET);  // LCD_RW = 1;读命令
	
	do
	{
		HAL_GPIO_WritePin(GPIOB,LCD_E_Pin, GPIO_PIN_SET); //LCD_EN = 1;
	//signal = LCD_BUSY;
		HAL_GPIO_WritePin(GPIOB,LCD_E_Pin, GPIO_PIN_RESET);//LCD_EN = 0;
	}while(signal);
}
void LCD_Clear(void)    
{
	LCD_Write_Command(0x01);// clear LCD display
}


/*******************************************************************************
* 函 数 名       : lcd1602_write_cmd
* 函数功能       : LCD1602写命令
* 输    入       : cmd：指令
*关于 E=H 脉冲——开始时初始化 E 为 0，然后置 E 为 1，再清 0.
* 输    出         : 无
*******************************************************************************/

void LCD_Write_Command(uchar CMD)    
{
	//while(LCD_Read_State());
	//判断LCD是否处于繁忙状态，如否就往LCD写指令
	HAL_GPIO_WritePin(GPIOB,LCD_RS_Pin, GPIO_PIN_RESET); // 数据命令选择
	HAL_GPIO_WritePin(GPIOB,LCD_RW_Pin, GPIO_PIN_RESET); //读写选择：
	HAL_GPIO_WritePin(GPIOB,LCD_E_Pin, GPIO_PIN_RESET);  //关闭使能信号，等待
	Delay_ms(1);
	LCD_PORT =(CMD|0xFF00);   //准备命令
	
	HAL_GPIO_WritePin(GPIOB,LCD_E_Pin, GPIO_PIN_SET); //使能脚上升沿写入
	Delay_ms(1); //等待1ms
	HAL_GPIO_WritePin(GPIOB,LCD_E_Pin, GPIO_PIN_RESET);  //关闭使能信号，完成写入
	
}

 /*******************************************************************************
* 函 数 名       : LCD_Write_Data
* 函数功能       : LCD1602写数据
* 输    入       : dat：数据
* 输    出         : 无
*******************************************************************************/
void LCD_Write_Data(uchar dat) 
{
	LCD_PORT = 0x00FF;//初始化PA0~PA7为低电平
	
	HAL_GPIO_WritePin(GPIOB,LCD_RS_Pin, GPIO_PIN_SET); //LCD_RS = 1; 选择数据
	HAL_GPIO_WritePin(GPIOB,LCD_RW_Pin, GPIO_PIN_RESET); //LCD_RW = 0; 选择写
	HAL_GPIO_WritePin(GPIOB,LCD_E_Pin, GPIO_PIN_RESET); //LCD_E_Pin = 0;//关闭使能信号
	
	Delay_ms(1);
	LCD_PORT = (dat|0xFF00); //准备数据
	
	HAL_GPIO_WritePin(GPIOB,LCD_E_Pin, GPIO_PIN_SET);  //Set LCD_E = 1,使能脚E上升沿写入
	Delay_ms(1); //
	HAL_GPIO_WritePin(GPIOB,LCD_E_Pin, GPIO_PIN_RESET);  //Set LCD_E = 0， 关闭使能信号
}


/*
*读数据：D0~D7:
*D7==1:LCD1602繁忙
*D7==0;LCD1602空闲
*/

uchar LCD_Read_State(void) //判断繁忙函数
{
	uchar state =0;
	return state;
}

 
/* ********************
*显示字符前设置光标位置
***********************/

void LCD_Set_Position(uchar row,uchar col)
{
	uint8_t adder;
	if(row ==1)
	{
		adder = 0x80+col;
	}
	 if(row ==2)
	 {
		adder = 0xc0+col;
	 }
	 
	 LCD_Write_Command(adder);
		
}


void LCD_Display_Char(uchar x,uchar y,uchar Char) // 显示字符ASCII码
{
	LCD_Set_Position(x,y);
	LCD_Write_Data(Char);
	
}

///*	@u8 row：行
//*	@u8 col：列
//*	@int ch：字符
//*/
//void Lcd1602_Display_Char(uchar row,uchar col,uchar ch)
//{
//	//显示位置
//	int add = 0;
//	//判断是那一行
//	if(row == 2)
//	{
//		//第二行的首地址 0x40
//		add += 0x40;
//	}
//	//第一行的首地址 0x80
//	add += 0x80 + col - 1;
//	LCD_Write_Command(add) ;  //Lcd1602_Write_Cmd(add);
//	LCD_Write_Data(ch);  //Lcd1602_Write_Data(ch);//显示内容	
//}

/*******************************************************************************
* 函 数 名       : lcd1602_show_string
* 函数功能       : LCD1602显示字符
* 输    入       : x,y：显示坐标，x=0~15，y=0~1;
                   str：显示字符串
* 输    出         : 无
*******************************************************************************/
void LCD_Display_String(uchar x,uchar y,uchar *str)
	{
		LCD_Set_Position(x,y); //当前字符的坐标（调用了坐标显示函数）		  

		while(*str !='\0') 	//‘\0'判断字符串是否结束标志
			{
				//LCD_Write_Data(str[i]); //写入内容对应的ASCII用于显示
				//LCD_Write_Data(*str++);
				//i++;
					
				LCD_Display_Char(x,y,*str);
				y +=1;
				str++;
			}

	}

/*显示数字
* 由于不能直接显示数字，得先转换成字符然后再显示
*  比如传过来的数字是789，那么我们得对789进行转换成字符7，字符8，和字符9再显示
*  先挨个位取出来数字7，数字8，数字9
*789/100=7.89，对7.89取余7.89%10=7（取出来了高位）
*789/10%10=8（取出来第二位）
*789/1%10=9（取出来最低位）
*按照这个方法从高位开始取
 */

int LCD_Pow(int X,int Y)
{
	unsigned char i;
	int Result=1;//如果Y=0，则下面的i<Y直接不成立，直接返回1，符合任何数的0次方等于1的规则
	for(i=0;i<Y;i++)
	{
		Result*=X;
    //如果Y=1，结果等于1*X就等于X的1次方
    //如果Y=2，结果等于1*X*X等于X的2次方
	}
	return Result;//返回值等于x的y次方
}


 /*
* LCD_ShowNum显示不带符号的数字
*
*/


//void LCD_ShowNum(unsigned char Line,unsigned char Column,uint32_t Number,unsigned char Length)
//{
//	unsigned char i;

//	LCD_Set_Position(Line,Column);
//	
//	for(i=Length;i>0;i--)//从高位开始取
//	{
//		LCD_Write_Data('0'+Number/LCD_Pow(10,i-1)%10) ;   //Number/LCD_Pow(10,i-1) 10=X，i-1=Y
//     
//	}
//}

  void LCD_ShowNum(uint8_t x, uint8_t y, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
			
	LCD_Display_Char(Number / LCD_Pow(10, Length - i - 1) % 10 + '0',x,y+i);
				
	}
}

/*************end of LCD.c**************/
