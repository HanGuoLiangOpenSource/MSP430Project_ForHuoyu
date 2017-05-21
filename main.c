/********************************************************************
//DM430-A型开发板LCD12864显示四路ADC转换结果程序，显示输入到ADC的电压转换后的数字量（没有换算，2.5V 参考电压）
//MSP430单片机内部12位ADC，采用内部2.5V基准，多通道单次转换，中断模式
//传感器输出模拟电压Aout，改变气体浓度可以更改输入电压，最大显示量4095(可以转换的最大电压即为基准源电压-此例5V基准)
//数码管控制采用74HC573锁存器控制，采用动态显示机制，采用后四位数码管显示
//调试环境：EW430 V5.30
//作者：sdssgl
//时间：2017.05.21
********************************************************************/

#include <msp430x14x.h>
#include "Config.h"                     //开发板配置头文件，主要配置IO端口信息

static uchar Flag=0;                    //标志变量
uint TEMP=0;                            //ADC值暂存变量 
uchar Temp[16] = 0;
uint results0;
uint results1;
uint results2;
uint results3;

//*************************************************************************
//			初始化IO口子程序
//*************************************************************************
void LCD_Port_Init()
{

  P4SEL = 0x00;
  P4DIR = 0xFF;
  P5SEL = 0x00;
  P5DIR|= BIT0 + BIT1 + BIT5 + BIT6 + BIT7;
  PSB_SET;		  //液晶并口方式
  RST_SET;
}

//***********************************************************************
//	显示屏命令写入函数
//***********************************************************************
void LCD_write_com(unsigned char com) 
{	
  RS_CLR;
  RW_CLR;
  EN_SET;
  DataPort = com;
  delay_ms(5);
  EN_CLR;
}

//***********************************************************************
//	显示屏数据写入函数
//***********************************************************************
void LCD_write_data(unsigned char data) 
{
  RS_SET;
  RW_CLR;
  EN_SET;
  DataPort = data;
  delay_ms(5);
  EN_CLR;
}

//***********************************************************************
//	显示屏清空显示
//***********************************************************************

void LCD_clear(void) 
{
  LCD_write_com(0x01);
  delay_ms(5);
}

//***********************************************************************
//函数名称：DisplayCgrom(uchar hz)显示CGROM里的汉字
//***********************************************************************
void DisplayCgrom(uchar addr,uchar *hz)
{
  LCD_write_com(addr);
  delay_ms(5);
  while(*hz != '\0')  
  {
    LCD_write_data(*hz);
    hz++;
    delay_ms(5);
  }

} 

//***********************************************************************
//	显示屏单字符写入函数
//***********************************************************************
void LCD_write_char(unsigned char x,unsigned char y,unsigned char data) 
{
	
    if (y == 0) 
    {
    	LCD_write_com(0x80 + x);        //第一行显示
    }
    if(y == 1) 
    {
    	LCD_write_com(0x90 + x);        //第二行显示
    }
    if (y == 2) 
    {
    	LCD_write_com(0x88 + x);        //第三行显示
    }
    if(y == 3) 
    {
    	LCD_write_com(0x98 + x);        //第四行显示
    }
    delay_ms(1);
    LCD_write_data(data);
    delay_ms(1);
}

//***********************************************************************
//	显示屏字符串写入函数
//***********************************************************************
void LCD_write_str(unsigned char x,unsigned char y,unsigned char *s) 
{
	
    if (y == 0) 
    {
    	LCD_write_com(0x80 + x);        //第一行显示
    }
    if(y == 1) 
    {
    	LCD_write_com(0x90 + x);        //第二行显示
    }
    if (y == 2) 
    {
    	LCD_write_com(0x88 + x);        //第三行显示
    }
    if(y == 3) 
    {
    	LCD_write_com(0x98 + x);        //第四行显示
    }
    delay_ms(2);
    while (*s) 
    {
    	LCD_write_data( *s);
        delay_ms(2);
    	s ++;
    }
}

//***********************************************************************
//	显示屏初始化函数
//***********************************************************************
void LCD_init(void) 
{
  LCD_write_com(FUN_MODE);			//显示模式设置
  delay_ms(5);
  LCD_write_com(FUN_MODE);			//显示模式设置
  delay_ms(5);
  LCD_write_com(CURSE_DIR);			//显示模式设置
  delay_ms(5);
  LCD_write_com(DISPLAY_ON);			//显示开
  delay_ms(5);
  LCD_write_com(CLEAR_SCREEN);			//清屏
  delay_ms(5);
}

//***********************************************************************
//      液晶显示界面初始化
//***********************************************************************
void LCD_Desk(void)
{    
  LCD_clear();
  DisplayCgrom(0x81,"Gas1:");
  DisplayCgrom(0x91,"Gas2:");
  DisplayCgrom(0x89,"Gas3:");
  DisplayCgrom(0x99,"Gas4:");

  delay_ms(250);
}

//***********************************************************************
//               MSP430IO口初始化
//***********************************************************************
void ADC_Port_Init()
{
  LED8SEL  = 0x00;                      //设置IO口为普通I/O模式，此句可省
  LED8DIR  = 0xFF;                      //设置IO口方向为输出
  LED8PORT = 0xFF;                      //P2口初始设置为FF
  
  DATASEL  = 0x00;                      //设置IO口为普通I/O模式，此句可省
  DATADIR  = 0xFF;                      //设置IO口方向为输出
  DATAPORT = 0xFF;                      //P4口初始设置为FF
  
  CTRSEL  =  0x00;                      //设置IO口为普通I/O模式，此句可省
  CTRDIR |=  BIT3 + BIT4;               //设置IO口方向为输出,控制口在P63,P64
  CTRPORT =  0xFF;                      //P6口初始设置为FF  
}

//*************************************************************************
//	74HC573控制数码管动态扫描显示函数，显示采集到的温度
//*************************************************************************

void Display(uint data_q,uint data_b,uint data_s,uint data_g)
{
  uchar i,j;

  j=0x01;                         //此数据用来控制位选
  for(i=0;i<4;i++)                //用后4位数码管来显示
  {
     DCTR1;                     
     WCTR1;                     
     DATAPORT=~j;                
     WCTR0;                      
     j=(j<<1);
     DATAPORT=0x00;               //前4位都不显示，送数据00即可
     DCTR0;                      
     delay_ms(2);                
  }
  
  DCTR1;                          //开始显示千位
  WCTR1;                    
  DATAPORT=~j;               
  WCTR0;                    
  j=(j<<1);
  DATAPORT=table[A1];         
  DCTR0;                    
  delay_ms(1); 
  
  DCTR1;                          //开始显示百位
  WCTR1;                    
  DATAPORT=~j;               
  WCTR0;                    
  j=(j<<1);
  DATAPORT=table[A2];         
  DCTR0;                    
  delay_ms(1);              

  DCTR1;                           //开始显示十位
  WCTR1;                     
  DATAPORT=~j;                
  WCTR0;                      
  j=(j<<1);
  DATAPORT=table[A3];           
  DCTR0;                      
  delay_ms(1);                

  DCTR1;                           //开始显示个位
  WCTR1;                     
  DATAPORT=~j;                
  WCTR0;                      
  j=(j<<1);
  DATAPORT=table[A4];           
  DCTR0;                      
  delay_ms(1);                
     
  DCTR1;                     
  WCTR1;                     
  DATAPORT=0xff;                
  WCTR0;                      
}

//*************************************************************************
//		电压数据处理函数
//*************************************************************************
void Data_do(uint temp_d)
{
  uint temp_1,temp_2;
  A1=temp_d/1000;                       //分离出千，百，十，和个位
  temp_1=temp_d%1000;
  A2=temp_1/100;
  temp_2=temp_1%100;
  A3=temp_2/10;
  A4=temp_2%10;
  
  Temp[0] = A1+0x30;
  Temp[1] = A2+0x30;
  Temp[2] = A3+0x30;
  Temp[3] = A4+0x30;
}

//*************************************************************************
//	ADC初始化程序，用于配置ADC相关寄存器
//*************************************************************************
void ADC_Init()
{
  P6SEL|=0x0f;                                    //选择ADC通道
  ADC12CTL0|= ADC12ON + MSC + SHT0_2 + REF2_5V + REFON; //ADC电源控制开，16个CLK，内部基准2.5V；ADC12CTL0|= ADC12ON + SHT0_2 + REF2_5V +SHP+ REFON;
  ADC12CTL1 = ADC12SSEL1 + ADC12SSEL0;            //SMCLK做时钟源
  ADC12CTL1 |= SHP+CONSEQ_1;
  ADC12MCTL0= INCH_0;                     //参考控制位及通道选择，这里选择通道0
  ADC12MCTL1= INCH_1;                     //参考控制位及通道选择，这里选择通道1
  ADC12MCTL2= INCH_2;                     //参考控制位及通道选择，这里选择通道2
  ADC12MCTL3= INCH_3+EOS;                     //参考控制位及通道选择，这里选择通道3
  ADC12IE|= 0x08;                                 //中断允许
  ADC12CTL0|= ENC;                        //使能转换器
}

//*************************************************************************
//	ADC中断服务程序
//*************************************************************************
#pragma vector=ADC_VECTOR
__interrupt void ADC12ISR(void)
{
  uchar j;
  while((ADC12CTL1&0x01)==1);           //要先判忙，如果ADC忙，则等待，否则读取ADC转换数值
  Flag = 1 ;
  
  results0 = ADC12MEM0;
  results1 = ADC12MEM1;
  results2 = ADC12MEM2;
  results3 = ADC12MEM3;
  
  Data_do(results0);
  for(j=0;j<50;j++)
  {
    //Display(A1,A2,A3,A4);               
    LCD_write_str(0x04,0,Temp); //显示ADC的数据
  }
  Data_do(results1);
  for(j=0;j<50;j++)
  {
    //Display(A1,A2,A3,A4);               
    LCD_write_str(0x04,1,Temp); //显示ADC的数据
  }
  Data_do(results2);
  for(j=0;j<50;j++)
  {
    //Display(A1,A2,A3,A4);               
    LCD_write_str(0x04,2,Temp); //显示ADC的数据
  }
  Data_do(results3);
  for(j=0;j<50;j++)
  {
    //Display(A1,A2,A3,A4);              
    LCD_write_str(0x04,3,Temp);  //显示ADC的数据
  }

}

//***********************************************************************
//            主程序
//***********************************************************************
void main(void)
{ 
  WDT_Init();                         //看门狗初始化
  Clock_Init();                       //时钟初始化
  LCD_Port_Init();
  //ADC_Port_Init();                        //ADC端口初始化，用于控制IO口输入或输出
  Close_LED();                        //LED数码管先关闭
  ADC_Init();                         //初始化ADC配置
  delay_ms(100);                                //延时100ms
  LCD_init();                                   //液晶参数初始化设置
  LCD_clear();                                  //清屏
  _EINT();                            //使能中断
  Flag=1;                             //标志位先置1
  LCD_Desk();
  
  while(1)
  {
    while(Flag==1)
    {
      Close_LED();
      ADC12CTL0 |= ADC12SC;           //开启转换
      //ADC12CTL0 &= ~ADC12SC;          //手动清零，如果SHP=1，则省去这句话
      Flag=0;                         //清零标志位
    }
  }
}