/********************************************************************
//DM430-A�Ϳ�����LCD12864��ʾ��·ADCת�����������ʾ���뵽ADC�ĵ�ѹת�������������û�л��㣬2.5V �ο���ѹ��
//MSP430��Ƭ���ڲ�12λADC�������ڲ�2.5V��׼����ͨ������ת�����ж�ģʽ
//���������ģ���ѹAout���ı�����Ũ�ȿ��Ը��������ѹ�������ʾ��4095(����ת��������ѹ��Ϊ��׼Դ��ѹ-����5V��׼)
//����ܿ��Ʋ���74HC573���������ƣ����ö�̬��ʾ���ƣ����ú���λ�������ʾ
//���Ի�����EW430 V5.30
//���ߣ�sdssgl
//ʱ�䣺2017.05.21
********************************************************************/

#include <msp430x14x.h>
#include "Config.h"                     //����������ͷ�ļ�����Ҫ����IO�˿���Ϣ

static uchar Flag=0;                    //��־����
uint TEMP=0;                            //ADCֵ�ݴ���� 
uchar Temp[16] = 0;
uint results0;
uint results1;
uint results2;
uint results3;

//*************************************************************************
//			��ʼ��IO���ӳ���
//*************************************************************************
void LCD_Port_Init()
{

  P4SEL = 0x00;
  P4DIR = 0xFF;
  P5SEL = 0x00;
  P5DIR|= BIT0 + BIT1 + BIT5 + BIT6 + BIT7;
  PSB_SET;		  //Һ�����ڷ�ʽ
  RST_SET;
}

//***********************************************************************
//	��ʾ������д�뺯��
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
//	��ʾ������д�뺯��
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
//	��ʾ�������ʾ
//***********************************************************************

void LCD_clear(void) 
{
  LCD_write_com(0x01);
  delay_ms(5);
}

//***********************************************************************
//�������ƣ�DisplayCgrom(uchar hz)��ʾCGROM��ĺ���
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
//	��ʾ�����ַ�д�뺯��
//***********************************************************************
void LCD_write_char(unsigned char x,unsigned char y,unsigned char data) 
{
	
    if (y == 0) 
    {
    	LCD_write_com(0x80 + x);        //��һ����ʾ
    }
    if(y == 1) 
    {
    	LCD_write_com(0x90 + x);        //�ڶ�����ʾ
    }
    if (y == 2) 
    {
    	LCD_write_com(0x88 + x);        //��������ʾ
    }
    if(y == 3) 
    {
    	LCD_write_com(0x98 + x);        //��������ʾ
    }
    delay_ms(1);
    LCD_write_data(data);
    delay_ms(1);
}

//***********************************************************************
//	��ʾ���ַ���д�뺯��
//***********************************************************************
void LCD_write_str(unsigned char x,unsigned char y,unsigned char *s) 
{
	
    if (y == 0) 
    {
    	LCD_write_com(0x80 + x);        //��һ����ʾ
    }
    if(y == 1) 
    {
    	LCD_write_com(0x90 + x);        //�ڶ�����ʾ
    }
    if (y == 2) 
    {
    	LCD_write_com(0x88 + x);        //��������ʾ
    }
    if(y == 3) 
    {
    	LCD_write_com(0x98 + x);        //��������ʾ
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
//	��ʾ����ʼ������
//***********************************************************************
void LCD_init(void) 
{
  LCD_write_com(FUN_MODE);			//��ʾģʽ����
  delay_ms(5);
  LCD_write_com(FUN_MODE);			//��ʾģʽ����
  delay_ms(5);
  LCD_write_com(CURSE_DIR);			//��ʾģʽ����
  delay_ms(5);
  LCD_write_com(DISPLAY_ON);			//��ʾ��
  delay_ms(5);
  LCD_write_com(CLEAR_SCREEN);			//����
  delay_ms(5);
}

//***********************************************************************
//      Һ����ʾ�����ʼ��
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
//               MSP430IO�ڳ�ʼ��
//***********************************************************************
void ADC_Port_Init()
{
  LED8SEL  = 0x00;                      //����IO��Ϊ��ͨI/Oģʽ���˾��ʡ
  LED8DIR  = 0xFF;                      //����IO�ڷ���Ϊ���
  LED8PORT = 0xFF;                      //P2�ڳ�ʼ����ΪFF
  
  DATASEL  = 0x00;                      //����IO��Ϊ��ͨI/Oģʽ���˾��ʡ
  DATADIR  = 0xFF;                      //����IO�ڷ���Ϊ���
  DATAPORT = 0xFF;                      //P4�ڳ�ʼ����ΪFF
  
  CTRSEL  =  0x00;                      //����IO��Ϊ��ͨI/Oģʽ���˾��ʡ
  CTRDIR |=  BIT3 + BIT4;               //����IO�ڷ���Ϊ���,���ƿ���P63,P64
  CTRPORT =  0xFF;                      //P6�ڳ�ʼ����ΪFF  
}

//*************************************************************************
//	74HC573��������ܶ�̬ɨ����ʾ��������ʾ�ɼ������¶�
//*************************************************************************

void Display(uint data_q,uint data_b,uint data_s,uint data_g)
{
  uchar i,j;

  j=0x01;                         //��������������λѡ
  for(i=0;i<4;i++)                //�ú�4λ���������ʾ
  {
     DCTR1;                     
     WCTR1;                     
     DATAPORT=~j;                
     WCTR0;                      
     j=(j<<1);
     DATAPORT=0x00;               //ǰ4λ������ʾ��������00����
     DCTR0;                      
     delay_ms(2);                
  }
  
  DCTR1;                          //��ʼ��ʾǧλ
  WCTR1;                    
  DATAPORT=~j;               
  WCTR0;                    
  j=(j<<1);
  DATAPORT=table[A1];         
  DCTR0;                    
  delay_ms(1); 
  
  DCTR1;                          //��ʼ��ʾ��λ
  WCTR1;                    
  DATAPORT=~j;               
  WCTR0;                    
  j=(j<<1);
  DATAPORT=table[A2];         
  DCTR0;                    
  delay_ms(1);              

  DCTR1;                           //��ʼ��ʾʮλ
  WCTR1;                     
  DATAPORT=~j;                
  WCTR0;                      
  j=(j<<1);
  DATAPORT=table[A3];           
  DCTR0;                      
  delay_ms(1);                

  DCTR1;                           //��ʼ��ʾ��λ
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
//		��ѹ���ݴ�����
//*************************************************************************
void Data_do(uint temp_d)
{
  uint temp_1,temp_2;
  A1=temp_d/1000;                       //�����ǧ���٣�ʮ���͸�λ
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
//	ADC��ʼ��������������ADC��ؼĴ���
//*************************************************************************
void ADC_Init()
{
  P6SEL|=0x0f;                                    //ѡ��ADCͨ��
  ADC12CTL0|= ADC12ON + MSC + SHT0_2 + REF2_5V + REFON; //ADC��Դ���ƿ���16��CLK���ڲ���׼2.5V��ADC12CTL0|= ADC12ON + SHT0_2 + REF2_5V +SHP+ REFON;
  ADC12CTL1 = ADC12SSEL1 + ADC12SSEL0;            //SMCLK��ʱ��Դ
  ADC12CTL1 |= SHP+CONSEQ_1;
  ADC12MCTL0= INCH_0;                     //�ο�����λ��ͨ��ѡ������ѡ��ͨ��0
  ADC12MCTL1= INCH_1;                     //�ο�����λ��ͨ��ѡ������ѡ��ͨ��1
  ADC12MCTL2= INCH_2;                     //�ο�����λ��ͨ��ѡ������ѡ��ͨ��2
  ADC12MCTL3= INCH_3+EOS;                     //�ο�����λ��ͨ��ѡ������ѡ��ͨ��3
  ADC12IE|= 0x08;                                 //�ж�����
  ADC12CTL0|= ENC;                        //ʹ��ת����
}

//*************************************************************************
//	ADC�жϷ������
//*************************************************************************
#pragma vector=ADC_VECTOR
__interrupt void ADC12ISR(void)
{
  uchar j;
  while((ADC12CTL1&0x01)==1);           //Ҫ����æ�����ADCæ����ȴ��������ȡADCת����ֵ
  Flag = 1 ;
  
  results0 = ADC12MEM0;
  results1 = ADC12MEM1;
  results2 = ADC12MEM2;
  results3 = ADC12MEM3;
  
  Data_do(results0);
  for(j=0;j<50;j++)
  {
    //Display(A1,A2,A3,A4);               
    LCD_write_str(0x04,0,Temp); //��ʾADC������
  }
  Data_do(results1);
  for(j=0;j<50;j++)
  {
    //Display(A1,A2,A3,A4);               
    LCD_write_str(0x04,1,Temp); //��ʾADC������
  }
  Data_do(results2);
  for(j=0;j<50;j++)
  {
    //Display(A1,A2,A3,A4);               
    LCD_write_str(0x04,2,Temp); //��ʾADC������
  }
  Data_do(results3);
  for(j=0;j<50;j++)
  {
    //Display(A1,A2,A3,A4);              
    LCD_write_str(0x04,3,Temp);  //��ʾADC������
  }

}

//***********************************************************************
//            ������
//***********************************************************************
void main(void)
{ 
  WDT_Init();                         //���Ź���ʼ��
  Clock_Init();                       //ʱ�ӳ�ʼ��
  LCD_Port_Init();
  //ADC_Port_Init();                        //ADC�˿ڳ�ʼ�������ڿ���IO����������
  Close_LED();                        //LED������ȹر�
  ADC_Init();                         //��ʼ��ADC����
  delay_ms(100);                                //��ʱ100ms
  LCD_init();                                   //Һ��������ʼ������
  LCD_clear();                                  //����
  _EINT();                            //ʹ���ж�
  Flag=1;                             //��־λ����1
  LCD_Desk();
  
  while(1)
  {
    while(Flag==1)
    {
      Close_LED();
      ADC12CTL0 |= ADC12SC;           //����ת��
      //ADC12CTL0 &= ~ADC12SC;          //�ֶ����㣬���SHP=1����ʡȥ��仰
      Flag=0;                         //�����־λ
    }
  }
}