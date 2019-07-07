#include <LPC214X.H>
 
 #include<string.h>
 #include<stdio.h>
 #include<math.h>
 #define CLEARSCREEN Lcd_Cmd(0x01)
 //#define Z     (1<<21)
 unsigned char X[5];
 unsigned char Z[5];
 unsigned char Y[5];
int A,B,C;
unsigned char buf[25];
void red();
void delay_s(unsigned int val); 
unsigned char *p,i,j=2,k[6],g[2];
void delete(void);
void Lcd_Cmd(int command);
void delay1(unsigned int delay);
void lcd_data(int data);
void LCD( char *dat, char loc);
int vv=1;
  char   SIM300_AT[]="AT";
  char   SIM300_ATE0[]="ATE0";											  
  char   SIM300_IPR[]="AT+IPR=9600";
  char   SIM300_CMGF[]="AT+CMGF=1";
  char   SIM300_CMGS[]="AT+CMGS=";
  char   SIM300_CMGR[]="AT+CMGR=1";
  char   SIM300_CMGD[]="AT+CMGD=1";
  char   SIM300_ATOK[]="OK";
  char   SIM300_MSGASK[]=">";
  char   SIM300_MSGOK[]="+CMGR: ";
  char   SIM300_SENDOK[]="+CMGS:";
  char   SIM300_READ[]="REC READ";
  char   SIM300_UNREAD[]="REC UNREAD";
  char   SIM300_ERROR[]="ERROR";
  char   a[]="1ON";	 
   char  b[]="1OF";
  char   c[]="2ON";
  char   d[]="2OF";
   char   OVER[1];
   char   SINGLE_QUETES[1];
   char   SEND_MSG[1];
   char   HUICHE[1];
 void ReadMsg();
 void ADC_Init(void);
 unsigned char setbit=0;
 unsigned char REC;
 unsigned char RX_Len;
 char TX_Buf[60];
 char RX_Buf[45];

  char data_len              = 0;
  int TX_Len  = 0;
  int TX_Max_Len             = 0;
  unsigned char Close;
  char  TX_Enable; 
  void Lcd_Init(void);
void lcdcmd(unsigned char command);
//  void lcd_data(unsigned char);
 void lcdstring_disp(unsigned char* string);
 char RX_Buf[45];
 char control_data[16];
 char Out[15];
 char carr                  = 0;
 char  TX_Enable;		
 char RX_Enable;		
 char Ok;
 char Send_Enable;
 char check;
 char count;
 char carr2;
 char data_send;
 char data_send1;
 char message;
 char web_error;
 char GPRS_Enable; 
 int ECG_Sample;
 char data_mode;
 char data_mode1=0;
 unsigned char medicine=0;
/*************** gps***********************/
 char Lat[12] 		    = {"0000.0000"};
 char Log[12] 		    = {"00000.0000"};
 char RX_Buf_GPS[200];
 char Temp_GPS[100];
 char Buf[4];
 volatile static  unsigned char GPS_Data;
 unsigned char RX_Enable_GPS;
 unsigned char Msg_End;
 unsigned char GGA_Enable;
 unsigned char Close;
 unsigned char RMC;
 
// void Lcd_Cmd(unsigned char Cmd);
 void Lcd_Delay(unsigned int del);
 char ISR;
 char Conversion_complete;
 char adc_value;
 char LCD_Buf[16];

void Delay(unsigned long delay);
 void UART0_ISR_GPS(void)__irq
 {
    char aa=0;
	aa=U0IIR;
	if(U0LSR==0x60)
	 {
		if(GPS_Data == 0x52)
			{
				      RX_Len_GPS = 0;
					  RMC=1; 
			}
			else if(RMC&&(GPS_Data !=0X0D)&&(GPS_Data != 0X0A))
			{
			    RX_Buf_GPS[RX_Len_GPS] = GPS_Data;
				RX_Len_GPS++;
			}
		    else  if(GPS_Data==0x0D||GPS_Data==0x0A)
			 {
			   Msg_End = 1;
			   RX_Buf_GPS[RX_Len_GPS]='\0';
			 }
	}
		else if(U0LSR==0x20 )
	   {
				if(TX_Buf[TX_Len]!='\0' && TX_Enable==1)
						U0THR=TX_Buf[TX_Len++];
				else 
						{
								TX_Len=0;
								TX_Enable=0;
						}
	   }	
	   
	VICVectAddr = 0x00000000; 
 }

void UART1(void) __irq
{
   char aa=0;
    if(U1LSR==0x61)
     {
     Close= U1RBR;
	 if(Close != 0x0D && Close != 0x0A)
      {
         RX_Buf[RX_Len] = Close;
         RX_Len=RX_Len+1;
      } 
     if(Close == 0x0D) 
      {
        setbit = 1;
        RX_Len = 0;
        REC++;
      }
	}
    else if(U1LSR==0x20 )
      {
        if(TX_Buf[TX_Len]!='\0' && TX_Enable==1)
        U1THR=TX_Buf[TX_Len++];
        else 
         {
          TX_Len=0;
          TX_Enable=0;
          }     
       } 
	    VICVectAddr = 0x00000000;
   } 

void delay(void)
{
   unsigned long wait;
   for(wait=0;wait<=250000;wait++);
} 

void Delay(unsigned long delay)
{
 while(delay--);
} 

void UART1_Init(void)
{
   U1FCR   = 0x07;
   U1LCR   = 0x83;
   U1DLL   = 0xc3;
   U1DLM   = 0x00;
   U1LCR   = 0x03;      
   U1IER   = 0x03;      // tX intr enable
   PINSEL0|= 0x50000;
   VICIntSelect   = 0x00000000;
   VICIntEnable   = 0x00000080;
   VICVectCntl2    = 0x00000020|7;  
   VICVectAddr2   = (unsigned int) UART1;
}
void UART0_Init(void)
{
    PINSEL0|= 5;
//    APBDIV	= 2;
	U0FCR	= 0x07;
	U0LCR	= 0x83;

	U0DLL	= 0xc3;
	U0DLM	= 0x00;
	U0LCR	= 0x03;		
	U0IER	= 0x03;	   // tX intr enable
    VICIntSelect	= 0x00000000;
	VICIntEnable	= 0x00000040;
	VICVectCntl3 	= 0x00000020|6;  
	VICVectAddr3	= (unsigned int) UART0_ISR_GPS;
}
void Start_TX(void)
{
    TX_Enable = 1;
    TX_Len      = 1;
    TX_Max_Len=strlen(TX_Buf);
    U1THR      = TX_Buf[0];
    while(TX_Enable);
}

/***************************************************************************************/
void delay_s(unsigned int val){ 
 
  T0MR3  = val*29498500; 
  T0EMR &= 0xFFFFFFF7; 
  T0TC   = 0x00000000; 
  T0TCR  =1; 
  while((T0EMR & 0x00000008)==0);  
} 
void ClrRsBuf(char *p,char j)
{   
   unsigned char i;
   
   for(i=0;i<j;i++,p++)
   {
      *p=' ';
   }
}

/*void Delay(unsigned long delay)
{
 while(delay--);
}*/

void Send_AT(void)
{
char *p;
while(1)
{
	sprintf(TX_Buf,"%s",SIM300_AT);
	strcat(TX_Buf,"\r");
    Start_TX();
	if(setbit==1)
	{
    setbit = 0;
    p=strstr(RX_Buf,SIM300_ATOK);
	strcpy(LCD_Buf,&RX_Buf[0]);
CLEARSCREEN;
	LCD(LCD_Buf,0XC0);
//	lcdcmd(0xc0); 
             
	Delay(10000);
      break;
	}
	else
	{

	Delay(1000000);
	Delay(1000000);
	Delay(1000000);
	Delay(100000);
	Delay(1000000);
          Delay(100000);
}

}
}

/*******************************************************************************************/
void Send_ATE0(void)
{
char *p;
while(1)
{
	RX_Len=0;
	sprintf(TX_Buf,"%s",SIM300_ATE0);
	strcat(TX_Buf,"\r");    
  
    Start_TX();
   	if(setbit==1)
	{
    setbit = 0;
    p=strstr(RX_Buf,SIM300_ATOK);
	//print_line(&RX_Buf[0],0xc0);
	strcpy(LCD_Buf,&RX_Buf[0]);
	Lcd_Cmd(0x01);
LCD(LCD_Buf,0xc0);
	//lcdcmd(0xc0); 
	Delay(10000);
   break;
	}
	else
	{
	Delay(1000000);
	Delay(1000000);
	Delay(1000000);
	Delay(100000);
	Delay(1000000);
    Delay(100000);
}
}
}

/*******************************************************************************************/
void SetMode(void)
{
  char *p;
   while(1)
   {
	  sprintf(TX_Buf,"%s",SIM300_CMGF);
	  strcat(TX_Buf,"\r");    
      Start_TX();
	  	if(setbit==1)
	{
    setbit = 0;
    p=strstr(RX_Buf,SIM300_ATOK);
	strcpy(LCD_Buf,&RX_Buf[0]);
	Lcd_Cmd(0x01);
LCD(LCD_Buf,0XC0);
//	lcdcmd(0xc0); 
	Delay(10000);
      break;
	}
	else
	{
	Delay(1000000);
	Delay(1000000);
	Delay(1000000);
	Delay(100000);
	Delay(1000000);
          Delay(100000);
}
}
}
/*******************************************************************************************/
void SIM300_init(void)
{
  	Lcd_Cmd(0x01); 
  LCD("SIM300 init begin!",0x80);
//	lcdcmd(0x80);
   Delay(100);
  // CLEARSCREEN;
  	Lcd_Cmd(0x01);
   Delay(100);
   LCD("AT",0x80);
	//lcdcmd(0x80);
   Send_AT();                  
    LCD("OK",0x80);
//	lcdcmd(0x80);
   Delay(1000);
   //CLEARSCREEN;
	Lcd_Cmd(0x01);
   Delay(1000);
   LCD("ATE0",0x80);
//	lcdcmd(0x80);
   Send_ATE0();            
    LCD("OK",0x80);
//	lcdcmd(0x80);
   Delay(1000);
   //CLEARSCREEN;
  	Lcd_Cmd(0x01);
   Delay(1000);

  LCD("AT+CMGF=1",0x80);
 // lcdcmd(0x80);
   SetMode();               
  LCD("OK",0x80);
  // lcdcmd(0x80);
   Delay(1000);
	Lcd_Cmd(0x01);
   //CLEARSCREEN;
   Delay(1000);
}
/*******************************************************************************************/
void SendMsg(  unsigned char  *MSG)
{
     char *p;
	 unsigned char i=20;
	     RX_Len=0;
	 TX_Buf[0] ='A';
	 TX_Buf[1] ='T';
	 TX_Buf[2] ='+';
	 TX_Buf[3] ='C';
	 TX_Buf[4] ='M';
	 TX_Buf[5] ='G';
	 TX_Buf[6] ='S';
	 TX_Buf[7] ='=';
	 TX_Buf[8] ='"';
	 TX_Buf[9] ='+';
	 TX_Buf[10]='9';
	 TX_Buf[11]='1';
	 TX_Buf[12]='9';
	 TX_Buf[13]='8';
	 TX_Buf[14]='4';
	 TX_Buf[15]='3';
	 TX_Buf[16]='8';
	 TX_Buf[17]='4';
	 TX_Buf[18]='3';
	 TX_Buf[19]='9';
	 TX_Buf[20]='1';
	 TX_Buf[21]='9';
	 TX_Buf[22]='"';
	 TX_Buf[23]='\r';
	// TX_Buf[24]='\r';
	 Start_TX();

    Delay(1000000);
	Delay(100000);
	Delay(1000000);
	Delay(100000);
	Delay(100000);
    Delay(100000);
     while(i--)
     {
	 	 	  	if(setbit==1)
	{
    setbit = 0;
    p=strstr(RX_Buf,SIM300_MSGASK);
	//print_line(&RX_Buf[0],0xc0);
	 strcpy(LCD_Buf,&RX_Buf[0]);
	Lcd_Cmd(0x01);
LCD(LCD_Buf,0xc0);
//	lcdcmd(0xc0); 
	Delay(10000);
	Lcd_Cmd(0x01);
   	}
       p=strstr(RX_Buf,SIM300_MSGASK);
       if(p != NULL)
       {
                  Delay(100); 
     	  sprintf(TX_Buf,"%s",MSG);
		  strcat(TX_Buf,SEND_MSG);
		   Start_TX();
	Delay(1000000);
        	Delay(100000);
        	Delay(1000000);
        	Delay(100000);
        	Delay(100000);
           Delay(100000);
       }
   }
}
void SendMsg1(  unsigned char  *MSG)
{
     char *p;
	 unsigned char i=20;
	 RX_Len=0;
	 TX_Buf[0] ='A';
	 TX_Buf[1] ='T';
	 TX_Buf[2] ='+';
	 TX_Buf[3] ='C';
	 TX_Buf[4] ='M';
	 TX_Buf[5] ='G';
	 TX_Buf[6] ='S';
	 TX_Buf[7] ='=';
	 TX_Buf[8] ='"';
	 TX_Buf[9] ='+';
	 TX_Buf[10]='9';
	 TX_Buf[11]='1';
	 TX_Buf[12]='8';
	 TX_Buf[13]='8';
	 TX_Buf[14]='7';
	 TX_Buf[15]='0';
	 TX_Buf[16]='8';
	 TX_Buf[17]='5';
	 TX_Buf[18]='6';
	 TX_Buf[19]='5';
	 TX_Buf[20]='4';
	 TX_Buf[21]='1';
	 TX_Buf[22]='"';
	 TX_Buf[23]='\r';
	// TX_Buf[24]='\r';
	 Start_TX();
           Delay(1000000);
	Delay(100000);
	Delay(1000000);
	Delay(100000);
	Delay(100000);
    Delay(100000);
     while(i--)
     {
	 	 	  	if(setbit==1)
	{
    setbit = 0;
    p=strstr(RX_Buf,SIM300_MSGASK);
	//print_line(&RX_Buf[0],0xc0);
	 strcpy(LCD_Buf,&RX_Buf[0]);
	Lcd_Cmd(0x01);
LCD(LCD_Buf,0xc0);
	//lcdcmd(0xc0); 
	Delay(10000);
Lcd_Cmd(0x01);
   	}
       p=strstr(RX_Buf,SIM300_MSGASK);
       if(p != NULL)
       {
          Delay(100); 
     	  sprintf(TX_Buf,"%s",MSG);
		  strcat(TX_Buf,SEND_MSG);
		   Start_TX();
           Delay(1000000);
        	Delay(100000);
        	Delay(1000000);
        	Delay(100000);
        	Delay(100000);
          Delay(100000);
       }
   }
}
void SendMsg2(  unsigned char  *MSG)
{
     char *p;
	 unsigned char i=20;
	 TX_Buf[0] ='A';
	 TX_Buf[1] ='T';
	 TX_Buf[2] ='+';
	 TX_Buf[3] ='C';
	 TX_Buf[4] ='M';
	 TX_Buf[5] ='G';
	 TX_Buf[6] ='S';
	 TX_Buf[7] ='=';
	 TX_Buf[8] ='"';
	 TX_Buf[9] ='+';
	 TX_Buf[10]='9';
	 TX_Buf[11]='1';
	 TX_Buf[12]='8';
	 TX_Buf[13]='6';
	 TX_Buf[14]='7';
	 TX_Buf[15]='5';
	 TX_Buf[16]='3';
	 TX_Buf[17]='5';
	 TX_Buf[18]='6';
	 TX_Buf[19]='5';
	 TX_Buf[20]='8';
	 TX_Buf[21]='0';
	 TX_Buf[22]='"';
	 TX_Buf[23]='\r';
	// TX_Buf[24]='\r';
	 Start_TX();

    Delay(1000000);
	Delay(100000);
	Delay(1000000);
	Delay(100000);
	Delay(100000);
    Delay(100000);
     while(i--)
     {
	 	 	  	if(setbit==1)
	{
    setbit = 0;
    p=strstr(RX_Buf,SIM300_MSGASK);
	//print_line(&RX_Buf[0],0xc0);
	 strcpy(LCD_Buf,&RX_Buf[0]);
	Lcd_Cmd(0x01);
	LCD(LCD_Buf,0xc0);
	//lcdcmd(0xc0); 
	Delay(10000);
	Lcd_Cmd(0x01);
   	}
       p=strstr(RX_Buf,SIM300_MSGASK);
       if(p != NULL)
       {
          Delay(100); 
     	  sprintf(TX_Buf,"%s",MSG);
		  strcat(TX_Buf,SEND_MSG);
		   Start_TX();
            Delay(1000000);
        	Delay(100000);
        	Delay(1000000);
        	Delay(100000);
        	Delay(100000);
            Delay(100000);
       }
   }
}
 /*******************************************************************************************/
unsigned char  gps_receiver(void)
{
     while(1)
	    {
		   if(Msg_End)
	       {
             Msg_End = 0;
			 RX_Len_GPS=0;
			strcpy(Temp_GPS,RX_Buf_GPS);
 			if(Temp_GPS[0] == 'M')			
			{
				unsigned char Comm = 0;
				unsigned char i = 0;
				while(Comm <7)
				{
					if(Temp_GPS[i] == 0x2c)   
					{
						Comm += 1;
						if(Comm == 2)
						{
						     	
							Buf[0] = Temp_GPS[++i];
				            Buf[1] = 0;
							if(Buf[0] == 0X41)
							{	
							  RMC_Enable = 1;
							}
							else 
							{
								 RMC_Enable = 0;
						   LCD("    Tracking  ",0x80);
							//lcdcmd(0xc0);
							LCD("   Satellites  ",0xc0);
							}
							i-= 2;
						  // lcdcmd(0x8e);
						LCD(Buf,0x8e);
						}
						i++;
					}
					else
					i++;
				}
				Comm = 0;
				if(RMC_Enable == 1)
				{
					unsigned char Comm=0;
					unsigned char k=0;
					unsigned char k1=0;
					unsigned char i = 0;
					RMC_Enable = 0;
			     	Send_Enable = 1;
					while(Comm < 6)
					{
						if(Temp_GPS[i] == 0x2c)
						{
							Comm += 1;
							i++;
						}
						else
						   i++;
						if(Comm == 3)			
						{
						   
							Lat[k++] = Temp_GPS[i];
						}
						else if(Comm == 5)
						{
							Log[k1++] = Temp_GPS[i];	
						}
					}
					Lat[--k] = 'N';
					Lat[++k] = 0;

					Log[--k1] = 'E';
					Log[++k1] = 0;
				//	CLEARSCREEN; 
				//	lcdcmd(0x80);
				LCD("                ",0x80);
				//	lcdcmd(0x80);
				    LCD("Lat:",0x80);
				    LCD(Lat,0x84);
                   // lcdcmd(0xc0);;
				    LCD("               ",0xc0);
				//	lcdcmd(0xc0);
				    LCD("Log:",0xc0);
				    LCD(Log,0xc4);
					
					return 1;
				}
			}
		}
      }
   
}
void delay1(unsigned int delay)
{
unsigned int i,j;
for(i=0;i<delay;i++)
{
for(j=0;j<65535;j++)
{}
}
} 

void Lcd_Init(void)
{
 IO1DIR=0XFFFFFFFF; 
 Lcd_Cmd(0x02);
 Lcd_Cmd(0x28);
 Lcd_Cmd(0x01);
 Lcd_Cmd(0x06);
 Lcd_Cmd(0x0d);
 Lcd_Cmd(0x0e);
 Lcd_Cmd(0x80);
 Lcd_Cmd(0xc0);
}
/*************************************************************************************/ 
void Lcd_Cmd(int command)
{
int Temp_command,Temp1_command,Temp2_command;
IO1PIN =0X10000;
Temp_command=command;
IO1PIN=Temp_command;
Temp1_command=Temp_command<<24;
IO1PIN=Temp1_command;
Temp1_command=Temp1_command&0XF0000000;
IO1PIN=Temp1_command;
Temp1_command=Temp1_command|0X08000000;
IO1PIN=Temp1_command;
delay1(3);
IO1PIN=Temp1_command^0X08000000;
Temp_command=command;
Temp2_command=Temp_command<<28;
Temp2_command=Temp2_command&0XF0000000;
IO1PIN=Temp2_command;
Temp2_command=Temp2_command|0X08000000;
IO1PIN=Temp2_command;
delay1(3);
IO1PIN=Temp2_command^0x08000000;
IO1PIN = 0X00000;
}
/*************************************************************************************/ 
void lcd_data(int data)
{
 int Temp_data,Temp1_data,Temp2_data;
Temp_data=data;
Temp1_data=Temp_data<<24;
Temp1_data=Temp1_data&0XF0000000;
IO1PIN=Temp1_data;
Temp1_data=Temp1_data|0X0C000000;
IO1PIN=Temp1_data;
delay1(3);
IO1PIN=Temp1_data^0X0C000000;
Temp_data=data;
Temp2_data=Temp_data<<28;
Temp2_data=Temp2_data&0XF0000000;
IO1PIN=Temp2_data;
Temp2_data=Temp2_data|0X0C000000;
IO1PIN=Temp2_data;
delay1(3);
IO1PIN=Temp2_data^0x0C000000;  
}
/*************************************************************************************/ 
 void LCD( char *dat, char loc)
{
 Lcd_Cmd(loc); 
 IO1PIN = 0X20000;
 while(*dat)
 {
   lcd_data(*dat);
   dat++;
  }
   IO1PIN=0X00000;
 }
int main(void)
{
  int A,B,C;
 
	   unsigned int newButtonState;
      int k=0;
	 unsigned char  E[]=40;
	  unsigned char F[]=70;

 	  OVER[0] = 0x0d ;
      SINGLE_QUETES[0] = 0x22;
      SEND_MSG[0] = 0x1a;
      HUICHE[0] = 0x0a;
	  Lcd_Init();
	 ADC_Init();
    
      UART1_Init();
      UART0_Init();
	   IO0DIR=(1<<21)|(1<<17)|(1<<10);
	   IO0CLR=(1<<10);
	   LCD("GSM Based Vehicle",0x80);  
 LCD("Monitoring System",0xC0); 
CLEARSCREEN; 
	 
       Delay(5000);
		 SIM300_init();	 
  while(1)
  {
   	gps_receiver();
	//CLEARSCREEN; 
    A=adc_read(1);	             //P0.28
     sprintf(X,"%d",A);
	  
	 B =adc_read(2);	       //P0.29
     sprintf(Y,"%d",B);

	 C=adc_read(3);			    //P0.30
     sprintf(Z,"%d",C);
		if(A > C)
 {
 if(IO0PIN&(1<<10))
	  {
	    LCD("Person ok",0x80);
		Delay(1000);
        Lcd_Cmd(0x01);
		Delay(1000);
	vv=0;
		}
		if(vv)
	  {
	    	Lcd_Cmd(0x01);
        LCD("FALL",0x80);

	    SIM300_init();
	    strcpy(buf,"FALL");
		 strcat(buf,Lat);
		 strcat(buf,Log);			   
		 SendMsg(buf);
		 delay();		 
		 for(k=0;buf[k]!='\0';k++)
		 {
		 buf[k]=0;
		 }
	  Delay(1000);
	   SIM300_init();
	    strcpy(buf,"FALL");
		 strcat(buf,Lat);
		 strcat(buf,Log);			   
		 SendMsg1(buf);
		 delay();		 
		 for(k=0;buf[k]!='\0';k++)
		 {
		 buf[k]=0;
		 }
	  Delay(1000);
	   SIM300_init();
	    strcpy(buf,"FALL");
		 strcat(buf,Lat);
		 strcat(buf,Log);			   
		 SendMsg2(buf);
		 delay();		 
		 for(k=0;buf[k]!='\0';k++)
		 {
		 buf[k]=0;
		 }
	  Delay(1000);
	  LCD("Monitoring...",0x80);
	  Delay(1000);
       Lcd_Cmd(0x01);
		}
	}
	else
	{
     	Lcd_Cmd(0x01);
	LCD(" NOT FALL",0x80);
	}  
}
}
void ADC_Init(void)
 {
 //PINSEL0 |= ((1<<8)|(1<<9)|(1<<10)|(1<<11));	//AD0.6+AD0.7
  PINSEL1=(1<<18)|(0<<19)|(0<<25)|(0<<27)|(1<<28)|(0<<29);
 VPBDIV  = 2;
}
 int adc_read(char ch)
{
 int i;
 long regVal;
AD0CR = 0x00200D01 | (1<<ch);	     
AD0CR |= 0x01000000;                // Start A/D Conversion
if ( regVal & 0x0000FF00 )	/* check OVERRUN error first */
      {
	    regVal = AD0GDR;
	  }
        do
         {
           i= AD0GDR;                             // Read A/D Data Register
         } while ((i & 0x80000000)!= 0x80000000); // Wait for end of A/D Conversion
		 
		
		 AD0CR &= 0xF8FFFFFF;                     // Stop A/D Conversion
	     return(i=(i >> 6) & 0x03FF);  			  // bit 6:15 is 10 bit AD value
		 }




