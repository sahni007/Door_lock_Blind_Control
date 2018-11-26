/*
 * this code is for one curtain 15seconds timer//tim,er1
 * and blynd 28 seconds timer//timer3
 */

#include <stdio.h>
#include <stdlib.h>

// PIC16F1526 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = HS        // Oscillator Selection (HS Oscillator, High-speed crystal/resonator connected between OSC1 and OSC2 pins)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (VCAP pin function disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#define _XTAL_FREQ 16000000

#define OPEN_1 RF1
#define CLOSE_1 RF0
#define OPEN_2 RA3
#define CLOSE_2 RA2
//#define FAN RE5

#define OPEN_INPUT_1 RF7//RA5
#define CLOSE_INPUT_1 RF5//RF2
#define OPEN_INPUT_2 RF3
#define CLOSE_INPUT_2 RF2
//#define SW5 RA5

#define REQUIRED_CURTAIN_DELAY_IN_SECONDS 30000 
#define CURTAIN_COUNTER (REQUIRED_CURTAIN_DELAY_IN_SECONDS / 125)

int curtFlag1=0, curtFlag2=0, curtFlag;
int i=0,j=0,cnt=0,FAN_SPEED=0,FAN_SPEED1=0;
int FAN_FLAG=0, TX_FLAG=0;
int len2=0, len1=0;
int sum=0, man=1;

unsigned int child_LOCK[32]="00000000000000000000000000000000";
unsigned char st[50]="TTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT";
unsigned char name[36]="a";
unsigned char copy[36]="$";
//char parents[32]="p";
int k=0,sw=0,COPY_FLAG=0;
unsigned int M1;unsigned int M2;unsigned int M3;unsigned int M4;
int TimerCounter1=0,TimerCounter2=0;
#define ONN 1
#define OFF 0
void writeUART(char *str2Write);
interrupt void isr(){     
    if(PIE3bits.TMR3IE==1 && PIR3bits.TMR3IF==1)
    {           
        PIR3bits.TMR3IF=0;
        if(TimerCounter2>=224){
        OPEN_2=0;           CLOSE_2=0;       // TX1REG='Q';
        st[9]='G';				st[10]='0';                st[11]='0';                st[12]='3';            writeUART(st+9);
        T3CONbits.TMR3ON=0;
        st[13]='G';             st[14]='0';                st[15]='0';                st[16]='4';            writeUART(st+13);
        }
        else if(curtFlag2){
        TimerCounter2=TimerCounter2+1;
        TMR3H=0x0B;        TMR3L=0xDC;        T3CONbits.TMR3ON = 1;
        //TX1REG='p';
        }        
    }
    
    
    if(PIE1bits.TMR1IE == 1 && PIR1bits.TMR1IF==1)
    {
        PIR1bits.TMR1IF=0;        //TX1REG='T';   
		if(TimerCounter1>=125){
        OPEN_1=0;           CLOSE_1=0;       // TX1REG='Q';
        st[1]='G';				st[2]='0';                  st[3]='0';                st[4]='1';            writeUART(st+1);
        T1CONbits.TMR1ON = 0;
        st[5]='G';                st[6]='0';                st[7]='0';                st[8]='2';            writeUART(st+5);
        }
        else if(curtFlag1){
        TimerCounter1=TimerCounter1+1;
        TMR1H=0x0B;        TMR1L=0xDC;        T1CONbits.TMR1ON = 1;
        //TX1REG='p';
        }
    }
    // ************************************* UART *********************************************** //
    // ************************************* UART 1 XBEE*********************************************** //
        if(RC1IF==1){
            if(RC1STAbits.OERR) // If over run error, then reset the receiver
            {                
                RC1STAbits.CREN = 0;                RC1STAbits.CREN = 1;                
                while(PIR1bits.TXIF==0);                TX1REG='F';                while(PIR1bits.TXIF==0);                
            }
            name[i]=RC1REG;
            //TX1REG=name[i];
            if(name[0]=='%')
            {
                i++;                
                if(i>15)
                {
                    i=0;    TX_FLAG = 1;   RC1IF=0;        //TX1REG='X';
                }               
            }
            else
            {                
                i=0;        RC1STAbits.CREN = 0;                RC1STAbits.CREN = 1;
                while(PIR1bits.TX1IF==0);                 TX1REG='F';
                while(PIR1bits.TX1IF==0);                 TX1REG='R';
                while(PIR1bits.TX1IF==0);                 TX1REG='R';
            }                
        }        
}

void ACTION(char Switch_Num_10s, char Switch_Num_1s, char sw_status, char speed_bit1, char speed_bit2, char parent,char finalFrameStat);
void pin_manager();

void TMR3_Initialize();
void TMR1_Initialize();

void EUSART_Initialize();
void copy_frame(int start,int end);


void periperal_init(){
      EUSART_Initialize();
      TMR1_Initialize();
      TMR3_Initialize();
    }

void main() {
    __delay_ms(2000);
//    __delay_ms(10000);
    unsigned int frame_start = 0, frame_end = 0,cnt1=0,cnt2=0;
    
    int RX_CHK_FLAG_start1 = 0 ,RX_CHK_FLAG_start2 = 0, start_flag=0, RX_CHK_FLAG_end1 = 0, RX_CHK_FLAG_end2 = 0, end_flag=0;
    
    pin_manager();
    periperal_init();
    
    OPEN_1 = OFF;    CLOSE_1 = OFF;      OPEN_2 = OFF;    CLOSE_2 = OFF;  
    M1=ONN;    M2=ONN;    M3=ONN;    M4=ONN;
    st[0]='%';    st[10]='@';
       
    while(1){
        if(TX_FLAG==1)
        {
//           TX1REG='X';            // %%0310000@@%%0310000@@
            TX_FLAG=0;
            start_flag = 0;
            end_flag = 0;
            
            if(name[0]=='%' && name[1]=='%' && name[14]=='@' && name[15]=='@')
            {
                for(k=0;k<16;k++)
                {
                    if(name[k] == '%' && name[k+1] == '%' && start_flag == 0)
                    {
                        RX_CHK_FLAG_start1 = k;
                        start_flag = 1;
                    }
                    else if(name[k] == '%' && name[k+1] == '%' && start_flag == 1)
                    {
                        RX_CHK_FLAG_start2 = k;
                    }
                    else if(name[k] == '@' && name[k+1] == '@' && end_flag == 0)
                    {
                        RX_CHK_FLAG_end1 = k;
                        end_flag = 1;
                        COPY_FLAG=1;
                        break;
                    }
                    else if(name[k] == '@' && name[k+1] == '@' && end_flag == 1)
                    {
                        RX_CHK_FLAG_end2 = k;
                        COPY_FLAG=2;
                        break;
                    }
                }
            }
            else
            {
                __delay_ms(10);                TX1REG='P';                __delay_ms(1);               TX1REG='K';                __delay_ms(1);   
                i=0;
                RC1STAbits.SPEN=0;                RC1STAbits.SPEN=1;               
                sw=0;
                for(k = 0; k< 15; k++)
                {
                    name[k] = '#';
                }   
                COPY_FLAG=0;                    
            }
       
                        
            if(COPY_FLAG==1)            // %% OPEN_INPUT_10 OPEN_INPUT_1 state speed10s speed1s childLok ackFlag res res res res parity @ @
            {
                copy_frame(RX_CHK_FLAG_start1,RX_CHK_FLAG_end1);
                //G_switching();
                // FORMAT -----> ACTION(switch_num_bit_10s, Switch_Num_bit_1s, sw_status, speed_bit1, speed_bit2, parent, finalFrameStat )
                ACTION(copy[2], copy[3], copy[4], copy[5], copy[6], copy[7],copy[8]);
            }
            else if(COPY_FLAG==2)
            {     
                copy_frame(RX_CHK_FLAG_start2,RX_CHK_FLAG_end2);    
                ACTION(copy[2], copy[3], copy[4], copy[5], copy[6], copy[7],copy[8]);
                //G_switching();
            }
        
       }// end of if(TX_FLAG==1)


/*manual Response started */
        
            if(child_LOCK[1]==OFF && OPEN_INPUT_1==OFF && M1==OFF)
			{     
                if(man==1)
                {
                st[1]='R';                st[2]='0';                st[3]='0';                st[4]='1';                writeUART(st+1);
                CLOSE_1=OFF;              OPEN_1=OFF;				curtFlag1=0;            TimerCounter1=0;
                }
				M1=1;                man=1;
			}
	   
			if(child_LOCK[1]==OFF && OPEN_INPUT_1==ONN && M1==ONN)
			{          
                if(man==1)
                {
                st[5]='R';                st[6]='0';                st[7]='0';                st[8]='2';                writeUART(st+5);
                CLOSE_1=OFF;
                st[1]='R';                st[2]='1';                st[3]='0';                st[4]='1';                writeUART(st+1);                                
                OPEN_1=ONN;               curtFlag1=1;              TimerCounter1=0;
                TMR1H=0x0B;               TMR1L=0xDC;               PIR1bits.TMR1IF=0;         T1CONbits.TMR1ON = 1;
                }
                M1=0;                man=1;                
			}
       ///// ********************************************* 222222222///
            if(child_LOCK[3]==OFF && CLOSE_INPUT_1==OFF && M2==OFF)
			{    
                if(man==1){
                st[5]='R';                st[6]='0';                st[7]='0';                st[8]='2';
                writeUART(st+5);
                CLOSE_1=OFF;              OPEN_1=OFF;				curtFlag1=0;              TimerCounter1=0;
				}
                M2=1;                man=1;
			}
       
            if(child_LOCK[3]==OFF && CLOSE_INPUT_1==ONN && M2==ONN)
			{     
                if(man==1){
                st[1]='R';                st[2]='0';                st[3]='0';                st[4]='1';                writeUART(st+1);
                OPEN_1=OFF;
                st[5]='R';                st[6]='1';                st[7]='0';                st[8]='2';                writeUART(st+5);                                
                CLOSE_1=ONN;              curtFlag1=1;              TimerCounter1=0;
                TMR1H=0x0B;               TMR1L=0xDC;               PIR1bits.TMR1IF=0;         T1CONbits.TMR1ON = 1;
                }
                M2=0;                man=1;
			}        
        
        ///// ********************************************* 333333333333 ///
        if(child_LOCK[5]==OFF && OPEN_INPUT_2==OFF && M3==OFF)
			{    
                if(man==1)
                {
                st[9]='R';                st[10]='0';               st[11]='0';                st[12]='3';                writeUART(st+9);
                CLOSE_2=OFF;              OPEN_2=OFF;				curtFlag2=0;               TimerCounter2=0;
                }
				M3=1;                man=1;
			}
	   
			if(child_LOCK[5]==OFF && OPEN_INPUT_2==ONN && M3==ONN)
			{                 
                if(man==1)
                {
                st[13]='R';               st[14]='0';               st[15]='0';                st[16]='4';               writeUART(st+13);
                CLOSE_2=OFF;
                st[9]='R';                st[10]='1';               st[11]='0';                st[12]='3';                writeUART(st+9);                                
                OPEN_2=ONN;               curtFlag2=1;              TimerCounter2=0;
                TMR3H=0x0B;               TMR3L=0xDC;               PIR3bits.TMR3IF=0;         T3CONbits.TMR3ON = 1;   
                }
                M3=0;                man=1;                
			}
       
        ///// ********************************************* 444444444444444444444///
            if(child_LOCK[7]==OFF && CLOSE_INPUT_2==OFF && M4==OFF)
			{  
                if(man==1){
                st[13]='R';                st[14]='0';              st[15]='0';                st[16]='4';                writeUART(st+13);
                CLOSE_2=OFF;               OPEN_2=OFF;				curtFlag2=0;               TimerCounter2=0;
				}                
                M4=1;                man=1;
			}
       
            if(child_LOCK[7]==OFF && CLOSE_INPUT_2==ONN && M4==ONN)
			{   
                if(man==1)
                {
                st[9]='R';                st[10]='0';               st[11]='0';                st[12]='3';                writeUART(st+9);
                OPEN_2=OFF;
                st[13]='R';                st[14]='1';              st[15]='0';                st[16]='4';               writeUART(st+13);                                
                CLOSE_2=ONN;                curtFlag2=1;            TimerCounter2=0;
                TMR1H=0x0B;                 TMR1L=0xDC;             PIR3bits.TMR3IF=0;         T3CONbits.TMR3ON = 1;   
                }
                M4=0;                man=1;
			}
    }
  //  }
}

void copy_frame(int start, int end){
    sw=0;
    for(k = start; k< end; k++)
    {
        copy[sw]=name[k];
        sw++;
        name[k] = '#';
    }
//    TX1REG=' ';
//    while(PIR1bits.TXIF==0);
//    TX1REG=' ';
//    while(PIR1bits.TXIF==0);   
    COPY_FLAG=0;    
}

void ACTION(char Switch_Num_10s, char Switch_Num_1s, char sw_status, char speed_bit1, char speed_bit2, char parent,char finalFrameStat)
{
    int response_starts=0;
    int switch_status=0;
    int SwNum=0;
    
    if(Switch_Num_1s != 'T')
    {
    unsigned char FanSpeedString[2], SwNumString[2];
    int FanSpeed=0;
    
    switch_status=sw_status - '0';
       
    SwNumString[0]=Switch_Num_10s;
    SwNumString[1]=Switch_Num_1s;
    SwNum=atoi(SwNumString);
    
    FanSpeedString[0] = speed_bit1;
    FanSpeedString[1] = speed_bit2;
    FanSpeed = atoi(FanSpeedString);
    
    int children=parent - '0';    
    int child_lock_num=(2*(SwNum)-1); // position of child_locked frame
    child_LOCK[child_lock_num]=children;

    response_starts=((1+4*(SwNum))-4);
    st[response_starts++]='G';
    st[response_starts++]=sw_status;
    st[response_starts++]=Switch_Num_10s;
    st[response_starts]=Switch_Num_1s;    
    
    response_starts-=3;
    if(finalFrameStat=='1')
    {
    writeUART(st+response_starts);
    }
    man=0;
    }
    
    switch(SwNum)       // char SwNum
    {
        case 1:// backward
            M1=switch_status;
            if(switch_status==1){
                CLOSE_1=OFF;
                st[5]='G';                st[6]='0';                st[7]='0';                st[8]='2';            writeUART(st+5);
                OPEN_1=ONN;               curtFlag1=1;              TimerCounter1=0;
                TMR1H=0x0B;               TMR1L=0xDC;               PIR1bits.TMR1IF=0;        T1CONbits.TMR1ON = 1;         		            
            }
            else{
                CLOSE_1=0;                OPEN_1=0;                 T1CONbits.TMR1ON = 0;		curtFlag1=0;            TimerCounter1=0;
            }
            break;           
        case 2: // forward
            M2=switch_status;
            if(switch_status==1){
                OPEN_1=OFF;
                st[1]='G';				st[2]='0';                  st[3]='0';                  st[4]='1';              writeUART(st+1);
                CLOSE_1=ONN;            curtFlag1=1;                TimerCounter1=0;            		
                TMR1H=0x0B;             TMR1L=0xDC;                 PIR1bits.TMR1IF=0;          T1CONbits.TMR1ON = 1;
            }
            else{
                CLOSE_1=0;            	OPEN_1=0;           	 	T1CONbits.TMR1ON = 0;       curtFlag1=0;            TimerCounter1=0;
            }
            break;

        case 3:// backward
            M3=switch_status;
            if(switch_status==1){
                CLOSE_2=OFF;
                st[13]='G';                st[14]='0';          st[15]='0';                st[16]='4';              writeUART(st+13);                                
                OPEN_2=ONN;                curtFlag2=1;         TimerCounter2=0;
                TMR3H=0x0B;                TMR3L=0xDC;          PIR3bits.TMR3IF=0;         T3CONbits.TMR3ON = 1;   
            }
            else{            
                CLOSE_2=0;                 OPEN_2=0;            T1CONbits.TMR1ON = 0;      curtFlag2=0;             TimerCounter2=0;            
            }
            break;           
        case 4: // forward
            M4=switch_status;
            if(switch_status==1){                
                OPEN_2=OFF;
                st[9]='G';                  st[10]='0';         st[11]='0';                st[12]='3';              writeUART(st+9);                                
                CLOSE_2=ONN;                curtFlag2=1;        TimerCounter2=0;
                TMR1H=0x0B;                 TMR1L=0xDC;         PIR3bits.TMR3IF=0;         T3CONbits.TMR3ON = 1;
            }
            else{
                CLOSE_2=0;                  OPEN_2=0;           T3CONbits.TMR3ON = 0;       curtFlag2=0;            TimerCounter2=0;
            }
            break;          
        default:
            break;
    }
}
     
void writeUART(char *str2Write){
    int Tx_count=0;
 	//while (*str2Write != '\0' || Tx_count!=4)
  	while(Tx_count!=4)
 	{
        //while(PIR1bits.TXIF==0); 
        while (!TX1STAbits.TRMT);
 		TX1REG = *str2Write;
 		*str2Write++;
        Tx_count++;
 	}
}

void EUSART_Initialize(){
    // disable interrupts before changing states
    PIE1bits.RC1IE = 0;
    PIE1bits.TX1IE = 0;

    // Set the EUSART module to the options selected in the user interface.

    // ABDOVF no_overflow; SCKP Non-Inverted; BRG16 16bit_generator; WUE enabled; ABDEN disabled;
    BAUD1CON = 0x0A;

    // SPEN enabled; RX9 8-bit; CREN enabled; ADDEN disabled; SREN disabled;
    RC1STA = 0x90;

    // TX9 8-bit; TX9D 0; SENDB sync_break_complete; TXEN enabled; SYNC asynchronous; BRGH hi_speed; CSRC slave;
    TX1STA = 0x24;

    // Baud Rate = 9600; SP1BRGL 12;
    //SPBRGL = 0x0C;
    //SPBRGL = 0x19;                  // SP1BRGL is 25 (hex value=0x19) for 9600 baud on 16 MHz crystal frequency
    SP1BRGL = 0xA0;                  // SYNC =0 ; BRGH = 1 ; BRG16=1;
    // Baud Rate = 9600; SP1BRGH 1;
    SP1BRGH = 0x01;

    // Enable all active interrupts ---> INTCON reg .... bit 7            page 105
    GIE = 1;

    // Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
    PEIE = 1;

    // enable receive interrupt
    PIE1bits.RC1IE = 1;                    // handled into INTERRUPT_Initialize()

    // Transmit Enabled
    TX1STAbits.TXEN = 1;

    // Serial Port Enabled
    RC1STAbits.SPEN = 1;
}

void TMR1_Initialize(void){
    //Set the Timer to the options selected in the GUI

    //T1CKPS Precaler 1:4; nT1SYNC synchronize; TMR1CS FOSC/4; TMR1ON enabled;
    T1CON = 0x30;

    //T1GSS T1G; TMR1GE disabled; T1GTM disabled; T1GPOL low; T1GGO_nDONE done; T1GSPM disabled;
    T1GCON = 0x00;

        //TMR1H 29;
    TMR1H = 0x00;

    //TMR1L 112;
    TMR1L = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR1bits.TMR1IF = 0;

    // Enabling TMR1 interrupt.
    PIE1bits.TMR1IE = 1;

    // Start TMR1
    T1CONbits.TMR1ON = 1;

    // Enable all active interrupts ---> INTCON reg .... bit 7            page 105
//    GIE = 1;

    // Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
//    PEIE = 1;
}

void TMR3_Initialize(void){
    //Set the Timer to the options selected in the GUI

    //T1CKPS Prescaler 1:4; nT1SYNC synchronize; TMR1CS FOSC/4; TMR1ON enabled;
    T3CON = 0x30;

    //T1GSS T1G; TMR1GE disabled; T1GTM disabled; T1GPOL low; T1GGO_nDONE done; T1GSPM disabled;
    T3GCON = 0x00;

    //TMR1H 29;
    TMR3H = 0x00;
 
    //TMR1L 112;
    TMR3L = 0x00;

    // Clearing IF flag before enabling the interrupt.
    PIR3bits.TMR3IF = 0;

    // Enabling TMR1 interrupt.
    PIE3bits.TMR3IE = 1;

    // Start TMR1
    T1CONbits.TMR1ON = 1;

    // Enable all active interrupts ---> INTCON reg .... bit 7            page 105
    GIE = 1;

    // Enables all active peripheral interrupts -----> INTCON reg .... bit 6         page 105
    PEIE = 1;
}

void pin_manager()  {         
     // NEW BOARD
     
     /* PORT G */     
     ANSELG=0x00;
//     TRISGbits.TRISG1=0;
     WPUG = 0;     
     
     /* PORT F */
     ANSELF=0x00;
     TRISFbits.TRISF0=0;            // relay 1
     TRISFbits.TRISF1=0;            // relay 2
     TRISFbits.TRISF2=1;            // switch 4
     TRISFbits.TRISF3=1;            // switch 3
     TRISFbits.TRISF4=1;
     TRISFbits.TRISF5=1;            // switch 2
     TRISFbits.TRISF6=1;
     TRISFbits.TRISF7=1;            // switch 1
     
     /* PORT E */
     WPUE=0x00;
     ANSELE=0x00;
     TRISEbits.TRISE3=1;               // zcd RE3 input
     TRISEbits.TRISE5=0;               // pwm RE5 output
     
     /* PORT D */  
     WPUD=0x00;
     ANSELD=0x00;
     TRISD=0xFF;
     
     /* PORT C */
     TRISCbits.TRISC0=0;
     TRISCbits.TRISC1=0;
     
     /* PORT B */
     ANSELB=0x00;
     TRISBbits.TRISB1=0;
     TRISBbits.TRISB3=0;
     WPUB = 0x00;
     
     /* PORT A */
     ANSELA = 0x00;
     TRISAbits.TRISA0=0;
     TRISAbits.TRISA1=0;
     TRISAbits.TRISA2=0;            // relay 4
     TRISAbits.TRISA3=0;            // relay 3
//     TRISAbits.TRISA4=1;
     TRISAbits.TRISA5=1;            // switch 5
     
    
      /* CCP1 */
     // PORT C - RC2 
     //TRISCbits.TRISC2 = 1;              // CCP1
     
    // uart pin initialization
    TRISCbits.TRISC6 = 0;               // Tx pin = output
    TRISCbits.TRISC7 = 1;               // Rx pin = input

}

