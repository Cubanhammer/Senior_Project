

#include <stdio.h>
#include <stdlib.h>
#include <p18f4520.h>
#include <delays.h>
#include <timers.h>


#pragma config OSC = INTIO67	/* use internal oscillator */
#pragma config WDT = OFF        /* no watchdog       */
#pragma config FCMEN = OFF
#pragma config IESO = OFF
#pragma config PWRT = ON
#pragma config BOREN = OFF
#pragma config MCLRE = ON
#pragma config PBADEN = OFF
#pragma config LVP = OFF
#pragma config XINST = OFF

int masterPower;
int slavePower;
int freqr;
int freql;
int encodercountl = 0;
int encodercountr = 0;
int error;
int errorprev;
float kp;
float kd;
float ki;
int derror;
int ierror;
int encodercountr2;
int encodercountl2;
int freqravg;
int freqlavg;
int timercount;
int errorfreq;
int x1;
int x2;
int y1;
int y2;


void enc_check (void);
void low_int_priorities (void);
void frequencycalc (void);

/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/
/* define interrupt vector area						      */
/*\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/*/

#pragma code low_vector=0x18	// tells compiler that this belongs @ loc 0x18
void low_interrupt (void)
{         
        _asm GOTO low_int_priorities _endasm	// inline assembly for interrupt vector

}
#pragma interruptlow low_int_priorities
void low_int_priorities (void)
{
    if(INTCONbits.TMR0IF == 1)
    {
        frequencycalc();
    }
    if(INTCON3bits.INT1F == 1 || INTCON3bits.INT2F == 1)
    {
        enc_check();
    }
}
#pragma code	// normal code area from here down

void frequencycalc (void)
{ 
    timercount++;
    freqr = (encodercountr2 * 20);
    freql = (encodercountl2 * 20);
    
    freqravg = ((freqravg * 3)>>2) + (freqr>>2);
    freqlavg = ((freqlavg * 3)>>2) + (freql>>2);
    if (timercount == 0)
    {
        for(;;){}
    }

   
    encodercountr2 = 0;
    encodercountl2 = 0;
    TMR0H = 0x3C;
    TMR0L = 0xB0;
    INTCONbits.TMR0IF = 0;
}
void enc_check (void) 
    {        
        if(INTCON3bits.INT1F == 1) 
            {
                encodercountr = encodercountr + 1;
                encodercountr2 = encodercountr2 + 1;
                if (encodercountr == 0xffff)
                {encodercountr = 0;}
                INTCON3bits.INT1F = 0;        // clear this interrupt flag
            }
        else if(INTCON3bits.INT2F == 1) 

            {
                encodercountl = encodercountl + 1;
                encodercountl2 = encodercountl2 + 1;
                if (encodercountl == 0xffff)
                {encodercountl = 0;}
                INTCON3bits.INT2F = 0;        // clear this interrupt flag
            }
    }

void inoutinit (void)
{
    TRISB = 0;
            
    TRISBbits.TRISB2 = 1;   //R encoder input
    TRISBbits.TRISB1 = 1;   //L encoder input
    TRISBbits.TRISB0 = 1;
    
    TRISCbits.TRISC1 = 0;	// R signal out
    TRISCbits.TRISC2 = 0;   // L signal out
    TRISAbits.TRISA4 = 1;
}
void timerinit (void)
{
    T0CONbits.T08BIT = 0;
    T0CONbits.T0CS = 0;
    T0CONbits.PSA = 1;
    TMR0H = 0x3C;
    TMR0L = 0xB0;
    T0CONbits.TMR0ON = 1;
}

void intinit (void)
{
    RCONbits.IPEN   = 1; // enable interrupt priority
    INTCONbits.GIE  = 1; // enable interrupts
    INTCONbits.PEIE = 1; // enable peripheral interrupts.
    INTCON3bits.INT1IE = 1;
    INTCON3bits.INT2IE = 1;
    INTCON3bits.INT1P = 0;
    INTCON3bits.INT2P = 0;
    INTCON2bits.INTEDG1 = 0;
    INTCON2bits.INTEDG2 = 0;
    
    INTCONbits.TMR0IE = 1;
    INTCON2bits.TMR0IP = 0;
}

void speedupdate(void)
{
    PORTCbits.RC2 = 1;
    Delay10TCYx(x1);
    PORTCbits.RC2 = 0;
    Delay10TCYx(y1);
    PORTCbits.RC1 = 1;
    Delay10TCYx(x2);
    PORTCbits.RC1 = 0;
    Delay10TCYx(y2);
}
void calcerror (void)
{
    x2 = slavePower;
    errorprev = error;
    error = encodercountl - encodercountr;
    errorfreq = freqlavg - freqravg;
     
    
    derror = error - errorprev;
 
    slavePower += ((errorfreq * kp) + (derror * kd) + (ierror * ki));
    
    if( slavePower >= 156)
    {
        slavePower = 150;
    }
    if(slavePower <= 103)
    {
        slavePower = 150;
    }
    
    ierror = ierror + error;
    
   
    encodercountl = 0;
    encodercountr = 0;
}
void main (void)
{
    OSCCONbits.IRCF0 = 0;  // Set internal clock to 4 MHz next 3 instr.
    OSCCONbits.IRCF1 = 1;  // see pp. 32 of 18F4520 PDF
    OSCCONbits.IRCF2 = 1;

    
    masterPower = x1;
    slavePower = x2;
    
    freqravg = 100;
    freqr = 0;
    freqlavg = 120;
    freql = 0;
    timercount = 0;
 
    error = 0;
    kp = 0.75;
    kd = 0;
    ki = 0;
    x1 = 144;
    x2 = 144;
    y1 = 10000 - x1;
    y2 = 10000 - x2;
  
   
    inoutinit();
    intinit();
    timerinit();

    while(1)
    { 
    y1 = 10000 - x1;
    y2 = 10000 - x2;
    
    
    INTCONbits.GIE = 0;
    speedupdate();
    calcerror();
    INTCONbits.GIE = 1;
     
    Delay100TCYx(125);
  
 
    if(PORTAbits.RA4 == 0)
         {
             x1 = x1 + 1;
             x2 = x2 + 1;
             Delay10KTCYx(5); 
         }
    if(PORTBbits.RB0 == 0)
         {
             x1 = x1 - 1;
             x2 = x2 - 1;
             Delay10KTCYx(5); 
         }
       }
} 
