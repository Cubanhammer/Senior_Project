

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

float masterPower;
float slavePower;
int freqr;
int freql;
int encodercountl = 0;
int encodercountr = 0;
float error;
float errorprev;
float kp;
float kd;
float ki;
float derror;
float ierror;
float encodercountr2;
float encodercountl2;
float freqravg;
float freqlavg;
int timercount;
float errorfreq;
int x1;
int x2;
int y1;
int y2;
int calibrate;
float P, I, D;
int setpoint;
int Imin, Imax;


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
    freqr = (encodercountr2);
    freql = (encodercountl2);
    
    freqravg = ((freqravg * 3)/4) + (freqr/4);
    freqlavg = ((freqlavg * 3)/4) + (freql/4);

   
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
            
    TRISBbits.TRISB2 = 1;   //L encoder input
    TRISBbits.TRISB1 = 1;   //R encoder input
    TRISBbits.TRISB0 = 1;
    
    TRISCbits.TRISC1 = 0;	// L signal out
    TRISCbits.TRISC2 = 0;   // R signal out
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
    errorprev = errorfreq;
    

    errorfreq = freqr - freql;  
    
    
    P = (errorfreq * kp); 
    
    derror = errorfreq - errorprev;
    D = (derror * kd);
    
    ierror = ierror + errorfreq;
    I = (ierror * ki);
    
    if(I>Imax)
    {
        I=1;
    }
    if(I<Imin)
    {
        I=-1;
       
    }
    
    slavePower -= P + I + D;
  


    
   
    encodercountl = 0;
    encodercountr = 0;
}
void main (void)
{
    OSCCONbits.IRCF0 = 0;  // Set internal clock to 4 MHz next 3 instr.
    OSCCONbits.IRCF1 = 1;  // see pp. 32 of 18F4520 PDF
    OSCCONbits.IRCF2 = 1;

    calibrate = 0;

    ierror = 0;
    freqravg = 0;
    freqr = 0;
    freqlavg = 0;
    freql = 0;
    timercount = 0;
    
    setpoint = 7;
    
    errorfreq = 0;
    errorprev = 0;
    error = 0;
    kp = 0.11; // ***.075 steady oscillation .05 responds to disturbance .075 steady
    
    kd = 0.7; // .02 steady .075 steady oscillation ***.25
    ki = 0;
    x1 = 143;
    x2 = 143;    
    masterPower = x1;
    slavePower = x2;
    y1 = 2000 - x1;
    y2 = 2000 - x2;
    Imin = -1;
    Imax = 1;
  
   
    inoutinit();
    intinit();
    timerinit();

    while(1)
    { 
    INTCONbits.GIE = 0;
    speedupdate();

    x2 = slavePower;
    y1 = 2000 - x1;
    y2 = 2000 - x2;
    
    calcerror();
    INTCONbits.GIE = 1; 
     
    Delay100TCYx(125);
  
    if(PORTAbits.RA4 == 0)
         {
             x1 = x1 + 1;
             //x2 = x2 + 1;
             Delay10KTCYx(5); 
         }
    if(PORTBbits.RB0 == 0)
         {
             x1 = x1 - 1;
             //x2 = x2 - 1;
             Delay10KTCYx(5); 
         }
       }
} 
