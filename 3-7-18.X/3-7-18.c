

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
float errorprevr;
float errorprevl;
float kp;
float kd;
float ki;
float derrorr;
float derrorl;
float ierrorr;
float ierrorl;
float encodercountr2;
float encodercountl2;
float freqravg;
float freqlavg;
int timercount;
float errorfreqr;
float errorfreql;
int x1;
int x2;
int y1;
int y2;
int calibrate;
float Pr, Ir, Dr;
float Pl, Il, Dl;
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
    timercount++;
    if(calibrate <= 1000)
    {
    calibrate++;
    }
    freqr = (encodercountr2);
    freql = (encodercountl2);
    
    freqravg = ((freqravg * 3)/4) + (freqr/4);
    freqlavg = ((freqlavg * 3)/4) + (freql/4);
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
    errorprevr = errorfreqr;
    errorprevl = errorfreql;
    
    //error = encodercountr - encodercountl;

    //errorfreq = freqravg - freqlavg;  
    
    errorfreqr = setpoint - freqravg;
    errorfreql = setpoint - freqlavg;
    
    Pr = (errorfreqr * kp); 
    Pl = (errorfreql * kp);
    
    derrorr = errorfreqr - errorprevr;
    derrorl = errorfreql - errorprevl;
    //derror = freqravg / freqlavg;
    Dr = (derrorr * kd);
    Dl = (derrorl * kd);
    
 

    slavePower -= Pl + Il + Dl;
    masterPower -= Pr + Ir + Dr;

    
    /*
    if( slavePower >= 156)
    {
        slavePower = 156;
    }
    if(slavePower <= (x1 - 1))
    {
        slavePower = (x1 - 1);
    }
    */
    
    ierrorl = ierrorl + errorfreql;
    ierrorr = ierrorr + errorfreqr;
    /*
    if( ierrorr < Imin)
    {
        ierrorr = Imin;
    }
    if( ierrorl < Imin)
    {
        ierrorl = Imin;
    }
    if( ierrorr > Imax)
    {
        ierrorr = Imax;
    }
    if( ierrorl > Imax)
    {
        ierrorl = Imax;
    }
     * */
    Ir = (ierrorr * ki);
    Il = (ierrorl * ki);
    
   
    encodercountl = 0;
    encodercountr = 0;
}
void main (void)
{
    OSCCONbits.IRCF0 = 0;  // Set internal clock to 4 MHz next 3 instr.
    OSCCONbits.IRCF1 = 1;  // see pp. 32 of 18F4520 PDF
    OSCCONbits.IRCF2 = 1;

    calibrate = 0;

    ierrorl, ierrorr = 0;
    freqravg = 0;
    freqr = 0;
    freqlavg = 0;
    freql = 0;
    timercount = 0;
    
    setpoint = 7;
    
    errorfreqr = 0;
    errorfreql = 0;
    errorprevl = 0;
    errorprevr = 0;
    error = 0;
    //kp = 0.125;
    kp = 0.15;
    kd = 1;
    //kd = 0;
    ki = 0;
    //ki = 0.0125;
    x1 = 144;
    x2 = 144;    
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
    x1 = masterPower;
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
