

#include <stdio.h>
#include <stdlib.h>
#include <p18f4520.h>
#include <delays.h>
#include <timers.h>
#include <usart.h>

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
float errordir;
int x1;
int x2;
int y1;
int y2;
int calibrate;
float P, I, D;
int setpoint;
int Imin, Imax;
float headinghigh = 0;
float headinglow = 0;
float headingprev = 0;
float flip1 = 0;
float flip2 = 0;
float heading = 0;
float direction;
float south, east, west = 0;
int hold = 0;
int start = 0;
int turn = 0;
int forward = 0;

void enc_check (void);
void low_int_priorities (void);
void frequencycalc (void);
void calcerror (void);
void heading_receive(void);
void initial_direction(void);
void right_turn_forward(void);
void right_turn(void);
void return_direction(void);
void left_turn_forward(void);
void left_turn(void);

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
    if(PIR1bits.RCIF == 1)
    {
        heading_receive();
    }
}
#pragma code	// normal code area from here down

void heading_receive(void)
{
    headinglow = getcUSART();
    
    if(headinglow < 0)
    {
        headinglow = headinglow + 255;
    }
    
    if(PORTAbits.RA7 == 1)
    {
        headinglow = headinglow + 256;
    }
    
    heading = headinglow;
    if (start == 1)
    {
    calcerror(); 
    }
}
void frequencycalc (void)
{ 
    freqr = (encodercountr2);
    freql = (encodercountl2);
    
    freqravg = ((freqravg * 3)/4) + (freqr/4);
    freqlavg = ((freqlavg * 3)/4) + (freql/4);
    encodercountr2 = 0;
    encodercountl2 = 0;
    TMR0H = 0xB1;
    TMR0L = 0xE0;
    INTCONbits.TMR0IF = 0;
}

void enc_check (void) 
    {        
        if(INTCON3bits.INT1F == 1) 
            {
                encodercountr = encodercountr + 1;
                encodercountr2 = encodercountr2 + 1;
                INTCON3bits.INT1F = 0;        // clear this interrupt flag
            }
        else if(INTCON3bits.INT2F == 1) 

            {
                encodercountl = encodercountl + 1;
                encodercountl2 = encodercountl2 + 1;
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
    TRISAbits.TRISA7 = 1;
}

void timerinit (void)
{
    T0CONbits.T08BIT = 0;
    T0CONbits.T0CS = 0;
    T0CONbits.PSA = 1;
    TMR0H = 0xB1;
    TMR0L = 0xE0;
    T0CONbits.TMR0ON = 1;
}

void intinit (void)
{
    RCONbits.IPEN   = 1; // enable interrupt priority
    INTCONbits.GIE  = 1; // enable interrupts
    INTCONbits.PEIE = 1; // enable peripheral interrupts.
    IPR1bits.RCIP = 0;
    PIE1bits.RCIE = 1;
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
    errorprev = errordir;
    
    if (turn == 0)
    {
        errordir = direction - heading;  
    }
    if (turn == 3)
    {
        errordir = south - heading;
    }
    

    if(errordir > 180)
    {
        errordir = errordir - 360;
    }
    else if(errordir < -180)
    {
        errordir = errordir + 360;
    }
    else
    {
        errordir = errordir;//do nothing
    }
    
    P = (errordir * kp); 
    
    derror = errordir - errorprev;
    D = (derror * kd);
    
    ierror = ierror + errordir;
    
    if( ierror < Imin)
    {
        ierror = Imin;
    }
   
    if( ierror > Imax)
    {
        ierror = Imax;
    }
    
    I = (ierror * ki);
    

     
    //slavePower += P + I + D;
  
    
    /* 
       if(slavePower >= 165)
    {        
        slavePower = 165;
    }
    if(slavePower <= 155) 
    { 
        slavePower = 155; 
    } 

 
    if(slavePower >= (x1 + 2))
    {        
        slavePower = (x1 + 2);
    }
    if(slavePower <= (x1 - 1)) 
    { 
        slavePower = (x1 - 1); 
    } 
*/
}
void initial_direction (void)
{
    INTCONbits.GIE = 0;
    speedupdate();
    x2 = slavePower;
    y1 = 2000 - x1;
    y2 = 2000 - x2;

    if(encodercountr >= 2837)
    {
        turn = 1;
        Delay10KTCYx(100);
    }

    INTCONbits.GIE = 1; 


    if(PORTAbits.RA4 == 0)
         {
            x1 = 160;
            x2 = 160;
            slavePower = 160;
            direction = heading;
            east = heading + 90;

            if(east > 360)
            {
                east = east - 360;
            }

            west = heading - 90;
            if(west < 0)
            { 
                west = west + 360;
            }

            south = heading + 180;
            if(south > 360)
            { 
                south = south - 360;
            }

            start = 1;
             //x2 = x2 + 1;
            Delay10KTCYx(5); 

         }
}

void right_turn_forward(void)
{
    if (forward == 0)
    {
    INTCONbits.GIE = 0;
    x1 = 143;
    x2 = 158;  
    y1 = 2000 - x1;
    y2 = 2000 - x2;
    speedupdate();
    
    if(encodercountr >= 200)
        {
        encodercountr = 0;
        forward = 1;            
        errordir = 0;
        Delay10KTCYx(100);
        }
    INTCONbits.GIE = 1;
    }
    if(forward == 1)
    {
        INTCONbits.GIE = 0;
        speedupdate();
        x2 = slavePower;
        y1 = 2000 - x1;
        y2 = 2000 - x2;

        if(encodercountr >= 200)
        {
            encodercountr = 0;
            turn = 2;
            forward = 0;
            Delay10KTCYx(100);
        }
        INTCONbits.GIE = 1;    
    }
}
void right_turn(void)
{
    INTCONbits.GIE = 0;
    x1 = 143;
    x2 = 158;  
    y1 = 2000 - x1;
    y2 = 2000 - x2;
    speedupdate();
    
    if(encodercountr >= 200)
        {
        encodercountr = 0;
        turn = 3;
        Delay10KTCYx(100);
        }
    INTCONbits.GIE = 1;
}
void return_direction(void)
{
    INTCONbits.GIE = 0;
    speedupdate();
    x2 = slavePower;
    y1 = 2000 - x1;
    y2 = 2000 - x2;

    if(encodercountr >= 2837)
    {
        turn = 4;
        Delay10KTCYx(100);
    }

    INTCONbits.GIE = 1; 
}
void left_turn_forward(void)
{
    if (forward == 0)
    {
    INTCONbits.GIE = 0;
    x1 = 158;
    x2 = 143;  
    y1 = 2000 - x1;
    y2 = 2000 - x2;
    speedupdate();
    
    if(encodercountr >= 200)
        {
        encodercountr = 0;
        forward = 1;
        Delay10KTCYx(100);
        }
    INTCONbits.GIE = 1;
    }
    if(forward == 1)
    {
        INTCONbits.GIE = 0;
        speedupdate();
        x2 = slavePower;
        y1 = 2000 - x1;
        y2 = 2000 - x2;

        if(encodercountr >= 200)
        {
            turn = 5;
            forward = 0;
                Delay10KTCYx(100);
        }
        INTCONbits.GIE = 1;    
    }
}
void left_turn(void)
{
    INTCONbits.GIE = 0;
    x1 = 158;
    x2 = 143;  
    y1 = 2000 - x1;
    y2 = 2000 - x2;
    speedupdate();
    
    if(encodercountr >= 200)
        {
        encodercountr = 0;
        turn = 0;
        }
    INTCONbits.GIE = 1;
}
void main (void)
{
    OSCCONbits.IRCF0 = 0;  // Set internal clock to 4 MHz next 3 instr.
    OSCCONbits.IRCF1 = 1;  // see pp. 32 of 18F4520 PDF
    OSCCONbits.IRCF2 = 1;
    
    OpenUSART ( USART_TX_INT_OFF &
                USART_RX_INT_ON &
            	USART_ASYNCH_MODE &
            	USART_EIGHT_BIT &
            	USART_CONT_RX &
            	USART_BRGH_HIGH, 25);

    ierror = 0;
    freqravg = 0;
    freqr = 0;
    freqlavg = 0;
    freql = 0;
    timercount = 0;
    direction = 5;
    errordir =0;
    errorfreq = 0;
    errorprev = 0;
    error = 0;
    kp = 0.0007; 
    kd = 0;
    ki = 0;
    x1 = 150;
    x2 = 150;    
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
        if(turn == 0)
        {
            initial_direction();
        }
        if(turn == 1)
        {
            right_turn_forward();
        }
        if(turn == 2)
        {
            right_turn();
        }
        if(turn == 3)
        {
            return_direction();
        }
        if(turn == 4)
        {
            left_turn_forward();
        }
        if(turn == 5)
        {
            left_turn();
        }
        
    if(PORTBbits.RB0 == 0)
         {
             x1 = 150;
             x2 = 150;
             errordir = 0;
             errorprev = 0;
             ierror = 0;
             start = 0;
             slavePower = 150;
             //x2 = x2 - 1;
             Delay10KTCYx(5); 
         }
 
       }
} 
