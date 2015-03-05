
#include "io430.h"
#include "PID.c"
#include "ADC.c"

#define CAP_FRONT 4
#define CAP_RIGHT 5
#define CAP_LEFT  6

#define FRONT 0
#define RIGHT 1
#define LEFT  2

#define V_MAX_L 94
#define V_MAX_R 100

/*****************************************
                PORT1
..........................................
BIT3 : Button S2
BIT4 : A4 --> Capteur Devant
BIT5 : A5 --> Capteur Droite
BIT6 : A6 --> Capteur Gauche
*****************************************/
void Config_P1()
{
  P1SEL &= ~(BIT4 + BIT5 + BIT6 + BIT3);
  P1SEL2 &= ~(BIT4 + BIT5 + BIT6 + BIT3);
  P1DIR &= ~(BIT4 + BIT5 + BIT6 + BIT3);
  P1REN |= BIT3;
  P1OUT |= BIT3;
  P1IE |= BIT3;
  P1IES |= BIT3;
  P1IFG &= ~BIT3;
}




/*****************************************
      Output for motor A and B
..........................................
BIT2 : PWM for A
BIT4 : PWM for B
..........................................
BIT1 : Motor turning direction of A
BIT5 : Motor turning direction of B
..........................................
BIT7 : Reserved for Interrupt
*****************************************/
void Config_P2()
{
  // PWM
  P2SEL |= BIT2 + BIT4;
  P2SEL2 &= ~(BIT2 + BIT4);
  
  // Sens
  P2SEL &= ~(BIT1 + BIT5);
  P2SEL2 &= ~(BIT1 + BIT5);
  
  // Output
  P2DIR |= BIT1 + BIT2 + BIT4 + BIT5;
  
  // Sens moteur
  P2OUT &= ~BIT1; // A
  P2OUT |= BIT5;  // B
}




/*****************************************
     Frequency for MSP430 : 1MHz
*****************************************/
void Config_Frequency()
{
  // Frequency 
  BCSCTL1= CALBC1_1MHZ; 
  DCOCTL= CALDCO_1MHZ; 
}




/*****************************************
    TIMER1 : generate PWM for motors
..........................................
Frequency : 1kHz
Mode      : Reset/Set
Clock     : SMCLK
*****************************************/
void Config_TIMER1()
{
  // TA1CTL
  TA1CTL = TASSEL_2;
  
  // TA1CCTL1, TA1CCTL2
  TA1CCTL1 |= OUTMOD_7;
  TA1CCTL2 |= OUTMOD_7;
  
  // start TIMER
  TA1CTL |= MC_1;
  
  // TA1CCR0, TA1CCR1, TA1CCR2
  TA1CCR0 = 1000; // 1kHz
  TA1CCR1 = 0;
  TA1CCR2 = 0;
}




/*****************************************
    TIMER0 : Used for counting time
..........................................
Clock     : SMCLK
Interrupt : Enabled
..........................................
We start this TIMER in the program, here
we just configure the TIMER
*****************************************/
void Config_TIMER0()
{
  // TACTL
  TACTL = TASSEL_2 + ID_3 + TAIE;
  
  // TACCR0
  TACCR0 = 62500;
}



/*****************************************
             Config_SYS
..........................................
All configuration
*****************************************/
void Config_SYS()
{
  // Configuration
  Config_Frequency();
  Config_P1();
  Config_P2();
  Config_TIMER1();
  Config_TIMER0();
  ADC_Init();
}