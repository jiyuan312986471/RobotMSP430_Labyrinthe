
#include "function.c"


/********************************************
            TA1.1 for engine A
            TA1.2 for engine B
********************************************/
// variable
char avancer = 0;      // avancer:  0 -> stop
                       //           1 -> run

int vitesse_L = 0, vitesse_R = 0;

char compteur = 0;

int main( void )
{
  // Stop watchdog timer to prevent time out reset
  WDTCTL = WDTPW + WDTHOLD;

  // Initialization
  Config_SYS();
  char sens = FRONT;     // sens   :  0 -> front
                         //           1 -> droite
                         //           2 -> gauche

  
  // PID
  PID_Uint PID;
  PID.Ur = 20480;   // PWM Min = (100-20480/1024)% = 80%
  PID.Un = 5;       // 
  PID.Kp = 2;       // P
  PID.Ti = 0;       // I 
  PID.Td = 0;       // D
  Init_PID_uint(&PID);
  reset_Uk(&PID);
  
  // Interrupt
  __enable_interrupt();
  
  // mesures
  int mesure_F = 0, mesure_R = 0, mesure_L = 0;
  
  // while
  while(1)
  {
  
    if(avancer)
    {
      // obstacle detection
      mesure_F = Capture(CAP_FRONT);
      mesure_R = Capture(CAP_RIGHT);
      mesure_L = Capture(CAP_LEFT);
      //sens = Obs_Detection(sens, mesure_F, mesure_R);
      sens = Obs_Detection(sens, mesure_F, mesure_R, mesure_L);
      
      // set wheel direction
      ChangeSens(sens);
      
      // run
      Avancer(avancer, vitesse_L, vitesse_R);
      
      // distance control
      if(!sens)
      {
        // Ctrl
        Ctrl_Dist(&vitesse_L, &vitesse_R, mesure_L, mesure_R, &PID);
      }
      
    }//endif(avancer)

  }//endwhile
  return 0;
}



/***************************************************
          Interrupt Service Routine P1
....................................................
Button S2 to start and stop the robot
***************************************************/
#pragma vector = PORT1_VECTOR
__interrupt void On_Off()
{
  // Flag
  P1IFG &= ~BIT3;
  
  // start
  switch(avancer)
  {
    case 0:
      // robot
      vitesse_L = V_MAX_L;
      vitesse_R = V_MAX_R;
      avancer = 1;
      
      // TIMER0
      TACTL |= MC_3;
      break;
      
    case 1:
      TA1CCR1 = 0;
      TA1CCR2 = 0;
      avancer = 0;
      break;
      
    default:
      break;
  }
}


#pragma vector = TIMER0_A1_VECTOR
__interrupt void Timer_Increment()
{
  switch(TAIV)
  {
    case 10:
      compteur++;
      if(compteur >= 120)
      {
        TACTL &= ~MC_3;
        TAR = 0;
        TA1CCR1 = 0;
        TA1CCR2 = 0;
        avancer = 0;
        compteur = 0;
      }
      break;
      
    default:
      break;
  }
}