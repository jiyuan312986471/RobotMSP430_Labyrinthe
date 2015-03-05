#include "io430.h"
#include "Configuration.c"

// fonction

/****************************************
                TEMPO
.........................................
Parameter : int ms
****************************************/
void Tempo(int ms)
{
  for(int i=0; i<ms; i++)
  {
    __delay_cycles(1000);
  }
}



/****************************************
                Avancer
.........................................
When avancer = 0 : Stop
When avancer = 1 : Avancer
****************************************/
void Avancer(char avancer, int vitesse_L, int vitesse_R)
{
  switch(avancer)
  {
    case 0:
      TA1CCR1 = 0;
      TA1CCR2 = 0;
      break;
      
    case 1:
      TA1CCR1 = vitesse_L * 10;  // --> A
      TA1CCR2 = vitesse_R * 10;  // --> B      
      break;
      
    default:
      break;
  } 
}



/****************************************
              ChangeSens
.........................................
Parameter : char sens
.........................................
When tourner = 0 : Stop
When tourner = 1 : Turn Right
When tourner = 2 : Turn Left
****************************************/
void ChangeSens(char sens)
{
  switch(sens)
  {
    case 0:
      // Sens Moteurs
      P2OUT &= ~BIT1;  // A: avancer
      P2OUT |= BIT5;   // B: avancer
      break;
      
    case 1:
      // Sens Moteurs
      P2OUT &= ~BIT1;  // A: avancer
      P2OUT &= ~BIT5;  // B: reculer
      break;
      
    case 2:
      // Sens Moteurs
      P2OUT |= BIT1;  // A: reculer
      P2OUT |= BIT5;  // B: avancer
      break;
      
    default:
      break;
  }
}



/****************************************
                Capture
.........................................
Parameter : char voie
Return    : int
.........................................
Read value of InfraRouge
Start ADC to convert
Return value of the result
****************************************/
int Capture(char voie)
{
  int i;
  // Convertion
  ADC10CTL1 &= ~0xF000;
  ADC_Demarrer_conversion(voie);
  i = ADC_Lire_resultat();
  return i;
}



/***************************************************
*                                                  *
*                CONTROL DISTANCE                  *
*                                                  *
***************************************************/
void Ctrl_Dist(int* vitesse_L, int* vitesse_R, int mesure_L, int mesure_R, PID_Uint *p)
{
  int set_M = 0, set_L = 695, set_R = 695;  // set_L, set_R -> 7cm before: 695
  int mesure_M = mesure_L - mesure_R;
  
  if(mesure_L < 204 && mesure_R < 204)  // Dist_L > 30cm && Dist_R > 30cm
  {
    // full speed
    *vitesse_L = V_MAX_L;
    *vitesse_R = V_MAX_R;
  }
  else if(mesure_L < 204)  // Dist_L > 30cm && Dist_R < 30cm
  {
    // control by Dist_R -> 7cm
    if(mesure_R < set_R)  // too far from right
    {
      *vitesse_L = V_MAX_L;
      *vitesse_R = V_MAX_R - PID_common(set_R, mesure_R, p);
    }
    else  // too close to right
    {
      *vitesse_L = V_MAX_L - PID_common(set_R, mesure_R, p);
      *vitesse_R = V_MAX_R;
    }
  }
  else if(mesure_R < 204)  // Dist_L < 30cm && Dist_R > 30cm
  {
    // control by Dist_L -> 7cm
    if(mesure_L < set_L)  // too far from left
    {
      *vitesse_L = V_MAX_L - PID_common(set_L, mesure_L, p);
      *vitesse_R = V_MAX_R;
    }
    else  // too close to left
    {
      *vitesse_L = V_MAX_L;
      *vitesse_R = V_MAX_R - PID_common(set_L, mesure_L, p);
    }
  }
  else  // Dist_L < 30cm && Dist_R < 30cm
  {
    // stay in mid
    if(mesure_M < set_M)  // distance left > distance right
    {
    *vitesse_L = V_MAX_L - PID_common(set_M, mesure_M, p);
    *vitesse_R = V_MAX_R;
    }
    else  // distance right > distance left
    {
    *vitesse_L = V_MAX_L;
    *vitesse_R = V_MAX_R - PID_common(set_M, mesure_M, p);
    }
  }
}



/****************************************
          Obstacle_Detection
.........................................
This function detects obstacles in front
of the robot and on the right of the ro-
bot.
And decide to run or turn
****************************************/
char Obs_Detection(char sens, int mesure_F, int mesure_R, int mesure_L)
{
  switch(sens)
  {
    case FRONT:
      if(mesure_F > 818)  // before: 750
      {
        if(mesure_R > mesure_L)
        {
          return LEFT;
        }
        else
        {
          return RIGHT;
        }
      }
      else
      {
        return FRONT;
      }
      break;
      
    case RIGHT:
      if(mesure_F < 242 && mesure_L > 635)  // before: 600
      {
        return FRONT;
      }
      else
      {
        return RIGHT;
      }
      break;
      
    case LEFT:
      if(mesure_F < 242 && mesure_R > 635)
      {
        return FRONT;
      }
      else
      {
        return LEFT;
      }

      break;
      
    default:
      break;
  }
}