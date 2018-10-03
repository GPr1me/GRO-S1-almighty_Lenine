/*
Projet: Déplacement du robot
Equipe: 21
Auteurs: Marc-Olivier Thibault, Vincent Pelletier, Émile Rousseau-Pinard, Charles Maheu
Description: Fonction main pour faire avancer et tourner le robot
Date: Derniere date de modification
*/

/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */

#include <LibRobus.h> // Essentielle pour utiliser RobUS



/* ****************************************************************************
Variables globales et defines
**************************************************************************** */
// -> defines...
// L'ensemble des fonctions y ont acces

const float KP = 0.0001;
const float KI = 0.00002;
const int CYCLEDELAY = 250;
// DISTANCE_PAR_CLIC
//const int TEMPS_PAUSE
int MOTOR_MASTER = 0;
int MOTOR_SLAVE = 1;


/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */
void Avancer(int speed, int distance)
{// start motors
  ENCODER_Reset(MOTOR_MASTER);
  ENCODER_Reset(MOTOR_SLAVE);
  MOTOR_SetSpeed(MOTOR_MASTER, speed);
  MOTOR_SetSpeed(MOTOR_SLAVE, speed);

  int travelDistance = 0;
  int clicNb_master = 0;
  int clicNb_slave = 0;
  int clicNb_start_MASTER = 0;
  int clicNb_start_SLAVE = 0;
  int clicNb_cycle_MASTER = 0;
  int clicNb_cycle_SLAVE = 0;
  int cycleNb = 0;
  float ErrorSpeedTotal=0;

  while(travelDistance < distance)
  {
    clicNb_start_MASTER = ENCODER_Read(MOTOR_MASTER);
    clicNb_start_SLAVE = ENCODER_Read(MOTOR_SLAVE);
    delay(CYCLEDELAY);
    clicNb_cycle_MASTER = ENCODER_Read(MOTOR_MASTER)-clicNb_start_MASTER;
    clicNb_cycle_SLAVE = ENCODER_Read(MOTOR_SLAVE)-clicNb_start_SLAVE;
    //CorrectDistance(/* incomplete */);
    CorrectSpeed(/*incomplet*/);
    

    cycleNb++;
  }

  // Stop motors
  MOTOR_SetSpeed(MOTOR_MASTER, 0);
  MOTOR_SetSpeed(MOTOR_SLAVE, 0);
  ENCODER_Reset(MOTOR_MASTER);
  ENCODER_Reset(MOTOR_SLAVE);
}

int ErrorClicCycle(int clicNb_cycle_MASTER, int clicNb_cycle_SLAVE)
{
  return clicNb_cycle_MASTER-clicNb_cycle_SLAVE;
}


float ErrorPowerCycle(int errorClic_SLAVE)
{
  int errorSpeed_SLAVE = errorClic_SLAVE/CYCLEDELAY;
  return errorSpeed_SLAVE * KP;
}

float CorrectSpeed(int clicNb_cycle_MASTER, int clicNb_cycle_SLAVE, speed)
{
  errorPower = ErrorPowerCycle + ErrorPowerTotal;
  MOTOR_SetSpeed(MOTOR_SLAVE, (speed+errorPower))

}

void CorrectDistance(int clicNb)
{

  
}

int DistanceToClics(float distance)
{
  float clics_turn=3200,total_clics=0, circonference=0,w_radius=3.5;
    circonference=2*PI*w_radius;
    total_clics=(clics_turn*distance)/circonference;
  return (total_clics); //Retourne le nombre de clique nécessaire pour la distance voulue
}

float ErrorIncrement(int clicNb_cycle_MASTER,int clicNb_cycle_SLAVE,float ErrorPowerTotal)
{
  ErrorPowerTotal+= ErrorClicCycle(clicNb_cycle_MASTER, clicNb_cycle_SLAVE);
  ErrorPowerTotal*KI;
  return (ErrorPowerTotal);
}

void SwitchMotorsHierarchy() // Power to the people!
{
  int temp = MOTOR_MASTER;
  MOTOR_MASTER = MOTOR_SLAVE;
  MOTOR_SLAVE = temp;
}

// Prend pour acquis qu'un angle 0 ne tourne pas.
// Un angle négatif tourne à gauche et positif à droite.
void Tourner(float angle)
{
  // ne tourne pas à 0
  if(angle != 0)
  {
    if(angle < 0)
    {
      SwitchMotorsHierarchy();
    }

    // Execution code virage


  }
}

/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
// -> Se fait appeler au debut du programme
// -> Se fait appeler seulement un fois
// -> Generalement on y initilise les varibbles globales

void setup(){
  BoardInit();
}


/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop() {
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  delay(10);// Delais pour décharger le CPU
}