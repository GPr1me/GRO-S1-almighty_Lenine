/*
Projet: Le nom du script
Equipe: Votre numero d'equipe
Auteurs: Les membres auteurs du script
Description: Breve description du script
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

void Avancer(int speed, float distance)
{// start motors
  ENCODER_Reset(MOTOR_MASTER);
  ENCODER_Reset(MOTOR_SLAVE);
  MOTOR_SetSpeed(MOTOR_MASTER, speed);
  MOTOR_SetSpeed(MOTOR_SLAVE, speed);

  float travelDistance = 0;
  int clicNb_master = 0;
  int clicNb_slave = 0;
  int cycleNb = 0;
  float ErrorSpeedTotal=0;

  while(travelDistance < distance)
  {
    delay(CYCLEDELAY);

    CorrectDistance(/* incomplete */);
    CorrectSpeed(cycleNb, DistanceToClics(distance), clicNb_master, 1.);

    cycleNb++;
  }

  // Stop motors
  MOTOR_SetSpeed(MOTOR_MASTER, 0);
  MOTOR_SetSpeed(MOTOR_SLAVE, 0);
  ENCODER_Reset(MOTOR_MASTER);
  ENCODER_Reset(MOTOR_SLAVE);
}

void CorrectSpeed(int cycleNb,
                  int clicNbGoal, 
                  int actualTotalClicNb,
                  float speedRatio)
{
  float MotorMaster_actualClicNb = ENCODER_Read(MOTOR_MASTER);
  // incomplete
}

void CorrectDistance()
{
  // incomplete
}

int DistanceToClics(int distance)
{
  // incomplete
  //return distance * ratio;
}

float ErrorIncrement(float errorSpeedCycle,float ErrorSpeedTotal)
{
  ErrorSpeedTotal+=errorSpeedCycle;
  return (ErrorSpeedTotal);
}

void SwitchMotorsHierarchy() // Power to the people!
{
  int temp = MOTOR_MASTER;
  MOTOR_MASTER = MOTOR_SLAVE;
  MOTOR_SLAVE = temp;
}

// Prend pour acquis que l'angle 0degrée est directement tout droit.
// Les angles négatifs sont vers la gauche et positif vers la droite.
void Tourner(float angle)
{
  if(angle == 0)
  {
    //Si angle == 0, On ne fait rien
  }
  else
  {
    if(angle < 0)
    {
      SwitchMotorsHierarchy();
    }

    // exécution du virage
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