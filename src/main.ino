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
const int CLIC_PER_ROTATION = 3200;

// Simule le type de base Booléen
// Sera en réalité un entier égale à 0 ou 1
typedef enum { MOTOR_LEFT, MOTOR_RIGHT } Motors;

int MOTOR_MASTER = 0;
int MOTOR_SLAVE = 1;

void Avancer(float speed, float distance)
{// start motors
  ENCODER_Reset(MOTOR_MASTER);
  ENCODER_Reset(MOTOR_SLAVE);
  MOTOR_SetSpeed(MOTOR_MASTER, speed);
  MOTOR_SetSpeed(MOTOR_SLAVE, speed);

  int clicTotal = DistanceToClics(distance);

  int clicNb_master = 0;
  int clicNb_slave = 0;

  int clicNb_start_MASTER = 0;
  int clicNb_start_SLAVE = 0;

  int clicNb_cycle_MASTER = 0;
  int clicNb_cycle_SLAVE = 0;

  int cycleNb = 0;
  float ErrorPowerTotal=0;
  float eci = 0;

  while(ENCODER_Read(MOTOR_MASTER) < clicTotal)
  {
    clicNb_start_MASTER = ENCODER_Read(MOTOR_MASTER);
    clicNb_start_SLAVE = ENCODER_Read(MOTOR_SLAVE);

    delay(CYCLEDELAY);

    clicNb_cycle_MASTER = ENCODER_Read(MOTOR_MASTER)-clicNb_start_MASTER;
    clicNb_cycle_SLAVE = ENCODER_Read(MOTOR_SLAVE)-clicNb_start_SLAVE;

    eci = CorrectSpeed(clicNb_cycle_MASTER, clicNb_cycle_SLAVE, speed, eci);
    
    cycleNb++;
  }

  // Stop motors
  MOTOR_SetSpeed(MOTOR_MASTER, 0);
  MOTOR_SetSpeed(MOTOR_SLAVE, 0);
  ENCODER_Reset(MOTOR_MASTER);
  ENCODER_Reset(MOTOR_SLAVE);
}
/*Cette fonction détermine la différence de cliques que le moteur slave assume*/
float ErrorClicCycle(int clicNb_cycle_MASTER, int clicNb_cycle_SLAVE)
{
  return ((clicNb_cycle_MASTER)-(clicNb_cycle_SLAVE))/CYCLEDELAY;
}

/*Cette fonction prend pour entrée la différence de cliques, la divise par la différence de temps
choisie et multiplie finalement le tout par KP. */
float ErrorPowerCycle(int errorClicCycle)
{
  return errorClicCycle * KP;
}

/*Cette fonction prend comme entrée le nombre de clique des deux moteurs ainsi que l'erreur cumulée
depuis le début du trajet */
float ErrorClicIncrement(int errorClicCycle,float errorClicIncrement)
{
  float errorClicIncrement = errorClicIncrement + errorClicCycle;
  return (errorClicIncrement);
}
float ErrorPowerIncrement(float errorClicIncrement)
{
  float errorPowerIncrement = KI * errorClicIncrement;
  return errorPowerIncrement;
}
float CorrectSpeed(int clicNb_cycle_MASTER,int clicNb_cycle_SLAVE,float initialMotorSpeed, float eci) //Cette partie réalise l'addition des deux paramètres contenant KI et KP
{
  int ecc = ErrorClicCycle(clicNb_cycle_MASTER,clicNb_cycle_SLAVE);
  eci += ErrorClicIncrement(ecc, eci);
  float errorPower = ErrorPowerCycle(ecc) 
                 + ErrorPowerIncrement(eci);
  initialMotorSpeed+=errorPower;
  MOTOR_SetSpeed(MOTOR_SLAVE,initialMotorSpeed);
  return eci;
}

//Retourne le nombre de clique nécessaire pour la distance voulue
int DistanceToClics(float distance)
{
  float w_radius = 3.5; //POUCES
  float circonference = 2 * PI * w_radius;

  return (CLIC_PER_ROTATION * distance)/circonference;
}

void SetMaster(Motors ID) // Power to the people!
{
  switch (ID) 
   {
      case MOTOR_LEFT:      
        MOTOR_MASTER = MOTOR_LEFT;
        MOTOR_SLAVE = MOTOR_RIGHT;
      case MOTOR_RIGHT:      
        MOTOR_MASTER = MOTOR_RIGHT;
        MOTOR_SLAVE = MOTOR_LEFT;
      // Ne devrait JAMAIS être un cas par défaut comme il s'agit d'un
      // ENUM. Sinon quoi revérifier la définition de l'ENUM Motors
      default: break;
   }
  int temp = MOTOR_MASTER;
  MOTOR_MASTER = MOTOR_SLAVE;
  MOTOR_SLAVE = temp;
}

// Prend pour acquis qu'un angle 0 ne tourne pas.
// Un angle négatif tourne à gauche et positif à droite.
void Turn(float angle)
{
  // ne tourne pas à 0
  if(angle != 0)
  {
    if(angle < 0)
    {
      SetMaster(MOTOR_LEFT); //Le moteur gauche est rendu MASTER
    }

    if(angle > 0)
    {
      SetMaster(MOTOR_RIGHT); //Le moteur droit est rendu MASTER
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
  Avancer(0.5,5)
}