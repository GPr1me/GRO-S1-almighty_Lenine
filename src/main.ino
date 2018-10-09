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

const float KP = 0.000025;
const float KI = 0.00012;

const float CYCLEDELAY = 0.025;
const int CLIC_PER_ROTATION = 3200;

// Simule le type de base Booléen
// Sera en réalité un entier égale à 0 ou 1
typedef enum { MOTOR_LEFT, MOTOR_RIGHT } Motors;

int MOTOR_MASTER = 0;
int MOTOR_SLAVE = 1;

void Avancer(float speed, float distance)
{// start motors
  float speed_accel = 0.05;

  ENCODER_Reset(MOTOR_MASTER);
  ENCODER_Reset(MOTOR_SLAVE);
  MOTOR_SetSpeed(MOTOR_MASTER, speed_accel);
  MOTOR_SetSpeed(MOTOR_SLAVE, speed_accel);

  int clicTotal = DistanceToClics(distance);

  int clicNb_master = 0;
  int clicNb_slave = 0;

  int clicNb_start_MASTER = 0;
  int clicNb_start_SLAVE = 0;

  int clicNb_cycle_MASTER = 0;
  int clicNb_cycle_SLAVE = 0;

  int cycleNb = 0;
  float speed_cycle_error = 0;
  float speed_total_error = 0;
  float speed_total = 0;

  while(ENCODER_Read(MOTOR_MASTER) < clicTotal)
  {
    Serial.println("----------------------------------------------------------------------------------------------------\n\n");
    Serial.println("cycleNumber: ");
    Serial.println(cycleNb);
    clicNb_start_MASTER = ENCODER_Read(MOTOR_MASTER);
    clicNb_start_SLAVE = ENCODER_Read(MOTOR_SLAVE);

    delay(CYCLEDELAY*1000);

    clicNb_cycle_MASTER = ENCODER_Read(MOTOR_MASTER)-clicNb_start_MASTER;
    clicNb_cycle_SLAVE = ENCODER_Read(MOTOR_SLAVE)-clicNb_start_SLAVE;

    speed_cycle_error = (clicNb_cycle_MASTER - clicNb_cycle_SLAVE)/CYCLEDELAY;
    speed_total_error = speed_total_error + speed_cycle_error;

    if (speed_accel < speed)
    {
    speed_accel += 0.05;
    }

    speed_total = speed_accel + (speed_cycle_error * KP) + (speed_total_error * KI);
    MOTOR_SetSpeed(MOTOR_SLAVE, speed_total);
    MOTOR_SetSpeed(MOTOR_MASTER, speed_accel);
    Serial.println("speed_cycle_error");
    Serial.println(speed_cycle_error);
    Serial.println("speed_total_error");
    Serial.println(speed_total_error);


    cycleNb++;
  }

  // Stop motors
  MOTOR_SetSpeed(MOTOR_MASTER, 0);
  MOTOR_SetSpeed(MOTOR_SLAVE, 0);
  ENCODER_Reset(MOTOR_MASTER);
  ENCODER_Reset(MOTOR_SLAVE);
}

//Retourne le nombre de clique nécessaire pour la distance voulue
int DistanceToClics(float distance)
{
  float w_radius = 3.8; //CM
  float circonference = 2 * PI * w_radius;

  return (CLIC_PER_ROTATION * distance)/circonference;
}

int AngleToClics (float angle)
{
  float w_distance = 16.00;
  float arc_complet = 2 * PI * w_distance/2;
  float arc_angle = angle * arc_complet / 360;

  return (DistanceToClics(arc_angle));
}
void SetMaster(Motors ID) // Power to the people!
{
  switch (ID) 
   {
      case MOTOR_LEFT:      
        MOTOR_MASTER = MOTOR_LEFT;
        MOTOR_SLAVE = MOTOR_RIGHT;
        break;
      case MOTOR_RIGHT:      
        MOTOR_MASTER = MOTOR_RIGHT;
        MOTOR_SLAVE = MOTOR_LEFT;
        break;
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
  float speed = 0.2;
  ENCODER_Reset(MOTOR_MASTER);
  ENCODER_Reset(MOTOR_SLAVE);
  delay(CYCLEDELAY * 10000);
  int clicNb_start_MASTER = 0;
  int clicNb_start_SLAVE = 0;
  int clicNb_cycle_MASTER = 0;
  int clicNb_cycle_SLAVE = 0;
  float speed_total_error = 0;
  float speed_cycle_error = 0;
  int nbClic_turn = AngleToClics(angle);
  float speed_total = 0;

  // ne tourne pas si angle=0
  if (angle!= 0)
  {
    if(angle < 0)
    {
      SetMaster(MOTOR_RIGHT); //Le moteur droit est rendu MASTER
      angle = angle * -1;
    }

    if(angle > 0)
    {
      SetMaster(MOTOR_LEFT); //Le moteur gauche est rendu MASTER
    }
    MOTOR_SetSpeed(MOTOR_MASTER, speed);
    MOTOR_SetSpeed(MOTOR_SLAVE, -1* speed);
    while (ENCODER_Read(MOTOR_MASTER) < nbClic_turn)
    {
      clicNb_start_MASTER = ENCODER_Read(MOTOR_MASTER);
      clicNb_start_SLAVE = ENCODER_Read(MOTOR_SLAVE);

      delay(CYCLEDELAY*1000);

      clicNb_cycle_MASTER = ENCODER_Read(MOTOR_MASTER)-clicNb_start_MASTER;
      clicNb_cycle_SLAVE = (ENCODER_Read(MOTOR_SLAVE)-clicNb_start_SLAVE)*-1;

      speed_cycle_error = (clicNb_cycle_MASTER - clicNb_cycle_SLAVE)/CYCLEDELAY;
      speed_total_error = speed_total_error + speed_cycle_error;

      
      speed_total = speed + (speed_cycle_error * KP) + (speed_total_error * KI);
      MOTOR_SetSpeed(MOTOR_SLAVE, -1* speed_total);
      Serial.println("nb de clic master");
      Serial.println(ENCODER_Read(MOTOR_MASTER));
    }
  MOTOR_SetSpeed(MOTOR_MASTER, 0);
  MOTOR_SetSpeed(MOTOR_SLAVE, 0);
  ENCODER_Reset(MOTOR_MASTER);
  ENCODER_Reset(MOTOR_SLAVE);
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

void loop()
 {
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  delay(10);// Delais pour décharger le CPU

  if(ROBUS_IsBumper(2))
  {
    //Avancer(0.4, 100);
    Turn(90);
    delay(500);
    Turn(90);
    delay(500);
    Turn(-90);
    delay(500);
    Turn(-90);
    //Serial.println("test");
  }
}