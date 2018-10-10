/*
Projet: Défi du parcours
Equipe: 21
Auteurs: Antoine Lesieur, Charles Maheu, Vincent Pelletier, Santiago Pereira, 
         William Rousseau, Émile Rousseau-Pinard, Philippe Théroux, Marc-Olivier Thibault
Description: Permet à Robus d'MoveForward et de tourner en se basant sur l'angle de rotation de chaque roues.
Date: 10 octobre 2018
*/

/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */

#include <LibRobus.h> // Essentielle pour utiliser RobUS
#include <Stream.h>


/* ****************************************************************************
Variables globales et defines
**************************************************************************** */

const float DELAY = 20.0;
const float KI = 0.0007;//0.0005 ok
const float KP = 0.00001;//0.00001 ok 
//try KP: 0.0001 trop grand, 0.001 pire, 0.01 nope, 0.000001 better hahaha, 0.00001, 0.0005
//try KI: 0.000002 trop grand, 0.00002 pire, 0.0002 nope, 0.00000002 lol, 0.0000002, 0.00001
const int MOTOR_MASTER = 0;
const int MOTOR_SLAVE = 1;
const int CLIC_PER_ROTATION = 3200;
const float DISTANCE_ENTRE_LES_ROUES = 19.05;
//3200 coches par tour de roue
//LEFT 0, RIGHT 1, FRONT 2, REAR 3
//constante clics/cm;


/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */

#pragma region ACC_MASTER
/// <summary>
/// Gère l'accélération du Robus.
/// Moteur gauche est le MOTOR_MASTER.
/// </summary>
/// <param name="start_speed">vitesse initiale</param>
/// <param name="end_speed">vitesse finale</param>
/// <param name="nb_iterations">Nombre d'itération pour ajuster la vitesse (accélération)</param>
void ACC_MASTER(float start_speed, float end_speed, int nb_iterations)
// Si vous avez besoin du moteur droit comme MOTOR_MASTER changez 
// MOTOR_SetSpeed(0, i) pour MOTOR_SetSpeed(1, i)
{

// Vitesse à 70% : 6776 clics par seconde
//Vitesse à 100% : 9635.5 clics par seconde
//Clics par cm = 23.876160
// (200+45/2)+50+45+50+(18+45/2)+(54+45/2)+(60+45/2)+50+76= 693.00
// La distance du trajet est de 693.00cm
//Le robot va faire 16546.17888 clics au total
// La vitesse doit être de 4% pour que le robot fasse le trajet en 60 secondes

float correction = 0;

  // accélération
  if (start_speed < end_speed)
  // La diff entre les 2 if c'est que la vitesse finale va etre plus petite 
  // que la vitesse initiale s'il ralentit et plus grande s'il accelere. Puisque
  // j'ai défini mon n comme étant vitesse finale - initiale, il va savoir tout seul
  // s'il faut qu'il incremente ou qu'il decremente. 
  {
    float n = (end_speed - start_speed)/nb_iterations;//10 pour 1 seconde
    // ici le n est diviser par 10. pour qu'il se rende à la vitesse finale en 10 loop
    // si j'avais mis un n comme 0.05 ou qq chose comme ça, vu que les vitesses changent 
    // tout le temps, le n se serait jamais rendu pile sur la vitesse souhaitée.
    for (float i = start_speed; i <= end_speed; i+=n)
    {
      //ajout de adjustement slave
      correction = SlaveAdjust(i, 0, correction);
      // delay sujet à changement ou à l'implementation en tant que variable au besoin
      // delay(50);
    }
  }
  // décélération
  else if (start_speed > end_speed)
  {
    float n = (end_speed - start_speed)/nb_iterations;
    for (float i = start_speed; i >= end_speed; i+=n)
    {

      // MOTOR_SetSpeed(LEFT, i);
      correction = SlaveAdjust(i, 0, correction);
      // delay(50);
    }
  }
  // en gros si la vitesse finale et initiale sont pareils fait rien
  /* else
  {
    correction = SlaveAdjust(end_speed, 0, correction);
  } */
}
#pragma endregion

#pragma region CalcTurnRatio
//ratio marche bien avec adaptation vitesse et fonction Turn
//par contre, avec des plus grands rayons. fonction angle a cm a tendance a ne plus donner les distances voulues
//peut-etre mettre en double

/// <summary>
/// Calcul le ratio entre les vitesses des roues nécessaire
/// pour faire une rotation d'un rayon défini.
/// </summary>
/// <param name="rayon">Rayon du tournant</param>
/// <returns>Le ratio permettant de tourner en respectant le rayon.</returns>
float CalcTurnRatio(float rayon)
{
    if (rayon > 0)
    {
      return (rayon + DISTANCE_ENTRE_LES_ROUES)/rayon;
    }
    else
    {
      return (-1)*(rayon - DISTANCE_ENTRE_LES_ROUES)/rayon;
    }
}
#pragma endregion

#pragma region ClicToCM
/// <summary>
/// Converti les "clics" d'une roue en distance (cm).
/// Un clic est 1/3200 de la circonférence d'une roue.
/// </summary>
/// <param name="nbClics">Angle de cercle en clics</param>
/// <returns>La longueur en cm équivalent aux nombre de clics.</returns>
double ClicToCM(long int nbClics)
{
  double circonference = (2. * 38 / 10 * PI);

  return (double)((circonference / CLIC_PER_ROTATION) * nbClics);
}
#pragma endregion

#pragma region DegToCM
/// <summary>
/// Converti un angle de cercle en distance en cm
/// </summary>
/// <param name="angle">Angle de cercle en degrée.
///                     Doit être entre 0 et 360.</param>
/// <param name="rayon">Rayon d'un cercle en cm</param>
/// <returns>La longueur en cm de l'arc de cercle.</returns>
float DegToCM(float angle, float rayon)
{
  // On retourne un produit croise
  return ( ((2 * PI) * (DISTANCE_ENTRE_LES_ROUES + rayon)) * (angle / 360.0));
}
#pragma endregion

#pragma region MoveForward
/// <summary>
/// Gère le déplacement en ligne droite du Robus.
/// </summary>
/// <param name="distance">Distance (cm) à atteindre</param>
/// <param name="iterations">Nombre d'itération pour atteindre la vitesse finale (accélération)</param>
/// <param name="start_speed">Vitesse initiale</param>
/// <param name="end_speed">Vitesse finale</param>
void MoveForward(double distance, int iterations, float start_speed, float end_speed)
{
  //resets values for adjustement
  ResetEncoders();
  //accelere jusqu'a vitesse max
  ACC_MASTER(start_speed, end_speed, iterations);
  while(ClicToCM( ENCODER_Read(LEFT) ) < distance)
  {
    ACC_MASTER(end_speed, end_speed, iterations);  
  }
}
#pragma endregion

#pragma region ResetEncoders
/// <summary>
/// Reset les encodeurs.
/// </summary>
void ResetEncoders()
{
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);
}
#pragma endregion

#pragma region SlaveAdjust
/// <summary>
/// Ajuste la vitesse du moteur esclave pour le synchroniser avec le moteur maitre.
/// </summary>
/// <param name="speed_master">Vitesse du moteur maitre</param>
/// <param name="ratio">Ratio voulu entre les vitesses des roues.
///                     Utilisé lors des virages (- gauche, + droite).</param>
/// <param name="correction">Correction à apporter sur la puissance de la roue 
///                          esclave pour la synchroniser avec la roue maitre</param>
/// <returns>La correction à apporter sur la nouvelle vitesse de la roue esclave.</returns>
float SlaveAdjust(float speed_master, float ratio, float correction)
{
  // Serial.println(ratio);

  int oldL = 0;
  int oldR = 0;
  float erreur;
  float erreurTotal;

  // ratio positif -> tourne a droite alors relentie la droite
  if(ratio > 0)
  {
    MOTOR_SetSpeed(LEFT, speed_master);
    MOTOR_SetSpeed(RIGHT, (speed_master / ratio) + 0.01);
  }

  // ratio negatif -> tourne a gauche alors relentie gauche
  else if(ratio < 0)
  {
    MOTOR_SetSpeed(RIGHT, speed_master);
    MOTOR_SetSpeed(LEFT, (speed_master / -ratio));
  }

  // aucun ratio -> accélération en ligne droite
  else
  {
    MOTOR_SetSpeed(LEFT, speed_master);
    MOTOR_SetSpeed(RIGHT, speed_master + correction);
    oldL = ENCODER_Read(LEFT);
    oldR = ENCODER_Read(RIGHT);
    //devrait laisser le temps de lire environ 67 coches
    delay(DELAY); //100 ok, 50 perds de la precision en longue distance 
    //garde l'erreur trouve pour cette lecture
    erreur = ((ENCODER_Read(LEFT) - oldL) - (ENCODER_Read(RIGHT) - oldR));
    erreurTotal = (ENCODER_Read(LEFT) - ENCODER_Read(RIGHT));

    correction += KI * erreur + KP * erreurTotal;
  }

  return correction;
}
#pragma endregion

#pragma region Turn
/// <summary>
/// Gère les virages de Robus.
/// </summary>
/// <param name="speed">Vitesse du moteur maitre</param>
/// <param name="rayon">Rayon du virage (+ droite, - gauche)</param>
/// <param name="angle">Angle du virage (rotation)</param>
void Turn(float speed, float rayon, float angle)
{
  float correction = 0;
  int sign = rayon < 0 ? -1 : 1;

  ResetEncoders();
  correction = SlaveAdjust(speed, CalcTurnRatio(rayon), correction);

  while(DegToCM(angle, sign*rayon) > ClicToCM(ENCODER_Read(RIGHT)))
    {
      correction = SlaveAdjust(speed, CalcTurnRatio(rayon), correction);
    }
}
#pragma endregion

#pragma region Spin

//fonction spin. de preference une vitesse d'environ 0.4 devrait etre ideale
//recoit une vitesse et angle a tourner
//angle negatif a gauche, angle positif a droite
//nice
void spin(float v, float angle){
  if(angle < 0){
    ResetEncoders();
    MOTOR_SetSpeed(LEFT, -(v + 0.01) ); //0.0
    MOTOR_SetSpeed(RIGHT, (v - 0)); // 0.01 
    while(DegToCM(- (angle - 10), DISTANCE_ENTRE_LES_ROUES / -2.) > ClicToCM(ENCODER_Read(RIGHT))){
    }
    MOTOR_SetSpeed(LEFT, 0);
    MOTOR_SetSpeed(RIGHT, 0);
  }
  else{
    ResetEncoders();
    MOTOR_SetSpeed(LEFT, v);
    MOTOR_SetSpeed(RIGHT, -(v - 0.01));
    while(DegToCM(angle, DISTANCE_ENTRE_LES_ROUES / -2.) > ClicToCM(ENCODER_Read(LEFT))){
    }
    MOTOR_SetSpeed(LEFT, 0);
    MOTOR_SetSpeed(RIGHT, 0);
  }
}
#pragma endregion

/*void turnOnSelf( float angle, float speed )
{
  ResetEncoders();
  if (angle < 0)
  {
    while (ClicToCM(ENCODER_Read(RIGHT)) < PI*DISTANCE_ENTRE_LES_ROUES*0.5);
    MOTOR_SetSpeed(RIGHT,speed);
    while (ClicToCM(ENCODER_Read(LEFT)) > PI*DISTANCE_ENTRE_LES_ROUES*0.5);
    MOTOR_SetSpeed(LEFT,-speed);
  }
  else if (angle > 0)
  {
  while (ClicToCM(ENCODER_Read(RIGHT)) > PI*DISTANCE_ENTRE_LES_ROUES*0.5);
  MOTOR_SetSpeed(RIGHT,-speed);
  while (ClicToCM(ENCODER_Read(LEFT)) < PI*DISTANCE_ENTRE_LES_ROUES*0.5);
  MOTOR_SetSpeed(LEFT,speed);
  }
  
}*/

#pragma region DoParcours
// Exécution du défi du parcours
int DoParcours()
{
  Serial.println("MoveForward1");
  MoveForward(186, 20, 0, 0.8);
  MoveForward(0, 6, 0.8, 0.6);

  Serial.println("Turn1");
  Turn(0.6, -3.0, 90);
  Turn(0.6, 18, 180);
  Turn(0.6, -3.0, 52);

  Serial.println("MoveForward2");
  MoveForward(0, 6, 0.6, 0.7);
  MoveForward(38, 5, 0.7, 0.7);
  MoveForward(0, 6, 0.7, 0.6);

  Serial.println("Turn2");
  Turn(0.6, -3.0, 68);

  Serial.println("MoveForward3");
  MoveForward(0, 6, 0.6, 0.7);
  MoveForward(22, 5, 0.7, 0.7);
  MoveForward(0, 6, 0.7, 0.6);
  
  Serial.println("Turn3");
  Turn(0.6, 12, 42);

  Serial.println("MoveForward4");
  MoveForward(0, 6, 0.6, 0.7);
  MoveForward(20, 5, 0.7, 0.7);
  MoveForward(0, 6, 0.7, 0.6);

  Serial.println("Turn4");
  Turn(0.6, 12, 16);

  Serial.println("MoveForward5");
  MoveForward(0, 6, 0.6, 0.7);
  MoveForward(68, 5, 0.7, 0.7);
  MoveForward(0, 10, 0.7, 0);    
}
#pragma endregion

/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
// -> Se fait appeler au debut du programme
// -> Se fait appeler seulement un fois
// -> Generalement on y initilise les varibbles globales

void setup(){
  BoardInit();
  Serial.begin(9600);
}


/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop() 
{
  delay(10);// Delais pour décharger le CPU

  //test pour l'avance
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  float correction = 0;

  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);

  /* MoveForward(0, 10, 0.7, 0);
  
  turnOnSelf(180, 0.5);
  MoveForward(0, 10, 0, 0); */
        
  if(ROBUS_IsBumper(REAR))
  {
    DoParcours();
  }

  if(ROBUS_IsBumper(LEFT))
  {
    ENCODER_Reset(LEFT);
    ENCODER_Reset(RIGHT);
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    // ACC_MASTER(0, 0.7);

    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      correction = SlaveAdjust(0.7, 0, correction);
    }
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      correction = SlaveAdjust(0.7, 0, correction);
    }
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      correction = SlaveAdjust(0.7, 0, correction);
    }
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      correction = SlaveAdjust(0.7, 0, correction);
    }
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      correction = SlaveAdjust(0.7, 0, correction);
    }
  }

  if(ROBUS_IsBumper(RIGHT))
  {
    ENCODER_Reset(LEFT);
    ENCODER_Reset(RIGHT);
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    // ACC_MASTER(0, 0.7, 10);

    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      correction = SlaveAdjust(0.4, 0, correction);
    }
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      correction = SlaveAdjust(0.4, 0, correction);
    }
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      correction = SlaveAdjust(0.4, 0, correction);
    }
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      correction = SlaveAdjust(0.4, 0, correction);
    }
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      correction = SlaveAdjust(0.4, 0, correction);
    }
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
  }

  if(ROBUS_IsBumper(FRONT))
  {   
    ENCODER_ReadReset(MOTOR_MASTER);
    ENCODER_ReadReset(MOTOR_SLAVE);
    ACC_MASTER(0, 0.9999, 10);
    while (ClicToCM(ENCODER_Read(MOTOR_MASTER))<180)
    {
     correction = SlaveAdjust(0.9999, 0, correction);  
    }
    ACC_MASTER(0.9999, 0, 10);
  }

}