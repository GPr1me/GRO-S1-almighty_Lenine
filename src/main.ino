/*
Projet: Défi du parcours
Equipe: 21
Auteurs: Antoine Lesieur, Charles Maheu, Vincent Pelletier, Santiago Pereira, 
         William Rousseau, Émile Rousseau-Pinard, Philippe Théroux, Marc-Olivier Thibault
Description: Permet à Robus d'MoveFoward et de Turn en se basant sur l'angle de rotation de chaque roues.
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

//variables pour gerer la correction
//correction vient du total de erreur * KP + erreurTotal * KI
//float correction;

float erreur;
float erreurTotal;
int oldL;
int oldR;

const float DELAY = 20.0;
const float KI = 0.0007;//0.0005 ok
const float KP = 0.00001;//0.00001 ok 
//try KP: 0.0001 trop grand, 0.001 pire, 0.01 nope, 0.000001 better hahaha, 0.00001, 0.0005
//try KI: 0.000002 trop grand, 0.00002 pire, 0.0002 nope, 0.00000002 lol, 0.0000002, 0.00001
const int MOTOR_MASTER = 0;
const int MOTOR_SLAVE = 1;
const float DISTANCE_ENTRE_LES_ROUES = 19.05;
//3200 coches par tour de roue
//LEFT 0, RIGHT 1, FRONT 2, REAR 3
//constante clics/cm;


/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */

//ratio marche bien avec adaptation vitesse et fonction Turn
//par contre, avec des plus grands rayons. fonction angle a cm a tendance a ne plus donner les distances voulues
//peut-etre mettre en double

float CalcTurnRation(float rayon)
//FONCTION POUR CALCULER LE RATIO DE VIRAGE
{
  float resultat=0;
    if (rayon>0)
    {
      resultat = (rayon + DISTANCE_ENTRE_LES_ROUES)/rayon ;
      // L'utilite de ce ratio c'est qu'avec un ratio donné, on va pouvoir
      // dire aux 2 roues de Turn a la meme vitesse, mais en diviser une par
      // le ratio pour qu'elle tourne moins vite.
      // Donc, dependemment de quelle roue on divise par le ratio, le robot
      // va Turn a droite ou a gauche et la seule chose qui va varier c'est le rayon.
    }
    if (rayon<0)
    {
      resultat = (-1)*(rayon - DISTANCE_ENTRE_LES_ROUES)/rayon ;
    }
  
return resultat;
}

float DegToCM(float angle, float rayon) //l'angle doit etre entre 0 et 360 deg
{
  //degre
  return ( ((2 * PI) * (DISTANCE_ENTRE_LES_ROUES + rayon)) * (angle / 360.0));
  // On retourne un produit croise
}


double ClicToCM(long int nb_de_clics)
// FONCTION POUR CHANGER LES CLICS EN VITESSE
{
  // Le nom des variables est long mais j'voulais être sûr que vous sachiez ce que je faisais
  double circonference = (2. * 38 / 10 * PI);
  int clics_par_tour=3200;
  // nombre de cm
  return (double)((circonference / clics_par_tour) * nb_de_clics);
}

// Vitesse à 70% : 6776 clics par seconde
//Vitesse à 100% : 9635.5 clics par seconde
//Clics par cm = 23.876160
// (200+45/2)+50+45+50+(18+45/2)+(54+45/2)+(60+45/2)+50+76= 693.00
// La distance du trajet est de 693.00cm
//Le robot va faire 16546.17888 clics au total
// La vitesse doit être de 4% pour que le robot fasse le trajet en 60 secondes

void ACC_MASTER(float ini_speed, float fin_speed, int nb_iterations)
// FONCTION POUR GERER L'ACCELERATION
// La fonction est faite pour le moteur gauche en tant que MOTOR_MASTER
// Si vous avez besoin du droit comme MOTOR_MASTER changez 
// MOTOR_SetSpeed(0, i) pour MOTOR_SetSpeed(1, i)
{
float correction = 0;

  if (ini_speed < fin_speed)
  // La diff entre les 2 if c'est que la vitesse finale va etre plus petite 
  // que la vitesse initiale s'il ralentit et plus grande s'il accelere. Puisque
  // j'ai défini mon n comme étant vitesse finale - initiale, il va savoir tout seul
  // s'il faut qu'il incremente ou qu'il decremente. 
  {
    float n = (fin_speed - ini_speed)/nb_iterations;//10 pour 1 seconde
    // ici le n est diviser par 10. pour qu'il se rende à la vitesse finale en 10 loop
    // si j'avais mis un n comme 0.05 ou qq chose comme ça, vu que les vitesses changent 
    // tout le temps, le n se serait jamais rendu pile sur la vitesse souhaitée.
    for (float i = ini_speed; i <= fin_speed; i+=n)
    {
      // vitesse moteur (Gauche, allant de v_initial à v_final)
      // MOTOR_SetSpeed(LEFT, i);

      //ajout de adjustement slave
      correction = SlaveAdjust(i, 0, correction);
      // delay sujet à changement ou à l'implementation en tant que variable au besoin
      // delay(50);
    }
  }
  else if (ini_speed > fin_speed)
  {
    float n = (fin_speed - ini_speed)/nb_iterations;
    for (float i = ini_speed; i >= fin_speed; i+=n)
    {

      // MOTOR_SetSpeed(LEFT, i);
      correction = SlaveAdjust(i, 0, correction);
      // delay(50);
    }
  }
  // en gros si la vitesse finale et initiale sont pareils fait rien
  else
  {
    correction = SlaveAdjust(fin_speed, 0, correction);
  }
}

//reset values for adjustement from turns to straight lines and vice-versa
void ResetAdjust(){
  oldL = 0;
  oldR = 0;
  //correction = 0;
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);
}

//doit ajuster les moteurs slave a la bonne vitesse
//ratio utilise lors de tournant pour verifier si la vitesse des roues est bien ajustee
//marche pour vitesses constantes apres quelques iterations
// - gauche, + droite
float SlaveAdjust(float master, float ratio, float correction)
{
  Serial.println(ratio);

  //ratio positif tourne a droite alors relentie la droite
  if(ratio > 0){
    MOTOR_SetSpeed(LEFT, master);
    MOTOR_SetSpeed(RIGHT, (master / ratio) + 0.01);

  }
  //ratio negatif tourne a gauche alors relentie gauche
  else if(ratio < 0){
    MOTOR_SetSpeed(RIGHT, master);
    MOTOR_SetSpeed(LEFT, (master / -ratio));
  }
  else{
    MOTOR_SetSpeed(LEFT, master);
    MOTOR_SetSpeed(RIGHT, master + correction);
    oldL = ENCODER_Read(LEFT);
    oldR = ENCODER_Read(RIGHT);
    //devrait laisser le temps de lire environ 67 coches
    delay(DELAY); //100 ok, 50 perds de la precision en longue distance 
    //garde l'erreur trouve pour cette lecture
    erreur = ((ENCODER_Read(LEFT) - oldL) - (ENCODER_Read(RIGHT) - oldR));
    erreurTotal = (ENCODER_Read(LEFT) - ENCODER_Read(RIGHT));

    Serial.print(erreur);
    Serial.print("   ");  
    Serial.print(erreurTotal);
    Serial.print("   ");
    Serial.println(correction);
    correction += KI * erreur + KP * erreurTotal;
  }

  return correction;
}

//distance en cm a atteindre
void MoveFoward(double distance, int iterations, float vI, float vF){
  //resets values for adjustement
  ResetAdjust();
  //accelere jusqu'a vitesse max
  ACC_MASTER(vI, vF, iterations);
  while(ClicToCM( ENCODER_Read(LEFT) ) < distance){
    ACC_MASTER(vF, vF, iterations);  
  }
}

//v: vitesse a laquelle Turn 
//rayon: rayon du tournant (+ a droite, - a gauche)
//angle: rotation a faire
void Turn(float v, float rayon, float angle){
  float correction = 0;
  if(rayon < 0){
    //resets values for adjustement
    ResetAdjust();
    correction = SlaveAdjust(v, CalcTurnRation(rayon), correction);
    while(DegToCM(angle, -rayon) > ClicToCM(ENCODER_Read(RIGHT)) ){
      correction = SlaveAdjust(v, CalcTurnRation(rayon), correction);
    }
  }
  else{
    //resets values for adjustement
    ResetAdjust();
    correction = SlaveAdjust(v, CalcTurnRation(rayon), correction);
    while(DegToCM(angle, rayon) > ClicToCM(ENCODER_Read(LEFT))){
      correction = SlaveAdjust(v, CalcTurnRation(rayon), correction);
    }
  }
}

DoParcours()
{
  //test pour l'avance
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
float correction = 0;

  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
  if(ROBUS_IsBumper(REAR)){

    MoveFoward(186, 20, 0, 0.8);

    MoveFoward(0, 6, 0.8, 0.6);
    
    Turn(0.6, -3.0, 90);
    
    Turn(0.6, 18, 180);

    Turn(0.6, -3.0, 52);

    MoveFoward(0, 6, 0.6, 0.7);
    
    MoveFoward(38, 5, 0.7, 0.7);

    MoveFoward(0, 6, 0.7, 0.6);
    
    Turn(0.6, -3.0, 68);

    MoveFoward(0, 6, 0.6, 0.7);
    
    MoveFoward(22, 5, 0.7, 0.7);

    MoveFoward(0, 6, 0.7, 0.6);
    
    Turn(0.6, 12, 42);

    MoveFoward(0, 6, 0.6, 0.7);

    MoveFoward(20, 5, 0.7, 0.7);

    MoveFoward(0, 6, 0.7, 0.6);

    Turn(0.6, 12, 16);

    MoveFoward(0, 6, 0.6, 0.7);

    MoveFoward(68, 5, 0.7, 0.7);

    MoveFoward(0, 10, 0.7, 0);
        
  }
  if(ROBUS_IsBumper(LEFT)){
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

  if(ROBUS_IsBumper(RIGHT)){
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

// Pour savoir quel coter on veut Turn, il faut seulement mettre la vitesse
//la plus basse soit sur MOTOR_MASTER ou MOTOR_SLAVE.

/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
// -> Se fait appeler au debut du programme
// -> Se fait appeler seulement un fois
// -> Generalement on y initilise les varibbles globales

void setup(){
  BoardInit();
  Serial.begin(9600);
  oldL = 0;
  oldR = 0;
}


/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop() 
{
  delay(10);// Delais pour décharger le CPU
  DoParcours();
}