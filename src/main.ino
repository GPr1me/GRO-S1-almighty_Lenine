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
#include <Stream.h>


/* ****************************************************************************
Variables globales et defines
**************************************************************************** */
// -> defines...
// L'ensemble des fonctions y ont acces

//variables pour gerer la correction
//correction vient du total de erreur * KP + erreurTotal * KI
float correction;

float erreur;
float erreurTotal;
long int oldL;
long int oldR;

const float DELAY = 20.0;
const float KI = 0.0007;//0.0005 ok
const float KP = 0.00001;//0.00001 ok 
//try KP: 0.0001 trop grand, 0.001 pire, 0.01 nope, 0.000001 better hahaha, 0.00001, 0.0005
//try KI: 0.000002 trop grand, 0.00002 pire, 0.0002 nope, 0.00000002 lol, 0.0000002, 0.00001
// DISTANCE_PAR_CLIC
//const int TEMPS_PAUSE
const int MOTOR_MASTER = 0;
const int MOTOR_SLAVE = 1;
const float distance_entre_les_roues = 19.05;
//3200 coches par tour de roue
//LEFT 0, RIGHT 1, FRONT 2, REAR 3
//constante clics/cm;


/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */

//ratio marche bien avec adaptation vitesse et fonction tourner
//par contre, avec des plus grands rayons. fonction angle a cm a tendance a ne plus donner les distances voulues
//peut-etre mettre en double


void maFonction(){
  // code
}

void correctSpeed(int cycleNb,
                  int clicNbGoal, 
                  int actualTotalClicNb,
                  float speedRatio)
{
  float MotorMaster_actualClicNb = ENCODER_Read(MOTOR_MASTER);
}

float ratio_de_virage(float rayon)
//FONCTION POUR CALCULER LE RATIO DE VIRAGE
{
  float resultat=0;
    if (rayon > 0)
    {
      resultat = (rayon + distance_entre_les_roues)/rayon ;
      // L'utilite de ce ratio c'est qu'avec un ratio donné, on va pouvoir
      // dire aux 2 roues de tourner a la meme vitesse, mais en diviser une par
      // le ratio pour qu'elle tourne moins vite.
      // Donc, dependemment de quelle roue on divise par le ratio, le robot
      // va tourner a droite ou a gauche et la seule chose qui va varier c'est le rayon.
    }
    if (rayon < 0)
    {
      resultat = (-1)*(rayon - distance_entre_les_roues)/rayon ;
    }
  
return resultat;
}

float angle_to_cm(float angle, float rayon) //l'angle doit etre entre 0 et 360 deg
{
  //degre
  return ( ((2 * PI) * (distance_entre_les_roues + rayon)) * (angle / 360.0));
  // On retourne un produit croise
}


double clic_to_cm(long int nb_de_clics)
// FONCTION POUR CHANGER LES CLICS EN VITESSE
{
  // Le nom des variables est long mais j'voulais être sûr que vous sachiez ce que je faisais
  double circonference = (2. * 38 / 10 * PI);
  int clics_par_tour=3200;
  // nombre de cm
  double nb_cm = (double)((circonference / clics_par_tour) * nb_de_clics);
  return nb_cm;
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
  //avec valeurs negatives
  if(fin_speed < 0 || ini_speed < 0){
    //si accelere en reculant
    if (ini_speed > fin_speed)
    // La diff entre les 2 if c'est que la vitesse finale va etre plus petite 
    // que la vitesse initiale s'il ralentit et plus grande s'il accelere. Puisque
    // j'ai défini mon n comme étant vitesse finale - initiale, il va savoir tout seul
    // s'il faut qu'il incremente ou qu'il decremente. 
    {
      float n = (fin_speed - ini_speed) / nb_iterations;//10 pour 1 seconde
      // ici le n est diviser par 10. pour qu'il se rende à la vitesse finale en 10 loop
      // si j'avais mis un n comme 0.05 ou qq chose comme ça, vu que les vitesses changent 
      // tout le temps, le n se serait jamais rendu pile sur la vitesse souhaitée.

      //accelere vers une plus petite valeur
      for (float i = ini_speed; i >= fin_speed; i+=n)
      {
        // vitesse moteur (Gauche, allant de v_initial à v_final)
        // MOTOR_SetSpeed(LEFT, i);

        //ajout de adjustement slave
        slaveAdujst(i, 0);
        // delay sujet à changement ou à l'implementation en tant que variable au besoin
        // delay(50);
      }
    }
    //si ralentit en reculant
    else if (ini_speed < fin_speed)
    {
      float n = (fin_speed - ini_speed)/nb_iterations;
      for (float i = ini_speed; i <= fin_speed; i+=n)
      {

        // MOTOR_SetSpeed(LEFT, i);
        slaveAdujst(i, 0);
        // delay(50);
      }
    }
    // en gros si la vitesse finale et initiale sont pareils fait rien
    else
    {
      slaveAdujst(fin_speed, 0);
      return;
    }
  }
  else{

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
        slaveAdujst(i, 0);
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
        slaveAdujst(i, 0);
        // delay(50);
      }
    }
    // en gros si la vitesse finale et initiale sont pareils fait rien
    else
    {
      slaveAdujst(fin_speed, 0);
      return;
    }
  }
}


/*float arc_de_cercle ( float angle, float rayon)
{
  return (2*PI*rayon*angle)/360;
  // la longueur de l'arc est la distance que la roue va parcourir
  //mettre un petit rayon pour que le robot tourne assez vite sans que les roues arretes
}
*/

//reset values for adjustement from turns to straight lines and vice-versa
void resetAdjust(){
  oldL = 0;
  oldR = 0;
  correction = 0;
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);
}

//doit ajuster les moteurs slave a la bonne vitesse
//ratio utilise lors de tournant pour verifier si la vitesse des roues est bien ajustee
//marche pour vitesses constantes apres quelques iterations
// - gauche, + droite
void slaveAdujst(float master, float ratio)
{
  Serial.println(ratio);
  if(master < 0){
    //TODO
    //ratios pour tourner de reculons sont a retravailler
    //en jouant avec on peut faire qu'un angle de 180 et 90 fassent les
    //tournants souhaitees avec differentes valeurs les tournant effectues varient

    //ratio positif tourne a gauche alors relentie la droite (ok) (over) 
    if(ratio > 0){
      MOTOR_SetSpeed(LEFT, master - 0.02);
      MOTOR_SetSpeed(RIGHT, (master / ratio)); //-0.01
      
    }
    //ratio negatif tourne a droite alors relentie gauche (droite a adapter) tourne trop
    else if(ratio < 0){
      MOTOR_SetSpeed(RIGHT, master );//0.08
      MOTOR_SetSpeed(LEFT, (master / -ratio) - 0.01);
      
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
      // if(erreurTotal <= 4){
      //   correctionR;
      // }
      // else{
      Serial.print(erreur);
      Serial.print("   ");  
      Serial.print(erreurTotal);
      Serial.print("   ");
      Serial.println(correction);
      correction += KI * erreur + KP * erreurTotal;
      // Serial.println(correctionR);
    }
  }
  else{
    //ratio positif tourne a droite alors relentie la droite
    if(ratio > 0){
      MOTOR_SetSpeed(LEFT, master);
      MOTOR_SetSpeed(RIGHT, (master / ratio) + 0.01);
      
    }
    //ratio negatif tourne a gauche alors relentie gauche
    else if(ratio < 0){
      MOTOR_SetSpeed(RIGHT, master);
      MOTOR_SetSpeed(LEFT, (master / -ratio));
      // oldR = ENCODER_Read(RIGHT);
      // oldL = ENCODER_Read(LEFT);
      //devrait laisser le temps de lire environ 67 coches
      // delay(DELAY); //100 ok, 50 perds de la precision en longue distance 
      //garde l'erreur trouve pour cette lecture
      // erreur = ((ENCODER_Read(RIGHT) - oldR) - ( (ENCODER_Read(LEFT) - oldL) * -ratio) );
      // erreurTotal = (ENCODER_Read(RIGHT) - ENCODER_Read(LEFT) * -ratio);
      // if(erreurTotal <= 4){
      //   correctionR;
      // }
      // else{
      // Serial.print(erreur);
      // Serial.print("   ");  
      // Serial.print(erreurTotal);
      // Serial.print("   ");
      // Serial.println(correction);
      // correction += KI * erreur + KP * erreurTotal;
      // Serial.println(correctionR);

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
      // if(erreurTotal <= 4){
      //   correctionR;
      // }
      // else{
      Serial.print(erreur);
      Serial.print("   ");  
      Serial.print(erreurTotal);
      Serial.print("   ");
      Serial.println(correction);
      correction += KI * erreur + KP * erreurTotal;
      // Serial.println(correctionR);
    }
  }
}

//distance en cm a atteindre
void avancer(double distance, int iterations, float vI, float vF){
  if(vF < 0 || vI < 0){
    //resets values for adjustement
    resetAdjust();
    //accelere jusqu'a vitesse max
    ACC_MASTER(vI, vF, iterations);
    while(clic_to_cm( ENCODER_Read(LEFT) ) > distance){
      ACC_MASTER(vF, vF, iterations);  
    }
  }
  else{
    //resets values for adjustement
    resetAdjust();
    //accelere jusqu'a vitesse max
    ACC_MASTER(vI, vF, iterations);
    while(clic_to_cm( ENCODER_Read(LEFT) ) < distance){
      ACC_MASTER(vF, vF, iterations);  
    }
  }
}

//v: vitesse a laquelle tourner 
//rayon: rayon du tournant (+ a droite, - a gauche)
//angle: rotation a faire
void tourner(float v, float rayon, float angle){
  if(v < 0){
    //tourne a droite
    if(rayon < 0){
      //resets values for adjustement
      resetAdjust();
      slaveAdujst(v, ratio_de_virage(rayon));
      while(angle_to_cm(angle, -rayon) > -clic_to_cm(ENCODER_Read(RIGHT)) ){
        slaveAdujst(v, ratio_de_virage(rayon));
      }
    }
    //tourne a gauche
    else{
      //resets values for adjustement
      resetAdjust();
      slaveAdujst(v, ratio_de_virage(rayon));
      while(angle_to_cm(angle, rayon) > -clic_to_cm(ENCODER_Read(LEFT))){
        slaveAdujst(v, ratio_de_virage(rayon));
      }
    }
  }
  else{
    //tourne a gauche
    if(rayon < 0){
      //resets values for adjustement
      resetAdjust();
      slaveAdujst(v, ratio_de_virage(rayon));
      while(angle_to_cm(angle, -rayon) > clic_to_cm(ENCODER_Read(RIGHT)) ){
        slaveAdujst(v, ratio_de_virage(rayon));
      }
    }
    //tourne a droite
    else{
      //resets values for adjustement
      resetAdjust();
      slaveAdujst(v, ratio_de_virage(rayon));
      while(angle_to_cm(angle, rayon) > clic_to_cm(ENCODER_Read(LEFT))){
        slaveAdujst(v, ratio_de_virage(rayon));
      }
    }
  }
}


// Pour savoir quel coter on veut tourner, il faut seulement mettre la vitesse
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
  correction = 0;
  oldL = 0;
  oldR = 0;
}


/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop() { //test pour l'avance
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  delay(10);// Delais pour décharger le CPU
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);
  if(ROBUS_IsBumper(REAR)){

    avancer(186, 20, 0, 0.8);

    avancer(0, 6, 0.8, 0.6);
    
    tourner(0.6, -3.0, 90);
    
    tourner(0.6, 18, 180);

    tourner(0.6, -3.0, 52);

    avancer(0, 6, 0.6, 0.7);
    
    avancer(40, 5, 0.7, 0.7);

    avancer(0, 6, 0.7, 0.4);
    
    tourner(0.4, -3.0, 68);

    avancer(0, 12, 0.4, 0.7);
    
    avancer(15, 5, 0.7, 0.7);

    avancer(0, 6, 0.7, 0.6);
    
    tourner(0.6, 12, 42);

    avancer(0, 6, 0.6, 0.7);

    avancer(18, 5, 0.7, 0.7);

    avancer(0, 6, 0.7, 0.6);

    tourner(0.6, 12, 12);

    avancer(0, 6, 0.6, 0.7);

    avancer(58, 5, 0.7, 0.7);

    avancer(0, 15, 0.7, 0);
    
    // FIN ALLÉ
    delay(200);
    
    avancer(-55, 20, 0, -0.7); // Premiere ligne retour

    avancer(0, 10, -0.7, -0.4); // Deceleration

    tourner(-0.4, 12.0, 22); // Premier tournant retour

    avancer(0, 10, -0.4, -0.7); // accel ligne 2
    
    avancer(-6, 5, -0.7, -0.7); // ligne 2

    avancer(0, 10, -0.7, -0.4); // decel ligne 2

    tourner(-0.4, 12.0, 53); // tournant pour ligne 3

    avancer(0, 10, -0.4, -0.7); // accel ligne 3

    avancer(-29.5, 10, -0.7, -0.7); // ligne 3

    avancer(0, 10, -0.7, -0.4); // decel ligne 3
 
    tourner(-0.4, -3.0, 95); // tournant 90

    avancer(0, 10, -0.4, -0.7); // accel ligne 4

    avancer(-1.5, 5, -0.7, -0.7); // ligne 4

    avancer(0, 10, -0.7, -0.4); // decel ligne 4

    tourner(-0.4, 3.0, 62); // tournant pour ligne 5

    tourner(-0.4, -8.0, 92); // tournant pour ligne 6

    tourner(-0.4, 10.0, 90); //tournant pour U turn 1/2

    tourner(-0.4, 10.0, 102); //tournant pour U turn 1/2

    tourner(-0.4, -28.0, 78); //tournant vers ligne finale

    avancer(-190, 12, -0.4, -0.9); //accel final stretch

    avancer(0, 10, -0.9, 0); //decelleration du champion


  }
  if(ROBUS_IsBumper(LEFT)){
    resetAdjust();
    delay(500);
    //rayon + a gauche
    tourner(-0.6, 3., 180); //(ok)
    MOTOR_SetSpeed(LEFT, 0);
    MOTOR_SetSpeed(RIGHT, 0);
    delay(500);
    tourner(-0.6, 3., 90); //(over)
    MOTOR_SetSpeed(LEFT, 0);
    MOTOR_SetSpeed(RIGHT, 0);
    delay(500);
    tourner(-0.6, 3., 90); //(over)
    MOTOR_SetSpeed(LEFT, 0);
    MOTOR_SetSpeed(RIGHT, 0);
    delay(500);
    //rayon - a droite
    tourner(-0.6, -3., 180); //(over)
    MOTOR_SetSpeed(LEFT, 0);
    MOTOR_SetSpeed(RIGHT, 0);
    delay(500);
    tourner(-0.6, -3., 90); //(over)
    MOTOR_SetSpeed(LEFT, 0);
    MOTOR_SetSpeed(RIGHT, 0);
    delay(500);
    tourner(-0.6, -3., 90); //(over)
    MOTOR_SetSpeed(LEFT, 0);
    MOTOR_SetSpeed(RIGHT, 0);
    delay(500);
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
      slaveAdujst(0.4, 0);
    }
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      slaveAdujst(0.4, 0);
    }
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      slaveAdujst(0.4, 0);
    }
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      slaveAdujst(0.4, 0);
    }
    // Serial.print(ENCODER_Read(LEFT));
    // Serial.print("  ");
    // Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 10; i++){
      slaveAdujst(0.4, 0);
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
    while (clic_to_cm(ENCODER_Read(MOTOR_MASTER))<180)
    {
     slaveAdujst(0.9999, 0);  
    }
    ACC_MASTER(0.9999, 0, 10);
  }
}


/* void loop()
 {
   // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
   delay(10);// Delais pour décharger le CPU
   if(ROBUS_IsBumper(REAR))
   {
     ACC_MASTER(0, 0.8, 10);
     MOTOR_SetSpeed (LEFT, 0.8);
     MOTOR_SetSpeed (RIGHT, 0.8);
     delay (1500);
     ACC_MASTER (0.8, 0.1, 10);

     ratio_de_virage (1.0); // rayon de 1 cm
    
    
  }
*/

// }

/*void loop()
{
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  delay(10);// Delais pour décharger le CPU
  if(ROBUS_IsBumper(REAR))

}
*/

//int main(int argc, char const *argv[])

//{
  
  //débuter avec l'accélération
  // distance de 222.5cm
  //ajouter correctspeed

  //distance à parcourir des roues avec l'arc de cercle et le ratio
  // angle de 90deg

  //accélération
  //distance de 50cm
  //ajouter correctspeed

  //distance à parcourir des roues avec l'arc de cercle et le ratio
  // angle de 90deg

  //accélération
  //distance de 45cm
  //ajouter correctspeed

  //distance à parcourir des roues avec l'arc de cercle et le ratio
  // angle de 90deg

  //accélération
  //distance de 50cm
  //ajouter correctspeed

  //distance à parcourir des roues avec l'arc de cercle et le ratio
  // angle de 90deg

  //accélération
  //distance de 40.5cm
  //ajouter correctspeed

  //distance à parcourir des roues avec l'arc de cercle et le ratio
  // angle de 45deg

  //accélération
  //distance de 76.5cm
  //ajouter correctspeed

  //distance à parcourir des roues avec l'arc de cercle et le ratio
  // angle de 90deg

  //accélération
  //distance de 82.5cm
  //ajouter correctspeed

  //distance à parcourir des roues avec l'arc de cercle et le ratio
  // angle de 45deg

  //accélération
  //distance de 50cm
  //ajouter correctspeed

  //distance à parcourir des roues avec l'arc de cercle et le ratio
  // angle de 12.5deg

  //accélération
  //distance de 76cm
  //ajouter correctspeed


 // return 0;
