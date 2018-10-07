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
float correctionR;

float erreur;
float erreurTotal;
int oldL;
int oldR;

const float KP = 0.01;
const float KI = 0.0002; 
//try KP: 0.0001 trop petit, 0.001 mieux mais pas assez, 0.01 
//try KI: 0.000002 trop petit, 0.00002 encore trop petit, 0.0002
// DISTANCE_PAR_CLIC
//const int TEMPS_PAUSE
const int MOTOR_MASTER = 0;
const int MOTOR_SLAVE = 1;
//3200 coches par tour de roue
//LEFT 0, RIGHT 1, FRONT 2, REAR 3
const float clics_par_cm = 23.876160;
//constante clics/cm;


/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */
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
  // LE CHIFFRE 14 EST PAS BON VEUILLEZ MESURER
  // LE CHIFFRE 14 EST PAS BON VEUILLEZ MESURER
  // LE CHIFFRE 14 EST PAS BON VEUILLEZ MESURER
  // LE CHIFFRE 14 EST PAS BON VEUILLEZ MESURER
  float distance_entre_les_roues = 14;
  float resultat = (rayon + distance_entre_les_roues)/rayon ;
  // L'utilite de ce ratio c'est qu'avec un ratio donné, on va pouvoir
  // dire aux 2 roues de tourner a la meme vitesse, mais en diviser une par
  // le ratio pour qu'elle tourne moins vite.
  // Donc, dependemment de quelle roue on divise par le ratio, le robot
  // va tourner a droite ou a gauche et la seule chose qui va varier c'est le rayon.
  return resultat;
}

float clic_to_speed(int nb_de_clics, float duree)
// FONCTION POUR CHANGER LES CLICS EN VITESSE
{
  // Le nom des variables est long mais j'voulais être sûr que vous sachiez ce que je faisais
  float circonference=(float)38/1000;
  int clics_par_tour=3200;
  // nombre de metres
  float nb_m = (float)(circonference/clics_par_tour)*nb_de_clics;
  // vitesse metres par seconde
  float V_m_par_s = nb_m/duree;
  return V_m_par_s;
}

// Vitesse à 70% : 6776 clics par seconde
//Vitesse à 100% : 9635.5 clics par seconde
//Clics par cm = 23.876160 clics par cm
// (200+45/2)+50+45+50+(18+45/2)+(54+45/2)+(60+45/2)+50+76= 693.00
// La distance du trajet est de 693.00cm
//Le robot va faire 16546.17888 clics au total
// La vitesse doit être de 4% pour que le robot fasse le trajet en 60 secondes

void ACC_MASTER(float ini_speed, float fin_speed)
// FONCTION POUR GERER L'ACCELERATION
// La fonction est faite pour le moteur gauche en tant que MOTOR_MASTER
// Si vous avez besoin du droit comme MOTOR_MASTER changez 
// MOTOR_SetSpeed(0, i) pour MOTOR_SetSpeed(1, i)
{
  if (ini_speed<fin_speed)
  // La diff entre les 2 if c'est que la vitesse finale va etre plus petite 
  // que la vitesse initiale s'il ralentit et plus grande s'il accelere. Puisque
  // j'ai défini mon n comme étant vitesse finale - initiale, il va savoir tout seul
  // s'il faut qu'il incremente ou qu'il decremente. 
  {
    float n = (fin_speed-ini_speed)/10.;
    // ici le n est diviser par 10. pour qu'il se rende à la vitesse finale en 10 loop
    // si j'avais mis un n comme 0.05 ou qq chose comme ça, vu que les vitesses changent 
    // tout le temps, le n se serait jamais rendu pile sur la vitesse souhaitée.
    for (int i=ini_speed; i<=fin_speed; i+=n)
    {
      // vitesse moteur (Gauche, allant de v_initial à v_final)
      // MOTOR_SetSpeed(LEFT, i);

      //ajout de adjustement slave
      slaveAdujst(i, 0);
      
      // delay sujet à changement ou à l'implementation en tant que variable au besoin
      delay(50);
    }
  }
  else if (ini_speed>fin_speed)
  {
    float n = (fin_speed-ini_speed)/10.;
    for (int i=ini_speed; i>=fin_speed; i+=n)
    {

      // MOTOR_SetSpeed(LEFT, i);
      slaveAdujst(i, 0);
      delay(50);
    }
  }
  // en gros si la vitesse finale et initiale sont pareils fait rien
  else
  {
    slaveAdujst(fin_speed, 0);
    return;
  }
}
float arc_de_cercle ( float angle, float rayon)
{
  return (2*PI*rayon*angle)/360;
  // la longueur de l'arc est la distance que la roue va parcourir
  //mettre un petit rayon pour que le robot tourne assez vite sans que les roues arretes
}
//doit ajuster les moteurs slave a la bonne vitesse
//ratio utilise lors de tournant pour verifier si la vitesse des roues est bien ajustee
void slaveAdujst(float master, float ratio)
{
  MOTOR_SetSpeed(LEFT, master);
  MOTOR_SetSpeed(RIGHT, master + correctionR);
  //devrait laisser le temps de lire environ 67 coches
  delay(10);
    //garde l'erreur trouve pour cette lecture
  erreur = ((ENCODER_Read(LEFT) - oldL) - (ENCODER_Read(RIGHT) - oldR));
  oldL = ENCODER_Read(LEFT);
  oldR = ENCODER_Read(RIGHT);
  erreurTotal = oldL - oldR;
  if(erreurTotal <= 4){
    correctionR = 0;
  }
  else{
    correctionR = KI * erreur + KP * erreurTotal;
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
  correctionR = 0;
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
    ENCODER_Reset(LEFT);
    ENCODER_Reset(RIGHT);
    MOTOR_SetSpeed(LEFT, 0.7);
    MOTOR_SetSpeed(RIGHT, 0.7);
    delay(2000);
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
    
    MOTOR_SetSpeed(LEFT, 0);
    MOTOR_SetSpeed(RIGHT, 0);
  }
  if(ROBUS_IsBumper(LEFT)){
    MOTOR_SetSpeed(LEFT, 1.);
    MOTOR_SetSpeed(RIGHT, 1.);
    delay (1000);
    MOTOR_SetSpeed(LEFT, 0.);
    MOTOR_SetSpeed(RIGHT, 0.);
  }

  if(ROBUS_IsBumper(RIGHT)){
    ENCODER_Reset(LEFT);
    ENCODER_Reset(RIGHT);
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
    // ACC_MASTER(0, 0.7);

    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 100; i++){
      slaveAdujst(0.7, 0);
    }
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 100; i++){
      slaveAdujst(0.7, 0);
    }
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 100; i++){
      slaveAdujst(0.7, 0);
    }
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 100; i++){
      slaveAdujst(0.7, 0);
    }
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
    //fait ca pendant environ 1 seconde
    for(int i = 0; i < 100; i++){
      slaveAdujst(0.7, 0);
    }
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
    for(int i = 0; i < 100; i++){
      slaveAdujst(0.7, 0);
    }
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
    for(int i = 0; i < 100; i++){
      slaveAdujst(0.7, 0);
    }
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
    for(int i = 0; i < 100; i++){
      slaveAdujst(0.7, 0);
    }
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
    for(int i = 0; i < 100; i++){
      slaveAdujst(0.7, 0);
    }
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
    for(int i = 0; i < 100; i++){
      slaveAdujst(0.7, 0);
    }
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
    for(int i = 0; i < 100; i++){
      slaveAdujst(0.7, 0);
    }
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
    for(int i = 0; i < 100; i++){
      slaveAdujst(0.7, 0);
    }
    Serial.print(ENCODER_Read(LEFT));
    Serial.print("  ");
    Serial.println(ENCODER_Read(RIGHT));
  }
}


// void loop()
// {
//   // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
//   delay(10);// Delais pour décharger le CPU
//   if(ROBUS_IsBumper(REAR))
//   {
//     ACC_MASTER(0, 0.8);
//     MOTOR_SetSpeed (LEFT, 0.8);
//     MOTOR_SetSpeed (RIGHT, 0.8);
//     delay (1500);
//     ACC_MASTER (0.8, 0.1);

//     arc_de_cercle (90, 2.0);
//     ratio_de_virage (2.0);
    
//   }


// }

/*void loop()
{
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  delay(10);// Delais pour décharger le CPU
  if(ROBUS_IsBumper(REAR))
  {
    ACC_MASTER(0, 0.8);
    delay()

    //total premiere distance = 222.5cm
    MOTOR_SetSpeed (LEFT, 0.8);
    MOTOR_SetSpeed (RIGHT, 0.8);
    delay (1500);
    ACC_MASTER (0.8, 0.1);

    arc_de_cercle (90, 2.0);
    ratio_de_virage (2.0);
    clic_to_speed(36, 400); //valeurs prisent au hasard
  }


}
*/

//int main(int argc, char const *argv[])

//{
  
  //débuter avec l'accélération
  // distance de 200cm
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
  //distance de 18cm
  //ajouter correctspeed

  //distance à parcourir des roues avec l'arc de cercle et le ratio
  // angle de 45deg

  //accélération
  //distance de 54cm
  //ajouter correctspeed

  //distance à parcourir des roues avec l'arc de cercle et le ratio
  // angle de 90deg

  //accélération
  //distance de 60cm
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
