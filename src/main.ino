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
// DISTANCE_PAR_CLIC
//const int TEMPS_PAUSE
const int MOTOR_MASTER = 0;
const int MOTOR_SLAVE = 1;
//3200 coches par tour de roue
//LEFT 0, RIGHT 1, FRONT 2, REAR 3

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

void ACC_MASTER(float fin_speed, float ini_speed)
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
      MOTOR_SetSpeed(0, i);
      // delay sujet à changement ou à l'implementation en tant que variable au besoin
      delay(50);
    }
  }
  else if (ini_speed>fin_speed)
  {
    float n = (fin_speed-ini_speed)/10.;
    for (int i=ini_speed; i>=fin_speed; i+=n)
    {
      MOTOR_SetSpeed(0, i);
      delay(50);
    }
  }
  // en gros si la vitesse finale et initiale sont pareils fait rien
  else
  {
    return;
  }
}
float arc_de_cercle ( float angle, float rayon)
{
  return (2*PI*rayon*angle)/360;
  // la longueur de l'arc est la distance que la roue va parcourir
  //mettre un petit rayon pour que le robot tourne assez vite sans que les roues arretes
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
}


/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop() {
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  delay(10);// Delais pour décharger le CPU

}



int main(void)
{
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


  return 0;
}
