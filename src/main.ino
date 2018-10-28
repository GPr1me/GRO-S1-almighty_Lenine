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


unsigned long lastMillis1 = 0;        //will store last time error was updated
unsigned long lastMillis2 = 0; //will store time for whistle pause without exterior timer
unsigned long deltaT = 0; //will store time that is lost during a whistle break

const float DELAY = 20.0; //selon test effectues par l'equipe. 20 assez de precision. semble ok
const float KI = 0.0007;//0.0005 ok
const float KP = 0.00001;//0.00001 ok 
//try KP: 0.0001 trop grand, 0.001 pire, 0.01 nope, 0.000001 better hahaha, 0.00001, 0.0005
//try KI: 0.000002 trop grand, 0.00002 pire, 0.0002 nope, 0.00000002 lol, 0.0000002, 0.00001
// DISTANCE_PAR_CLIC
//const int TEMPS_PAUSE
const int MOTOR_MASTER = 0;
const int MOTOR_SLAVE = 1;
const float distance_entre_les_roues = 19.05;
const int clics_par_tour = 3200;
const double circonference = (2. * 38 / 10 * PI);
//3200 coches par tour de roue
//LEFT 0, RIGHT 1, FRONT 2, REAR 3
//constante clics/cm;

//variables pour si il entend le sifflet
boolean sifflet = false
;
const float ITERATIONSIFFLET = 20.;


/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */

void maFonction(){
  // code
}

//FONCTION POUR CALCULER LE RATIO DE VIRAGE
float ratio_de_virage(float rayon)
{
  float resultat = 0;
    if (rayon > 0)
    {
      resultat = (rayon + distance_entre_les_roues) / rayon ;
      // L'utilite de ce ratio c'est qu'avec un ratio donné, on va pouvoir
      // dire aux 2 roues de tourner a la meme vitesse, mais en diviser une par
      // le ratio pour qu'elle tourne moins vite.
      // Donc, dependemment de quelle roue on divise par le ratio, le robot
      // va tourner a droite ou a gauche et la seule chose qui va varier c'est le rayon
    }
    if (rayon < 0)
    {
      resultat = (-1) * (rayon - distance_entre_les_roues) / rayon;
    }
  
  return resultat;
}

//l'angle doit etre entre 0 et 360 deg
double angle_to_cm(double angle, float rayon) 
{
  //On retourne un produit croise
  return ( 2 * PI * (distance_entre_les_roues + rayon)  * angle / 360.0 );
}

// FONCTION POUR CHANGER LES CLICS EN VITESSE
double clic_to_cm(long int nb_de_clics)
{
  // Le nom des variables est long mais j'voulais être sûr que vous sachiez ce que je faisais
  // nombre de cm
  double nb_cm = (double)( circonference / clics_par_tour * nb_de_clics );
  return nb_cm;
}

//Vitesse à 70% : 6776 clics par seconde
//Vitesse à 100% : 9635.5 clics par seconde
//Clics par cm = 23.876160
//(200+45/2) +50+45+50+ (18+45/2) + (54+45/2) + (60+45/2) +50+76 = 693.00
//La distance du trajet est de 693.00cm
//Le robot va faire 16546.17888 clics au total
//La vitesse doit être de 4% pour que le robot fasse le trajet en 60 secondes

//fonction doit gerer un changement de vitesse vers le positif et un changement de vitesse vers le negatif
//FONCTION POUR GERER L'ACCELERATION
//La fonction est faite pour le moteur gauche en tant que MOTOR_MASTER
//nouveau:
//reagis au sifflet et fait arreter le code si il entend le sifflet
void ACC_MASTER(float vI, float vF, int nb_iterations, float ratio)
{
  
  //pas d'iterations, met les deux moteurs a la vitesse et adujste au besoin
  if(nb_iterations == 0){
    //si entend un sifflet pendant son execution
    if(sifflet){
      //garde temps ecoule pour le timer 
      deltaT = 0;
      entendSifflet(vI, ratio);
      sifflet = false;
    }
    slaveAdjust(vF, ratio);
  }
  else{
    unsigned long newMillis;
    lastMillis1 = millis();
    //Puisque j'ai défini mon n comme étant vitesse finale - initiale, il va savoir tout seul
    //s'il faut qu'il incremente ou qu'il decremente 
    //si veut acc vers une vitesse pour avancer de face ou relentir de reculons
    if (vI < vF)
    {
      //nombre positif a ajouter
      float n = (vF - vI) / nb_iterations;//50 pour 1 seconde
      // ici le n est diviser par 10. pour qu'il se rende à la vitesse finale en 10 loop
      // si j'avais mis un n comme 0.05 ou qq chose comme ça, vu que les vitesses changent 
      // tout le temps, le n se serait jamais rendu pile sur la vitesse souhaitée.
      
      for (float i = vI; i <= vF;)
      {
        
        //si entend un sifflet pendant son execution
        if(sifflet){
          //garde temps ecoule pour le timer 
          deltaT = newMillis - lastMillis1;
          entendSifflet(i, ratio);
          sifflet = false;
        }

        newMillis = millis();

        if(newMillis - lastMillis1 > DELAY ){
          //accelere avec adjustement
          slaveAdjust(i, ratio);
          lastMillis1 = newMillis;
          i+=n;
        }
        //accelere avec adjustement
        // slaveAdjust(i, ratio); //delay de 20 ms
      }
    }
    //si veut acc vers une vitesse pour avancer de reculons ou relentir de face
    else if (vI > vF)
    {
      //nombre negatif a enlever
      float n = (vF - vI) / nb_iterations;

      for (float i = vI; i >= vF;)
      {
        //si entend un sifflet pendant son execution
        if(sifflet){
          //garde temps ecoule pour le timer 
          deltaT = newMillis - lastMillis1;
          entendSifflet(i, ratio);
          sifflet = false;
        }

        newMillis = millis();
        if(newMillis - lastMillis1 > DELAY ){
          //accelere avec ajustement
          slaveAdjust(i, ratio);
          lastMillis1 = newMillis;
          i+=n;
        }
        //accelere avec ajustement
        // slaveAdjust(i, ratio); //delay 20 ms
      }  
    }
    // en gros si la vitesse finale et initiale sont pareils. fait juste appliquer l'ajustement
    else
    {
      //si entend un sifflet pendant son execution
      if(sifflet){
        //garde temps ecoule pour le timer 
        deltaT = newMillis - lastMillis1;
        entendSifflet(vF, ratio);
        sifflet = false;
      }
      slaveAdjust(vF, ratio); 
    }
  }

}

//reset values for adjustement from turns to straight lines and vice-versa
void resetAdjust(){
  oldL = 0;
  oldR = 0;
  correction = 0;
  ENCODER_Reset(LEFT);
  ENCODER_Reset(RIGHT);
}

//doit ajuster les moteurs slave a la bonne vitesse (+/-)
//ratio utilise lors des tournant pour verifier si la vitesse des roues est bien ajustee
//marche pour vitesses constantes apres quelques iterations
//ratio: - gauche, + droite
void slaveAdjust(float master, float ratio)
{
  // Serial.println(ratio);
  
  //ratio + tourne a droite alors relentie la droite
  //master set a droite vu que roue la plus lente
  //assure que les deux roues tournent a meme vitesse avec ajustement
  if(ratio > 0){
    
    // devrait laisser le temps de lire environ 67 coches
    // delay(DELAY);

    // garde l'erreur trouve pour cette lecture
    erreur = ( (ENCODER_Read(RIGHT) - oldR) - (ENCODER_Read(LEFT) - oldL) / ratio ); //ancien test * -ratio sur left
    erreurTotal = ( ENCODER_Read(RIGHT) - ENCODER_Read(LEFT) / ratio ); //ancien test * -ratio sur LEFT
    // Serial.print(erreur);
    // Serial.print("   ");  
    // Serial.print(erreurTotal);
    // Serial.print("   ");
    // Serial.println(correction);
    correction += KI * erreur + KP * erreurTotal;
    // Serial.println(correction);
    

    //new placement start of correction
    MOTOR_SetSpeed(LEFT, master + correction);
    MOTOR_SetSpeed(RIGHT, (master/ ratio) );
    oldR = ENCODER_Read(RIGHT);
    oldL = ENCODER_Read(LEFT);

  
    
  }
  //ratio - tourne a gauche alors relentie gauche
  //master set a gauche vu que roue la plus lente
  //assure que les deux roues tournent a meme vitesse avec ajustement
  else if(ratio < 0){
     
    //devrait laisser le temps de lire environ 67 coches
    // delay(DELAY); //100 ok, 50 perds de la precision en longue distance 
    
    //garde l'erreur trouve pour cette lecture
    erreur = ( (ENCODER_Read(LEFT) - oldL) - (ENCODER_Read(RIGHT) - oldR) / -ratio ); //ancien test * -ratio sur left
    erreurTotal = ( ENCODER_Read(LEFT) - ENCODER_Read(RIGHT) / -ratio ); //ancien test * -ratio sur LEFT
    // Serial.print(erreur);
    // Serial.print("   ");  
    // Serial.print(erreurTotal);
    // Serial.print("   ");
    // Serial.println(correction);
    correction += KI * erreur + KP * erreurTotal;
    // Serial.println(correction);

    //new placement for correction
    MOTOR_SetSpeed(LEFT, (master/ -ratio));
    MOTOR_SetSpeed(RIGHT, master + correction);
    oldR = ENCODER_Read(RIGHT);
    oldL = ENCODER_Read(LEFT);

  }
  //ligne droite avec master set a gauche
  else{
    //devrait laisser le temps de lire environ 67 coches
    // delay(DELAY); //50 ok, 20 perds de la precision en longue distance mais courte distance plus de verifications 
    //garde l'erreur trouve pour cette lecture
    erreur = ((ENCODER_Read(LEFT) - oldL) - (ENCODER_Read(RIGHT) - oldR));
    erreurTotal = (ENCODER_Read(LEFT) - ENCODER_Read(RIGHT));
    // Serial.print(erreur);
    // Serial.print("   ");  
    // Serial.print(erreurTotal);
    // Serial.print("   ");
    // Serial.println(correction);
    correction += KI * erreur + KP * erreurTotal;
    // Serial.println(correction);

    //new placement for correction
    MOTOR_SetSpeed(LEFT, master);
    MOTOR_SetSpeed(RIGHT, master + correction);
    oldL = ENCODER_Read(LEFT);
    oldR = ENCODER_Read(RIGHT);
    
  }
}
//corrects for spins with master as left
//+ value spin right 
//- value spin left
void adjustSpin(float master){

  if(master > 0){
    // MOTOR_SetSpeed(LEFT, master); //+
    // MOTOR_SetSpeed(RIGHT, - (master + correction)); //-
    // //+
    // oldL = ENCODER_Read(LEFT);
    // //-
    // oldR = ENCODER_Read(RIGHT);

    // delay(DELAY);

    erreur = (ENCODER_Read(LEFT) - oldL - (oldR - ENCODER_Read(RIGHT)));
    erreurTotal = (ENCODER_Read(LEFT) + ENCODER_Read(RIGHT));
    correction += KI * erreur + KP * erreurTotal;

    //new placement for correction
    MOTOR_SetSpeed(LEFT, master); //+
    MOTOR_SetSpeed(RIGHT, - (master + correction)); //-
    //+
    oldL = ENCODER_Read(LEFT);
    //-
    oldR = ENCODER_Read(RIGHT);
  }
  else{
    // MOTOR_SetSpeed(LEFT, master); //-
    // MOTOR_SetSpeed(RIGHT, -(master - correction));//+
    // //-
    // oldL = ENCODER_Read(LEFT);
    // //+
    // oldR = ENCODER_Read(RIGHT);

    // delay(DELAY);

    erreur = (oldL - ENCODER_Read(LEFT) - (ENCODER_Read(RIGHT) - oldR));
    erreurTotal = (-ENCODER_Read(LEFT) - ENCODER_Read(RIGHT));
    correction += KI * erreur + KP * erreurTotal;
    //new placement for correction
    MOTOR_SetSpeed(LEFT, master); //-
    MOTOR_SetSpeed(RIGHT, -(master - correction));//+
    //-
    oldL = ENCODER_Read(LEFT);
    //+
    oldR = ENCODER_Read(RIGHT);
  }
}

//distance en cm a atteindre. Positive si avance, negative si recule
//nouveau:
//reagit au sifflet et fait arreter le code si il entend le sifflet
void avancer(double distance, int iterations, float vI, float vF){
  
  //resets values for adjustement
  resetAdjust();
  //accelere/decelere jusqu'a vitesse finale
  ACC_MASTER(vI, vF, iterations, 0.);
  lastMillis1 = millis();
  unsigned long newMillis;
  //si doit avancer de reculons une fois atteint sa vitesse
  if(vF < 0){
    //boucle pour atteindre distance desiree
    //clics vont etre negatifs alors distance negative
    while(clic_to_cm( ENCODER_Read(LEFT) ) > distance){
      //si entend un sifflet pendant son execution
      if(sifflet){
        //garde temps ecoule pour le timer 
        deltaT = newMillis - lastMillis1;
        entendSifflet(vF, 0.);
        sifflet = false;
      }

      newMillis = millis();
      if(newMillis - lastMillis1 > DELAY ){
        slaveAdjust(vF, 0.);
        lastMillis1 = newMillis;
      }
      
      //continue a faire la correction
      // slaveAdjust(vF, 0.);  
    }
  }

  //si doit avancer de face a la fin une fois atteint sa vitesse
  else if(vF > 0){
    //boucle pour atteindre la distance desiree
    //clics vont etre positifs alors distance positive
    while(clic_to_cm( ENCODER_Read(LEFT) ) < distance){
      //si entend un sifflet pendant son execution
      if(sifflet){
        //garde temps ecoule pour le timer 
        deltaT = newMillis - lastMillis1;
        entendSifflet(vF, 0.);
        sifflet = false;
      }

      newMillis = millis();
      if(newMillis - lastMillis1 > DELAY ){
        slaveAdjust(vF, 0.);
        lastMillis1 = newMillis;
      }

      //continue a faire la correction
      // slaveAdjust(vF, 0.);  
    }
  }
  //si vitesse finale est de 0. Ignore la distance. Donc, relentit pendant les iterations jusqu'a l'arret
}

//v: vitesse a laquelle tourner 
//rayon: rayon du tournant (+ a droite, - a gauche)
//angle: rotation a faire
//test avec 0.4v et 0.2r (ok) decalage negligable
//test avec 0.4v et 10r: (ok) decalage negligable
//test parcours: trajet semble plus constant! (fait pas le parcours doe haha. code lenin a 0.7 sur staline)
//nouveau:
//reagit au sifflet et arrete lorsqu'il l'entend
void tourner(float vI, float vF, int iterations, float rayon, double angle){
  unsigned long newMillis;
  float ratio = ratio_de_virage(rayon);
  //cas avec vitesse negative
  if(vF < 0){
    //valeurs mises a 0 pour ajustement
    resetAdjust();

    //accelere/decelere jusqu'a vitesse finale
    ACC_MASTER(vI, vF, iterations, ratio);
    //distance si tourne a droite
    if(rayon < 0){
      lastMillis1 = millis();
      while(angle_to_cm(angle, -rayon) > -clic_to_cm(ENCODER_Read(RIGHT)) ){

        //si entend un sifflet pendant son execution
        if(sifflet){
          //garde temps ecoule pour le timer 
          deltaT = newMillis - lastMillis1;
          entendSifflet(vF, ratio);
          sifflet = false;
        }

        newMillis = millis();
        if(newMillis - lastMillis1 > DELAY ){
          slaveAdjust(vF, ratio);
          lastMillis1 = newMillis;
        }
        //applique l'ajustement a faire pour faire tourner les roues pour qu'elles parcourent les bonnes distances
        // slaveAdjust(vF, ratio_de_virage(rayon));
      }
    }
    //distance si tourne a gauche
    else{
      lastMillis1 = millis();
      while(angle_to_cm(angle, rayon) > -clic_to_cm(ENCODER_Read(LEFT)) ){

        //si entend un sifflet pendant son execution
        if(sifflet){
          //garde temps ecoule pour le timer 
          deltaT = newMillis - lastMillis1;
          entendSifflet(vF, ratio);
          sifflet = false;
        }

        newMillis = millis();
        if(newMillis - lastMillis1 > DELAY ){
          slaveAdjust(vF, ratio);
          lastMillis1 = newMillis;
        }

        //applique l'ajustement a faire pour faire tourner les roues pour qu'elles parcourent les bonnes distances
        // slaveAdjust(vF, ratio_de_virage(rayon));
      }  
    }
  }

  //cas avec vitesse positive
  else{
    //valeurs mises a 0 pour ajustement
    resetAdjust();

    //accelere/decelere jusqu'a vitesse finale
    ACC_MASTER(vI, vF, iterations, ratio);

    // //applique l'ajustement afin de tourner dans le bon sens
    // slaveAdjust(v, ratio_de_virage(rayon));
    
    //distance si tourne a gauche
    if(rayon < 0){
      while(angle_to_cm(angle, -rayon) > clic_to_cm(ENCODER_Read(RIGHT)) ){

        //si entend un sifflet pendant son execution
        if(sifflet){
          //garde temps ecoule pour le timer 
          deltaT = newMillis - lastMillis1;
          entendSifflet(vF, ratio);
          sifflet = false;
        }

        newMillis = millis();
        if(newMillis - lastMillis1 > DELAY ){
          slaveAdjust(vF, ratio);
          lastMillis1 = newMillis;
        }
        // slaveAdjust(vF, ratio_de_virage(rayon));
      }
    }
    //distance si tourne a droite
    else{
      while(angle_to_cm(angle, rayon) > clic_to_cm(ENCODER_Read(LEFT))){

        //si entend un sifflet pendant son execution
        if(sifflet){
          //garde temps ecoule pour le timer 
          deltaT = newMillis - lastMillis1;
          entendSifflet(vF, ratio);
          sifflet = false;
        }

        newMillis = millis();
        if(newMillis - lastMillis1 > DELAY ){
          slaveAdjust(vF, ratio);
          lastMillis1 = newMillis;
        }
        // slaveAdjust(vF, ratio_de_virage(rayon));
      }  
    }
  }
}

//acceleration pas possible pour le moment
//contient nouveau PID
//angle + a droite, angle - a gauche
//vitesse de 0.4 assez vite

//nouveau:
//reagit au sifflet et arrete le robot quand il entend le sifflet
void spin(float v, double angle){
  //angle + distance +, angle - distance -
  double distance = angle_to_cm(angle, (distance_entre_les_roues - (0.005 * 12) ) / -2.); 
  unsigned long newMillis;
  //spin a droite
  if(angle > 0){
    
    //si entend un sifflet a l'arret
    if(sifflet){
      entendSiffletSpin(0);
    }

    //timer pour ajustement
    lastMillis1 = millis();

    //valeurs mises a 0 pour ajustement
    resetAdjust();
    //commence a tourner
    MOTOR_SetSpeed(LEFT, v);
    MOTOR_SetSpeed(RIGHT, -v);

    //ajuste pendant les tours
    adjustSpin(v);
    
    //wait till master reaches the distance
    //since corrected same distance
    while(distance > clic_to_cm(ENCODER_Read(LEFT))){
      
      //si entend un sifflet en tournant a droite
      if(sifflet){
        newMillis = millis();

        //garde temps ecoule pour le timer 
        deltaT = newMillis - lastMillis1;

        entendSiffletSpin(v);

      }

      newMillis = millis();
      if(newMillis - lastMillis1 > DELAY ){
        adjustSpin(v);
        lastMillis1 = newMillis;
      }    
    }    
    MOTOR_SetSpeed(LEFT, 0);
    MOTOR_SetSpeed(RIGHT, 0);
  }
  //spin a gauche avec angle -
  else{

    //si entend un sifflet a l'arret
    if(sifflet){
      entendSiffletSpin(0);
    }

    //timer pour ajustement
    lastMillis1 = millis();

    //valeurs mises a 0 pour ajustement
    resetAdjust();
    //commence a tourner
    MOTOR_SetSpeed(LEFT, -v);
    MOTOR_SetSpeed(RIGHT, v);

    adjustSpin(-v);
    //wait till one reaches distance
    while(-distance > clic_to_cm(ENCODER_Read(RIGHT))){

      //si entend un sifflet pendant en tournant a gauche
      if(sifflet){
        newMillis = millis();

        //garde temps ecoule pour le timer 
        deltaT = newMillis - lastMillis1;

        entendSiffletSpin(-v);
      }

      newMillis = millis();
      if(newMillis - lastMillis1 > DELAY ){
        adjustSpin(-v);
        lastMillis1 = newMillis;
      }
    }
    MOTOR_SetSpeed(LEFT, 0);
    MOTOR_SetSpeed(RIGHT, 0);
  }

}

//code a executer si il entend un sifflet
void entendSifflet(float v, float ratio){
  
  //sauvegarde le temps actuel
  unsigned long newMillis;
  
  //commence le timer de 10 secondes
  lastMillis2 = millis();
  
  //va arreter le robot le plus vite possible et attendre 10 secondes avant de le reaccelerer
  if (v < 0)
  {
    
    //nombre positif a ajouter
    float n = (0 - v) / ITERATIONSIFFLET;//50 pour 1 seconde
    // ici le n est diviser par 10. pour qu'il se rende à la vitesse finale en 10 loop
    // si j'avais mis un n comme 0.05 ou qq chose comme ça, vu que les vitesses changent 
    // tout le temps, le n se serait jamais rendu pile sur la vitesse souhaitée.
    
    //timer utilise pour les accelerations/decelerations
    lastMillis1 = millis();

    for (float i = v; i <= 0;)
    {
      newMillis = millis();
      
      if(newMillis - lastMillis1 > DELAY ){
        //decelere avec adjustement
        slaveAdjust(i, ratio);
        lastMillis1 = newMillis;
        i+=n;
      }
    }
    
    //attends 10 secondes
    while(10000 > millis() - lastMillis2){}

    //restart timer for acceleration
    lastMillis1 = millis();

    for (float i = 0; i >= v;)
    {
      newMillis = millis();
      
      if(newMillis - lastMillis1 > DELAY ){
        //decelere avec adjustement
        slaveAdjust(i, ratio);
        lastMillis1 = newMillis;
        i-=n;
      }
    }

    //reset le temps a la valeur avant d'entendre le sifflet
    lastMillis1 = millis() - deltaT;

  }
  else if (v > 0)
  {
    //nombre negatif a enlever
    float n = (0 - v) / ITERATIONSIFFLET;

    //timer utilise pour les accelerations/decelerations
    lastMillis1 = millis();

    for (float i = v; i >= 0;)
    {
      newMillis = millis();
      if(newMillis - lastMillis1 > DELAY ){
        //decelere avec ajustement
        slaveAdjust(i, ratio);
        lastMillis1 = newMillis;
        i+=n;
      }
    }

    //attends 10 secondes
    while(10000 > millis() - lastMillis2){}

    //timer utilise pour les accelerations/decelerations
    lastMillis1 = millis();

    for (float i = 0; i <= v;)
    {
      newMillis = millis();
      if(newMillis - lastMillis1 > DELAY ){
        //decelere avec ajustement
        slaveAdjust(i, ratio);
        lastMillis1 = newMillis;
        i-=n;
      }
    }

    //reset le temps a la valeur avant d'entendre le sifflet
    lastMillis1 = millis() - deltaT;

  }
  //si a l'arret lors du coup de sifflet
  else{
    //arrete le robot si il est en mouvement
    MOTOR_SetSpeed(LEFT, 0.);
    MOTOR_SetSpeed(RIGHT, 0.);
    //attends 10 secondes
    while(10000 > millis() - lastMillis2){}

    //reset le temps a la valeur avant d'entendre le sifflet
    lastMillis1 = millis() - deltaT;

  }

}

//cod.e a executer si il entend un sifflet pendant qu'il spin
void entendSiffletSpin(float v){

  //sauvegarde le temps actuel
  unsigned long newMillis;
  
  //commence le timer de 10 secondes
  lastMillis2 = millis();

  //si entend un sifflet en tournant a droite
  if(v > 0){
    
    //commence timer de 10 secondes
    lastMillis2 = newMillis;
    
    //arrete le robot
    MOTOR_SetSpeed(LEFT, 0.);
    MOTOR_SetSpeed(RIGHT, 0.);

    //attend 10 secondes
    while(10000 > millis() - lastMillis2){}
    
    //reset le timer comme avant le sifflet
    lastMillis1 = millis() - deltaT;

    //remet le robot en marche
    adjustSpin(v);
    sifflet = false;
  }

  //si entend un sifflet pendant en tournant a gauche
  else if(v < 0){
    
    //commence timer de 10 secondes
    lastMillis2 = newMillis;
    
    //arrete le robot
    MOTOR_SetSpeed(LEFT, 0.);
    MOTOR_SetSpeed(RIGHT, 0.);

    //attend 10 secondes
    while(10000 > millis() - lastMillis2){}
    
    //reset le timer comme avant le sifflet
    lastMillis1 = millis() - deltaT;

    //remet le robot en marche
    adjustSpin(-v);
    sifflet = false;
  }

  //si entend un sifflet a l'arret
  else{
    //arrete le robot
    MOTOR_SetSpeed(LEFT, 0.);
    MOTOR_SetSpeed(RIGHT, 0.);
    
    //commence timer de 10 secondes
    lastMillis2 = millis();

    //attend 10 secondes
    while(10000 > millis() - lastMillis2){}
    
    sifflet = false;
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
    
    tourner(0.6, 0.6, 20, -3.0, 90);
    
    tourner(0.6, 0.6, 20, 18, 180);

    tourner(0.6, 0.6, 20, -3.0, 52);

    avancer(0, 6, 0.6, 0.7);
    
    avancer(40, 5, 0.7, 0.7);

    avancer(0, 6, 0.7, 0.4);
    
    tourner(0.4, 0.4, 20, -3.0, 68);

    avancer(0, 12, 0.4, 0.7);
    
    avancer(15, 5, 0.7, 0.7);

    avancer(0, 6, 0.7, 0.6);
    
    tourner(0.6, 0.6, 20, 12, 42);

    avancer(0, 6, 0.6, 0.7);

    avancer(18, 5, 0.7, 0.7);

    avancer(0, 6, 0.7, 0.6);

    tourner(0.6, 0.6, 20, 12, 12);

    avancer(0, 6, 0.6, 0.7);

    avancer(58, 5, 0.7, 0.7);

    avancer(0, 15, 0.7, 0);
    
    // FIN ALLÉ
    delay(200);
    
    avancer(-55, 20, 0, -0.7); // Premiere ligne retour

    avancer(0, 10, -0.7, -0.4); // Deceleration

    tourner(-0.4, -0.4, 20, 12.0, 22); // Premier tournant retour

    avancer(0, 10, -0.4, -0.7); // accel ligne 2
    
    avancer(-6, 5, -0.7, -0.7); // ligne 2

    avancer(0, 10, -0.7, -0.4); // decel ligne 2

    tourner(-0.4, -0.4, 20, 12.0, 53); // tournant pour ligne 3

    avancer(0, 10, -0.4, -0.7); // accel ligne 3

    avancer(-29.5, 10, -0.7, -0.7); // ligne 3

    avancer(0, 10, -0.7, -0.4); // decel ligne 3
 
    tourner(-0.4, -0.4, 20, -3.0, 95); // tournant 90

    avancer(0, 10, -0.4, -0.7); // accel ligne 4

    avancer(-1.5, 5, -0.7, -0.7); // ligne 4

    avancer(0, 10, -0.7, -0.4); // decel ligne 4

    tourner(-0.4, -0.4, 20, 3.0, 62); // tournant pour ligne 5

    tourner(-0.4, -0.4, 20, -8.0, 92); // tournant pour ligne 6

    tourner(-0.4, -0.4, 20, 10.0, 90); //tournant pour U turn 1/2

    tourner(-0.4, -0.4, 20, 10.0, 102); //tournant pour U turn 1/2

    tourner(-0.4, -0.4, 20, -28.0, 78); //tournant vers ligne finale

    avancer(-190, 12, -0.4, -0.9); //accel final stretch

    avancer(0, 10, -0.9, 0); //decelleration du champion


  }
  //odd enough but since they are over by the same amount they balance out and it goes back to initial position hehe
  //surely means that by adding a slightly smaller angle than actually desired angle can be reached
  if(ROBUS_IsBumper(LEFT)){
    // resetAdjust();
    // delay(500);
    // avancer(0, 50 , 0, 0.4);
    // avancer(200, 0, 0.4, 0.4);

    // //rayon + a gauche de reculons
    // tourner(-0.6, -0.6, 20, 3., 180); //(ok)
    // avancer(0, 0, -0.6, 0);
    // delay(500);
    // tourner(-0.6, -0.6, 20, 3., 90); //(over)
    // avancer(0, 0, -0.6, 0);
    // delay(500);
    // tourner(-0.6, -0.6, 20, 3., 90); //(over)
    // avancer(0, 0, -0.6, 0);
    // delay(500);
    // //rayon - a droite de reculons
    // tourner(-0.6, -0.6, 20, -3., 180); //(over)
    // avancer(0, 0, -0.6, 0);
    // delay(500);
    // tourner(-0.6, -0.6, 20, -3., 90); //(over)
    // avancer(0, 0, -0.6, 0);
    // delay(500);
    // tourner(-0.6, -0.6, 20, -3., 90); //(over)
    // avancer(0, 0, -0.6, 0);
    // delay(500);

    spin(0.4, 90);
    delay(1000);

    spin(0.4, -90);
    delay(1000);

    spin(0.4, 180);
    delay(1000);

    spin(0.4, -180);
    delay(1000);

    spin(0.4, 270);
    delay(1000);

    spin(0.4, -270);
    delay(1000);
    
    delay(1500);

    spin(0.2, 90);
    delay(1000);

    spin(0.2, -90);
    delay(1000);

    spin(0.2, 180);
    delay(1000);

    spin(0.2, -180);
    delay(1000);

    spin(0.2, 270);
    delay(1000);

    spin(0.2, -270);
    delay(1000);

    delay(1500);

    spin(0.8, 90);
    delay(1000);

    spin(0.8, -90);
    delay(1000);

    spin(0.8, 180);
    delay(1000);

    spin(0.8, -180);
    delay(1000);

    spin(0.8, 270);
    delay(1000);

    spin(0.8, -270);
    delay(1000);
  }

  if(ROBUS_IsBumper(RIGHT)){

    tourner(0.4, 0.4, 20, 10., 90);
    avancer(0., 0, 0.4, 0);
    delay(500);

    tourner(0.4, 0.4, 20, -10., 90);
    avancer(0., 0, 0.1, 0);
    delay(500);

    tourner(-0.4, -0.4, 20, 10., 90);
    avancer(0., 0, -0.4, 0);
    delay(500);

    tourner(-0.4, -0.4, 20, -10., 90);
    avancer(0., 0, -0.1, 0);
    delay(500);

    tourner(0.4, 0.4, 20, 10., 180);
    avancer(0., 0, 0.4, 0);
    delay(500);

    tourner(0.4, 0.4, 20, -10., 180);
    avancer(0., 0, 0.1, 0);
    delay(500);

    tourner(-0.4, -0.4, 20, 10., 180);
    avancer(0., 0, -0.4, 0);
    delay(500);

    tourner(-0.4, -0.4, 20, -10., 180);
    avancer(0., 0, -0.1, 0);
    delay(500);

    
    tourner(0.4, 0.4, 20, 10., 90);
    avancer(0., 0, 0.4, 0);
    delay(500);

    tourner(0.4, 0.4, 20, -10., 90);
    avancer(0., 0, 0.1, 0);
    delay(500);

    tourner(-0.4, -0.4, 20, 10., 90);
    avancer(0., 0, -0.4, 0);
    delay(500);

    tourner(-0.4, -0.4, 20, -10., 90);
    avancer(0., 0, -0.1, 0);
    delay(500);

    tourner(0.4, 0.4, 20, 10., 180);
    avancer(0., 0, 0.4, 0);
    delay(500);

    tourner(0.4, 0.4, 20, -10., 180);
    avancer(0., 0, 0.1, 0);
    delay(500);

    tourner(-0.4, -0.4, 20, 10., 180);
    avancer(0., 0, -0.4, 0);
    delay(500);
    
    tourner(-0.4, -0.4, 20, -10., 180);
    avancer(0., 0, -0.1, 0);
    delay(500);

    delay(1500);

    avancer(100., 100, 0., 0.95);
    avancer(-200., 100, 0.95, -0.95);
    avancer(100., 100, -0.95, 0.95);
    avancer(4000., 100, 0.95, 0.);
  }
  if(ROBUS_IsBumper(FRONT)){ 
    avancer(180, 50, 0, 0.9999);  
    avancer(420, 50, 0.9999, 0);
  }
}

