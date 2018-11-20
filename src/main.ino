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

/*//variables et constante pour ecoute sifflet
boolean check = false;
unsigned long timer = 0;
boolean sifflet = false;
//délai entre les deux checks du micro
const float DELAY2 = 240; //peut surement etre plus petit
//changer le treshold si des sons aléatoire sont entendus
int treshold = 385;
//pin output pour 5khz
int pin_5khz = 8;*/

//variables for servos and scanning
const int Horizontal_Angle = 72;
const int VERTICAL = 0;
const int HORIZONTAL = 1;
const int SENSORHEIGHT = 20;
const float Distance_from_sensor_to_pivot = 5.5;
const float sonarCorretionMultiplier = 1.32;
const float sonarCorrectionAdjust = -0.2799;
int smallestAngle;
float smallestDistance;
float Height;
float Wall1;
float Wall2;
float Wall3;
float Wall4;
float Length;
float Width;
float FloorArea;
float Wall1Area;
float Wall2Area;
float Wall3Area;
float Wall4Area;
float RoomVolume;

float distances[360];

/* ****************************************************************************
Vos propres fonctions sont creees ici
**************************************************************************** */

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
void ACC_MASTER(float vI, float vF, int nb_iterations, float ratio)
{
  //pas d'iterations, met les deux moteurs a la vitesse et adujste au besoin
  if(nb_iterations == 0){
    slaveAdjust(vF, ratio);
  }
  else{
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
        unsigned long newMillis = millis();
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
        unsigned long newMillis = millis();
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
void tourner(float vI, float vF, int iterations, float rayon, double angle){
  unsigned long newMillis;
  //cas avec vitesse negative
  if(vF < 0){
    //valeurs mises a 0 pour ajustement
    resetAdjust();

    //accelere/decelere jusqu'a vitesse finale
    ACC_MASTER(vI, vF, iterations, ratio_de_virage(rayon));
    lastMillis1 = millis();
    //distance si tourne a droite
    if(rayon < 0){
      while(angle_to_cm(angle, -rayon) > -clic_to_cm(ENCODER_Read(RIGHT)) ){
        newMillis = millis();
        if(newMillis - lastMillis1 > DELAY ){
          slaveAdjust(vF, ratio_de_virage(rayon));
          lastMillis1 = newMillis;
        }
        //applique l'ajustement a faire pour faire tourner les roues pour qu'elles parcourent les bonnes distances
        // slaveAdjust(vF, ratio_de_virage(rayon));
      }
    }
    //distance si tourne a gauche
    else{
      while(angle_to_cm(angle, rayon) > -clic_to_cm(ENCODER_Read(LEFT)) ){
        newMillis = millis();
        if(newMillis - lastMillis1 > DELAY ){
          slaveAdjust(vF, ratio_de_virage(rayon));
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
    ACC_MASTER(vI, vF, iterations, ratio_de_virage(rayon));

    // //applique l'ajustement afin de tourner dans le bon sens
    // slaveAdjust(v, ratio_de_virage(rayon));
    
    //distance si tourne a gauche
    if(rayon < 0){
      while(angle_to_cm(angle, -rayon) > clic_to_cm(ENCODER_Read(RIGHT)) ){
        newMillis = millis();
        if(newMillis - lastMillis1 > DELAY ){
          slaveAdjust(vF, ratio_de_virage(rayon));
          lastMillis1 = newMillis;
        }
        // slaveAdjust(vF, ratio_de_virage(rayon));
      }
    }
    //distance si tourne a droite
    else{
      while(angle_to_cm(angle, rayon) > clic_to_cm(ENCODER_Read(LEFT))){
        newMillis = millis();
        if(newMillis - lastMillis1 > DELAY ){
          slaveAdjust(vF, ratio_de_virage(rayon));
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
void spin(float v, double angle){
  //angle + distance +, angle - distance -
  double distance = angle_to_cm(angle, (distance_entre_les_roues - (0.005 * 12) ) / -2.); 
  lastMillis1 = millis();
  unsigned long newMillis;
  //spin a droite
  if(angle > 0){
    //valeurs mises a 0 pour ajustement
    resetAdjust();
    
    //ajuste pendant les tours
    adjustSpin(v);
    
    //wait till master reaches the distance
    //since corrected same distance
    while(distance > clic_to_cm(ENCODER_Read(LEFT))){
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
    //valeurs mises a 0 pour ajustement
    resetAdjust();

    adjustSpin(-v);
    //wait till one reaches distance
    while(-distance > clic_to_cm(ENCODER_Read(RIGHT))){
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

/*void ecouteSifflet(){
  //check temps actuel
  unsigned long newMillis = millis();
  
  //check delay
  if((newMillis - timer) >= DELAY2){
    //update le timer si delay passe
    timer = newMillis;
  
    //check once 
    if(!check && analogRead(pin_5khz) > treshold){
      Serial.println("1 triggered at ");
      Serial.println(analogRead(pin_5khz));
      Serial.println("!");
      Serial.println();
      check = true;
    }
    //check again
    else if(check && analogRead(pin_5khz) > treshold){
      Serial.println("2 triggered at ");
      Serial.println(analogRead(pin_5khz));
      Serial.println("!");
      sifflet = true;
      check=false;
    }
    //if the checks fail either random noise or no whistle
    else{
      check=false;
      sifflet = false;
    }

  }
}*/

float sonarCorrection(){
  return SONAR_GetRange(1)*sonarCorretionMultiplier + sonarCorrectionAdjust;
}

void MinimalValue(int startAngle, int endAngle, int step){
  smallestDistance = 5000000000;
  for (int i = startAngle+step; i <= endAngle ; i+=step){
    if(distances[i]==(Distance_from_sensor_to_pivot+sonarCorrectionAdjust)){
      Serial.print("Reading error at ");
      Serial.println(i);
    }
    else if(distances[i] < smallestDistance){// dans cette fonction if : && distances[i] != 0
      smallestDistance = distances[i];
      smallestAngle = i;
    }
  }
}


//Scanning function
//scan version 0.1
//valeur de angle entre 0 et 359
//bond = nombre de degrees par mesure
void DistanceScan(int startAngle, int endAngle, int step){
    SERVO_SetAngle(VERTICAL, Horizontal_Angle);
    int Scan_CurrentAngle = startAngle;
    Serial.println("DÉBUT DU SCAN");
    
    if(startAngle >= 180){
        spin(0.2, 180);
    }

    while (Scan_CurrentAngle <= endAngle){
      Serial.print(Scan_CurrentAngle);
      Serial.print(": ");
      if(Scan_CurrentAngle <= 179){
        SERVO_SetAngle(HORIZONTAL, Scan_CurrentAngle);
        distances[Scan_CurrentAngle] = sonarCorrection() + Distance_from_sensor_to_pivot;
        Serial.println(distances[Scan_CurrentAngle]);
        Scan_CurrentAngle += step;
        delay(100);
      }
      else{
        if(Scan_CurrentAngle <= 180 && Scan_CurrentAngle <= (180 + step) && !(startAngle >= 180)){
          spin(0.2, 180);
        }
        SERVO_SetAngle(HORIZONTAL, Scan_CurrentAngle - 180);
        distances[Scan_CurrentAngle] = sonarCorrection() + Distance_from_sensor_to_pivot;
        Serial.println(distances[Scan_CurrentAngle]);
        Scan_CurrentAngle += step;
        delay(100);
      }
    }
    
    MinimalValue(startAngle,endAngle,step);
    //DistanceFromWalls();
    //RoomSize();
    //HeightScan();
}

/*void print_ScannedDistances(int data[]){
	for (int i=0; i<= 179; i++){
    Serial.println(data[i]);
	}
}*/


void DistanceFromWalls(){ //distances are from the pivoting point of the servos
  Wall1 = distances[smallestAngle];
  if (smallestAngle + 90 < 359){
  Wall2 = distances[smallestAngle+90];
  }
  else {
    Wall2 = distances[smallestAngle-270];
  }

  if (smallestAngle + 180 < 359){
    Wall3 = distances[smallestAngle+180];
  }
  else {
    Wall3 = distances[smallestAngle-180];
  }
  
  if (smallestAngle + 270 < 359){
    Wall4 = distances[smallestAngle+270];
  }
  else {
    Wall4 = distances[smallestAngle-90];
  }
 //code peut etre rafiner pour s'assurer d'être à 90 degrées
  // Serial.println(Wall1);
  // Serial.println(Wall2);
  // Serial.println(Wall3);
  // Serial.println(Wall4);
}

void HeightScan(){
  delay(500);
  SERVO_SetAngle(VERTICAL,Horizontal_Angle+90);
  delay(400);
  Height = sonarCorrection() + SENSORHEIGHT;
  delay(200);
  SERVO_SetAngle(VERTICAL, Horizontal_Angle);
}

void CenterRobot()
{
  spin(0.1,smallestAngle);
  float distance_y = ((Wall4 - Wall2)/2);
  float distance_x = ((Wall1 - Wall3)/2);
  if(distance_y >= 0)
  {
    avancer(distance_y, 0, 0.3, 0.3);
    spin(0.3, 90);
    if(distance_x >= 0)
    {
      avancer(distance_x, 0, 0.3, 0.3);
    }
    if(distance_x < 0)
    {
      avancer(distance_x, 0, -0.3, -0.3);
    }
  }
  if(distance_y < 0)
  {
    avancer(distance_y, 0, -0.3, -0.3);
    spin(0.3, 90);
    if(distance_x >= 0)
    {
      avancer(distance_x, 0, 0.3, 0.3);
    }
    if(distance_x < 0)
    {
      avancer(distance_x, 0, -0.3, -0.3);
    }
  }
}
void RoomSize(){
  Length = Wall1 + Wall3;
  Width = Wall2 + Wall4;
  FloorArea = Length*Width;
  Wall1Area = Width * Height;
  Wall2Area = Length * Height;
  Wall3Area = Wall1Area;
  Wall4Area = Wall2Area;
  RoomVolume = FloorArea * Height;
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
  SERVO_Enable(0);
  SERVO_Enable(1);
  SERVO_SetAngle(VERTICAL, Horizontal_Angle);
  SERVO_SetAngle(HORIZONTAL,0);
  
}


/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop() {
  // SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  delay(10);// Delais pour décharger le CPU
  MOTOR_SetSpeed(LEFT, 0);
  MOTOR_SetSpeed(RIGHT, 0);

  if(ROBUS_IsBumper(FRONT)){
    /*Serial.println(sonarCorrection());
    delay(100); //minimium delay according to documentation
    */
   CenterRobot();
  }

  if(ROBUS_IsBumper(REAR)){
    DistanceScan(0, 359, 2);
    HeightScan();
    DistanceFromWalls();
    RoomSize();
    Serial.print("Height : ");
    Serial.println(Height);
    Serial.print("Width : ");
    Serial.println(Width);
    Serial.print("Length : ");
    Serial.println(Length);
    Serial.print("Room Volume : ");
    Serial.println(RoomVolume);
    Serial.print("Floor Area : ");
    Serial.println(FloorArea);
    Serial.print("Wall1 : ");
    Serial.println(Wall1);
    Serial.print("Wall2 : ");
    Serial.println(Wall2);
    Serial.print("Wall3 : ");
    Serial.println(Wall3);
    Serial.print("Wall4 : ");
    Serial.println(Wall4);
    Serial.print("smallestAngle : ");
    Serial.println(smallestAngle);
  }

  if(ROBUS_IsBumper(RIGHT)){
    HeightScan();
    DistanceFromWalls();
    RoomSize();
    Serial.print("Height : ");
    Serial.println(Height);
    Serial.print("Width : ");
    Serial.println(Width);
    Serial.print("Length : ");
    Serial.println(Length);
    Serial.print("Room Volume : ");
    Serial.println(RoomVolume);
    Serial.print("Floor Area : ");
    Serial.println(FloorArea);
    Serial.print("Wall1 : ");
    Serial.println(Wall1);
    Serial.print("Wall2 : ");
    Serial.println(Wall2);
    Serial.print("Wall3 : ");
    Serial.println(Wall3);
    Serial.print("Wall4 : ");
    Serial.println(Wall4);
    Serial.print("smallestAngle : ");
    Serial.println(smallestAngle);
  }

  if(ROBUS_IsBumper(LEFT)){
    SERVO_SetAngle(VERTICAL,Horizontal_Angle);
    SERVO_SetAngle(HORIZONTAL,180);
  }
}