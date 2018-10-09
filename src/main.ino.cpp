# 1 "c:\\users\\emile\\appdata\\local\\temp\\tmpo9fcju"
#include <Arduino.h>
# 1 "C:/Users/emile/OneDrive - USherbrooke/Baccalauréat Génie Robotique/S1/Projet/GRO-S1-almighty_Lenine/src/main.ino"
# 25 "C:/Users/emile/OneDrive - USherbrooke/Baccalauréat Génie Robotique/S1/Projet/GRO-S1-almighty_Lenine/src/main.ino"
#include <LibRobus.h>
# 45 "C:/Users/emile/OneDrive - USherbrooke/Baccalauréat Génie Robotique/S1/Projet/GRO-S1-almighty_Lenine/src/main.ino"
const float KP = 0.0001;

const float KI = 0.00002;



const float CYCLEDELAY = 0.1;

const int CLIC_PER_ROTATION = 3200;







typedef enum { MOTOR_LEFT, MOTOR_RIGHT } Motors;



int MOTOR_MASTER = 0;

int MOTOR_SLAVE = 1;
void Avancer(float speed, float distance);
int DistanceToClics(float distance);
void SetMaster(Motors ID);
void Turn(float angle);
void setup();
void loop();
#line 71 "C:/Users/emile/OneDrive - USherbrooke/Baccalauréat Génie Robotique/S1/Projet/GRO-S1-almighty_Lenine/src/main.ino"
void Avancer(float speed, float distance)

{

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



    speed_total = speed + (speed_cycle_error * KP) + (speed_total_error * KI);

    MOTOR_SetSpeed(MOTOR_SLAVE, speed_total);



    cycleNb++;

  }





  MOTOR_SetSpeed(MOTOR_MASTER, 0);

  MOTOR_SetSpeed(MOTOR_SLAVE, 0);

  ENCODER_Reset(MOTOR_MASTER);

  ENCODER_Reset(MOTOR_SLAVE);

}





int DistanceToClics(float distance)

{

  float w_radius = 3.8;

  float circonference = 2 * PI * w_radius;



  return (CLIC_PER_ROTATION * distance)/circonference;

}



void SetMaster(Motors ID)

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





      default: break;

   }

  int temp = MOTOR_MASTER;

  MOTOR_MASTER = MOTOR_SLAVE;

  MOTOR_SLAVE = temp;

}







void Turn(float angle)

{



  if(angle != 0)

  {

    if(angle < 0)

    {

      SetMaster(MOTOR_RIGHT);

    }



    if(angle > 0)

    {

      SetMaster(MOTOR_LEFT);

    }
# 275 "C:/Users/emile/OneDrive - USherbrooke/Baccalauréat Génie Robotique/S1/Projet/GRO-S1-almighty_Lenine/src/main.ino"
  }

}
# 295 "C:/Users/emile/OneDrive - USherbrooke/Baccalauréat Génie Robotique/S1/Projet/GRO-S1-almighty_Lenine/src/main.ino"
void setup(){

  BoardInit();

}
# 315 "C:/Users/emile/OneDrive - USherbrooke/Baccalauréat Génie Robotique/S1/Projet/GRO-S1-almighty_Lenine/src/main.ino"
void loop()

 {



  delay(10);







    Avancer(0.4, 1000);





}