#include <math.h> 
#include <Kinematics.h>
Kinematics k(12,12);



double l = 5.5;  //distance from chess width to base center (cm)
double L = 30.25; //chess length (cm)
double l1 = 12; //lower arm
double l2 = 12; //upper arm
double l3 = 12; //60 degrees gripper angle

double z = -12.0; //height of lower arm joint from chess board
double pi = 3.141592653589793238;
double phi = 3*pi/2; //end-effector angle


//dynamic values:
double polar[2]; //(r,alpha)
double th[4];

#include <Servo.h>

Servo servoArray[5];
Servo s6;
String servos[5]={"base", "lower_arm", "upper_arm", "gripper_arm","gripper"};//descendingorder

bool done=false;
int delay1=30;
int currPosArray[5];
int prevPosArray[5];


int coor[2];
void setup() {
  Serial.begin(9600);
  servoArray[0].attach(3);
  servoArray[0].write(90);
  servoArray[1].attach(5);
  servoArray[1].write(90);
  servoArray[2].attach(6);
  servoArray[2].write(180);
  servoArray[3].attach(9);
  servoArray[3].write(0);

  servoArray[4].attach(10); 
  servoArray[4].write(10);
  s6.attach(11); //extra unused 
  s6.write(0);
  
//  reset_1();
}



void loop() {
 
  if(not done){
    get_piece();
    move_piece();
  
  done = true;
  }
 
}




void get_piece() {
  coor[0] = -1; coor[1] = -1;
  get_kinematic_coordinates(coor); //updates polar array

  k.moveToPosition(polar[0], polar[1], -2);
  Angle a = k.getAngles();

  th[0] = 180.0 + a.theta3 ; //base s[0]
  th[1] = 90.0 - a.theta1; //lower s[1]
  th[2] = 90 + a.theta2; //upper s[2]
  th[3] = 90- a.theta1- a.theta2; //gripper arm s[3]
  
  gripperEnable();

  for (int i=3; i>=0; i--) {
    currPosArray[i]=th[i];
    setToAngle(servoArray[i],servoArray[i].read(),currPosArray[i],delay1);
    prevPosArray[i] = currPosArray[i];
    delay(1000);//takes a second before it moves next joint  
  }
}

void move_piece() {
  coor[0] = -3; coor[1] = -2;
  get_kinematic_coordinates(coor); //updates polar array

  k.moveToPosition(polar[0], polar[1], -2);
  Angle a = k.getAngles();

  th[0] = 180.0 + a.theta3 ; //base
  th[1] = 90.0 - a.theta1; //lower
  th[2] = 90 + a.theta2; //upper
  th[3] = 0; //unused
  th[4] = 90- a.theta1- a.theta2; //gripper arm
  

  for (int i=3; i>=0; i--) {
    currPosArray[i]=th[i];
    setToAngle(servoArray[i],servoArray[i].read(),currPosArray[i],delay1);
    prevPosArray[i] = currPosArray[i];
    delay(1000);//takes a second before it moves next joint  
  }
  gripperDisable();
  
}


void reset_1(){
  s6.write(0);   //unused
  setToAngle(servoArray[4], servoArray[4].read(), 10, delay1); //gripper
  setToAngle(servoArray[3], servoArray[3].read(), 0, delay1); //gripper arm
  setToAngle(servoArray[2], servoArray[2].read(), 180, delay1); //upper arm
  setToAngle(servoArray[1], servoArray[1].read(), 90, delay1); //lower arm
  setToAngle(servoArray[0], servoArray[0].read(), 90, delay1); 
  
   
  
   
  prevPosArray[0]=90;
  prevPosArray[1]=90;
  prevPosArray[2]=180;
  prevPosArray[3]=0;
  prevPosArray[4]=10; 
}




void gripperEnable(){
  setToAngle(servoArray[4], servoArray[4].read(), 70, delay1);
}

void gripperDisable(){
  setToAngle(servoArray[4], servoArray[4].read(), 0, delay1);
}


void setToAngle(Servo servoNum, int angle_init, int angle, int delay1){
  if(angle_init < angle){
    for(int i = angle_init; i <= angle; i++){
      servoNum.write(i);
      delay(delay1);
    }
   } else {
    for(int i = angle_init; i >= angle; i--){
      servoNum.write(i);
      delay(delay1);
    }
   }
}






 
void get_kinematic_coordinates(int pos[2]) {
  double x = (double) pos[0];
  double y = (double) pos[1];

  double x1;
  if (x > 0) {x1 = x - 0.5;}
  else {x1 = x + 0.5;}

  double y1;
  if (y > 0) {y1 = y - 0.5; }
  else {y1 = y + 0.5;}


  double x_cm = x1*L/8;
  double y_cm = y1*L/8 - (L/2 + l);
 
//  double r = sqrt(sq(x_cm) + sq(y_cm));
//  double alpha = atan2 (y_cm, x_cm);
  polar[0] = x_cm;
  polar[1] = y_cm;
}
