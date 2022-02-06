#include <math.h> 
#include <Servo.h>

double l = 5.5;  //distance from chess width to base center (cm)
double L = 30.25; //chess length (cm)
double l1 = 12.0; //lower arm
double l2 = 12.0; //upper arm
double l3 = 12.0; //60 degrees gripper angle

double z = -12.0; //height of lower arm joint from chess board
double pi = 3.141592653589793238;
double phi = 3*pi/2; //end-effector angle


//dynamic values:
double polar[2]; //(r,alpha)
double th[5];


Servo servoArray[5];
Servo s6;
String servos[5]={"base", "lower_arm", "upper_arm", "gripper_arm","gripper"};//descendingorder

int coor[2];
int delay1=20;
int currPosArray[5];
int prevPosArray[5];
void serial_setup() {
  Serial.begin(115200);
  //to be called in setup
  // put your setup code here, to run once:

}

void loop_serial() {
  //to be called in loop
  if (Serial.available()){
    int data=Serial.read();
    int y=data%16;
    int x=int((data-y)/16);
    x=x-4;
    y=y-4;
     coor[0] = x; coor[1] =y;
  }

}


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

  gripper_open();
  gripper_close();
 
  
//  reset_1();
}






void loop() {
  int coor[2]={-1,-1}; 
  get_kinematic_coordinates(coor); 
  get_angle_arr(polar[0], polar[1]);
  delay(6000);
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
 
  double r = sqrt(sq(x_cm) + sq(y_cm));
  double alpha = atan2 (y_cm, x_cm);
  polar[0] = r;
  polar[1] = alpha;
  Serial.println(r); Serial.println(alpha); 
}



void get_angle_arr(double r, double alpha) {
  double th0 = 90 + alpha; //base angle
   
  //inverse Kinematics:
  double th1 = atan2(z,r) - acos((sq(l1) + sq(r) + sq(z) - sq(l2))/(2*l*sqrt(sq(r)+sq(z))));
  double th2 = pi - acos((sq(l1) - sq(r) - sq(z) + sq(l2))/ (2*l1*l2)); 
  double th3 = phi - th1 - th2;  

  th[0] = th0*180/pi; 
  th[1] = 90 + th1*180/pi ; 
  th[2] = 90 - (th2)*180/pi; 
  th[3] = th3*180/pi; 
  th[4] = 0.0;
}







void gripper_open(){
  setToAngle(servoArray[4], prevPosArray[0], 70, delay1);
}

void gripper_close(){
  setToAngle(servoArray[4], prevPosArray[0], 0, delay1);
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
