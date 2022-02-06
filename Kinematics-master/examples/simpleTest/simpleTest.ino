#include <Kinematics.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  Kinematics k(10, 10);
  Position p;
  Angle a;

  p = k.getPositions();
  a = k.getAngles();

  // should get 10, 0, 10
  k.printPositions();

  // should get 90, -90, 0
  k.printAngles();

  k.moveToPosition(0, -10, 10);

  p = k.getPositions();
  a = k.getAngles();

  // should get 0, -10, 10
  k.printPositions();

  // should get 90, -90, -90
  k.printAngles();
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
