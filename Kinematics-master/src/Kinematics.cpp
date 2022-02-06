#if ARDUINO >= 100
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif

#include "Kinematics.h"

Kinematics::Kinematics(int length1, int length2){
  Serial.begin(9600);
  l1 = length1;
  l2 = length2;

  currentAngles.theta1 = 90;
  currentAngles.theta2 = -90;
  currentAngles.theta3 = 0;

  currentPositions.x = length2;
  currentPositions.y = 0;
  currentPositions.z = length1;
}

Position Kinematics::updatePositions(float x, float y, float z){
  Position newPositions;
  newPositions.x = x;
  newPositions.y = y;
  newPositions.z = z;
  return newPositions;
}

Angle Kinematics::updateAngles(float theta1, float theta2, float theta3){
  Angle newAngles;
  newAngles.theta1 = theta1;
  newAngles.theta2 = theta2;
  newAngles.theta3 = theta3;
  return newAngles;
}

void Kinematics::moveToAngle(float theta1, float theta2, float theta3){
  float xPrime = l1 * cos(radians(theta1)) + l2 * cos(radians(theta1) + radians(theta2));
  float z = l1 * sin(radians(theta1)) + l2 * sin(radians(theta1) + radians(theta2));
  float x = xPrime * cos(radians(theta3));
  float y = xPrime * sin(radians(theta3));

  currentAngles = updateAngles(theta1, theta2, theta3);
  currentPositions = updatePositions(x, y, z);
}

void Kinematics::moveToPosition(float x, float y, float z){
  float xPrime = sqrt(pow(x, 2) + pow(y, 2));
  
  float theta3 = degrees(atan2(y/x));
  float theta2 = degrees(acos((pow(xPrime, 2) + pow(z, 2) - pow(l1, 2) - pow(l2, 2))/(2 * l1 * l2)));
  float theta1 = degrees(atan2(xPrime, z) - atan2((l2 * sin(radians(theta2))), (l1 + l2 * cos(radians(theta2)))));
  
  currentAngles = updateAngles(theta1, theta2, theta3);
  currentPositions = updatePositions(x, y, z);
}

void Kinematics::printPositions(){
  Serial.print("positions:");
  Serial.print("\t");
  Serial.print(currentPositions.x);
  Serial.print("\t");
  Serial.print(currentPositions.y);
  Serial.print("\t");
  Serial.print(currentPositions.z);
  Serial.println("\t");
}

void Kinematics::printAngles(){
  Serial.print("Angles:");
  Serial.print("\t");
  Serial.print("\t");
  Serial.print(currentAngles.theta1);
  Serial.print("\t");
  Serial.print(currentAngles.theta2);
  Serial.print("\t");
  Serial.print(currentAngles.theta3);
  Serial.println("\t");
}

Position Kinematics::getPositions(){
  return currentPositions;
}

Angle Kinematics::getAngles(){
  return currentAngles;
}
