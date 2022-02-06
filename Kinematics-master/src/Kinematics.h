#ifndef Kinematics_h
#define Kinematics_h

struct pos{
  float x;
  float y;
  float z;
};

struct angle{
  float theta1;
  float theta2;
  float theta3;
};

typedef struct pos Position;
typedef struct angle Angle;

class Kinematics{
  private:
    float l1, l2;
    Position currentPositions;
    Angle currentAngles;
    
    Position updatePositions(float x, float y, float z);
    Angle updateAngles(float theta1, float theta2, float theta3);

  public:
    Kinematics(int length1, int length2);

    void moveToAngle(float theta1, float theta2, float theta3);
    void moveToPosition(float x, float y, float z);
	
	void printPositions();
	void printAngles();
    
    Position getPositions();
    Angle getAngles();
};

#endif
