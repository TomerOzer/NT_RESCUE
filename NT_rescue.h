#ifndef NT_RESCUE_H
#define NT_RESCUE_H

class NT_rescue {
public:
  NT_rescue(int _m1a, int _m1b, int _m1e, int _m2a, int _m2b, int _m2e, int _m3a, int _m3b, int _m3e, int _m4a, int _m4b, int _m4e);
  void calibrateMPU();
  int getYaw();
  void begin();
  void moveF(int speed);
  void moveB(int speed);
  void L(int speed);
  void R(int speed);
  void move(int degree, int speed);
  void turnR(int speed);
  void turnL(int speed);
  void StupGyro();
  void printYaw();
  void resetYaw();
  void LcdCount();
  double calc(double sp, double pv, float kp, float ki, float kd);
  double integral;
  void Of(int speed);
  void Ob(int speed);
  void Tf(int speed);
  void Tb(int speed);
  void THf(int speed);
  void THb(int speed);
  void Ff(int speed);
  void Fb(int speed);
  void DbR(int speed);
  void DfR(int speed);
  void DbL(int speed);
  void DfL(int speed);
  void setLCD();
  void LcdYaw();
  void turn(int degree, int speed);
  void stopMotors();
  void beginHusky();
  int FindLine();
  void movein(int degree, int speed);
  void FollowLine(int speed, int, int color, float kp, float ki, float kd);
  double P_calc(double sp, double pv, float kp);
  void check_Ku(int speed, int degree, int Ku);
private:
  int m1a, m1b, m1e, m2a, m2b, m2e, m3a, m3b, m3e, m4a, m4b, m4e;
  int Kp;
  int Ki;
  int Kd;
  int xOrigin;
  int yOrigin;
  int xTarget;
  int yTarget;
  double kp, ki, kd, dtime, turnOut;
  double P, I, D, moveFOut, moveFSR, moveFSL, moveOut, turnSpeed, moveSpeed, moveinOut;
  double output, error, Time, prev_Time, deltaTime, prev_Error, derivative;

};

#endif

