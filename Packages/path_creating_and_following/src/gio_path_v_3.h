#ifndef __GIO_PATH_H_
#define __GIO_PATH_H_

#include <math.h>
#include "curves_v_3.h"

#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>

using namespace std;

#define d_y_0               0.0002         //ab wann y als 0 wahrgenommen werden kann
#define d_th_0              0.001          //ab wann th als 0 wahrgenommen werden kann

#define AC_GRAD             0
#define AC_RAD              1

#define ABS                 1
#define RES                 0

#define GradToRadian        0.01745328
#define RadianToGrad       57.295827909

#define NormalizeAngle(ang) while(fabs(ang)>M_PI) ang+=(ang>0)?-2*M_PI:2*M_PI
#define SQR(c)  ((c) * (c))
/**
 * @class CGioController
 * @brief class encapsules path follower
 * @author Niko, Hartmut, Stefan
 * @see code fragment for using
 */
class CGioController{
protected:
  /**
   * ...
   */
  int loop_exit;
  //Lokales Koordinatensystem des Roboters
  double ex[2];
  double ey[2];

  double x0;
  double y0;
  double phi0;

  // festgestellte Konstanten vom Roboter
  double AXIS_LENGTH;
  double Vm;
  double d_y;
  double d_th;
  double kr_max;
  double u0;
  double a;
  double epsilon;

  void InitDefault();
  /**
   * Method to set local coordinate system
   * @param ang angle for rotating the coordinate system
   */
  void setLocalSystem(double ang);

  double H_case_1(double y, double th, double u, double alpha, double *gama);
  double H_case_2(double y, double th, double u, double alpha, double *gama);
  double Compute_W(double y, double th, double a, double u, int *err);

public:
  std::ofstream giofile;
  CCurve *path;

  CGioController();

  ~CGioController();

  void setAxisLength(double val);
  double getAxisLength();

  void setCurrentVelocity(double val, int abs = ABS);
  double getCurrentVelocity();

  void setPose(double x, double y, double phi);
  void getPose(double &x, double &y, double &ph);
  int getPathFromFile(const char* fname);
  int writePathToFile(const char* fname);
  int canDetermineRobotPosition(int looped = 0);
  int getNextState(double &u, double &w, double &vleft, double &vright, int looped = 0);
};

#endif
