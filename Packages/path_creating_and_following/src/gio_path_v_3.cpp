#include "gio_path_v_3.h"

using namespace std;

//initialisierung des CGIO Controllers
void CGioController::InitDefault(){
  this->loop_exit    = 0;
  this->AXIS_LENGTH  = 0.485;
  this->Vm           = 1; 
  this->d_y          = 0.001;
  this->d_th         = 0.34; 
  this->kr_max       = 2/AXIS_LENGTH;
  this->u0           = Vm/2.0;
  this->a            = 1.2;             //alpha = 1.2 = P?
  this->epsilon      = 1e-4;            //I? = 0.0001
  this->path         = new CCurve();
  this->setPose(0.0, 0.0, 0);
  this->setLocalSystem(0.0);
}

void  CGioController::setLocalSystem(double ang){
  ex[0] = cos(ang); 
  ex[1] = sin(ang); 
  ey[0] = -sin(ang); 
  ey[1] = cos(ang); 
}

//Gl. 20
double  CGioController::H_case_1(double y, double theta, double u, double alpha, double *gamma){
  double h_j = SQR(kr_max) / (SQR(theta) * SQR(1+2*alpha));
  *gamma = 2*alpha*u*sqrt(h_j);
  return h_j;
};

//Gl. 21
double  CGioController::H_case_2(double y, double theta, double u, double alpha, double *gamma) {
  double h_j = 0.5 * ( - SQR(theta)/SQR(y) + sqrt( SQR(SQR(theta)/SQR(y)) + 4 * SQR(kr_max) / ( SQR(y) * SQR(1 + 2*alpha) ) ) );
  *gamma = 2*alpha*u*sqrt(h_j);
  return h_j;
};

//berechnen der winkelgeschwindigkeit des roboters
double  CGioController::Compute_W(double y, double theta, double a, double u, int *err){
  double res, h, gamma;

  if(fabs(y) < epsilon) {
    if(fabs(theta) >= d_th) {
     cerr << "H1_1\n";
	 h = H_case_1(y, theta, u, a, &gamma);
	 *err = 0;
    } else {
    cerr << "H1_2\n";
	 h = H_case_1(y, d_th, u, a, &gamma);
	 *err = 0;
    }
  } else {
    if(fabs(y) >= d_y) {
	 h = H_case_2(y, d_th, u, a, &gamma);
	 *err = 0;
    } else {
	 if(fabs(theta) < d_th) {
	   h = H_case_2(d_y, d_th, u, a, &gamma);
	   *err = 0;
	 } else {
	   h = H_case_2(d_y, theta, u, a, &gamma);
	   *err = 0;
	 }
    }
  }

  if(fabs(theta) >= d_th) {
    res = -h * u * y * sin(theta)/theta - gamma * theta;        //Gl. 28
  } else {
    if(fabs(theta) <= d_th && fabs(theta) >= d_th_0){
	 res = -h * u * y - gamma * theta;
    } else {
	 res = -h * u * y;
    }
  }

  return res;
}

//Constructor
CGioController::CGioController(){
  this->InitDefault();
  giofile.open("../test.dat");
  if(giofile.is_open()){
	  //printf("File Opened.");
  }
};

//Deconstructor
CGioController::~CGioController(){
  giofile.flush();
  giofile.close();
  giofile.clear();  
  delete path;
}

    //setzt die achsenlaenge
void CGioController::setAxisLength(double val) {
  if(fabs(val) > 0) {
    this->AXIS_LENGTH = fabs(val);
    this->kr_max = 2/AXIS_LENGTH;
  }
}
//gibt Achsenlaenge zurueck
double CGioController::getAxisLength() {
  return this->AXIS_LENGTH;
}

//uebergibt dem Roboter eine neue gewuenschte Geschwindigkeit
void CGioController::setCurrentVelocity(double val, int abs) {
  if(abs) {
    this->u0 = val;
  } else {
    this->u0 = (this->u0>val)?(val+this->u0):(this->u0+val);
  }
  this->Vm = this->u0 * 2.0;
}

//gibt die momentane geschwindigkeit zurueck
double CGioController::getCurrentVelocity() {
  return this->u0;
}

//setzt die momentane position auf die uebergeben werte
void CGioController::setPose(double x, double y, double phi) {
  this->x0 = x;
  this->y0 = y;
  this->phi0 = phi;
  NormalizeAngle(this->phi0);
}
//gibt die momentane position zurueck
void CGioController::getPose(double &x, double &y, double &ph) {
  x = this->x0;
  y = this->y0;
  ph = this->phi0;
}
//liest den Pfad aus einem File aus (uebergeben wird hier der name)
int CGioController::getPathFromFile(const char* fname){
  int res = path->LoadFromFile(fname);
	if(res==0){
	//cout << "getPathFromFile/(res): " << res << ", loading failed" << endl;
	}else{
	//cout << "getPathFromFile/(res): " << res << ", loading done" << endl;
	}
  if(res){
    path->initTraversal();

    this->loop_exit = path->getNext();
  }	
  return res;
}
//Gibt 0 zurück, falls keine Position errechnet werden kann, und 1, wenn doch.
int CGioController::canDetermineRobotPosition(int looped){
  int exit;

  exit = 0;
	if(!exit){
	}

  while(this->loop_exit != 0 && !exit) {
    if(path->pointInn(x0,y0)) {
	 if(path->getDistanceToEnd(x0, y0) > 0.1) { 
	   exit = 1;
	   giofile << path->getDistanceToEnd(x0, y0) << " "; // u 1
	   continue;
	 }
    }
    this->loop_exit = path->getNext(looped);
  }
  return exit;
}

//Berechnet den nächsten Zustand des Roboters (w, x, y)
int CGioController::getNextState(double &u, double &w, double &vleft, double &vright, int looped){
  double l, phic = 0, pathAng = 0;
  int err;
 
//wenn die Roboterposition nicht bestimmt werden kann setze alles auf null!
  if(!canDetermineRobotPosition(looped)) {
    u = 0;
    w = 0;
    vleft = 0;
	cout << "CGIOController/cannot determine pos" << endl;
    vright = 0;
    return 0;
  }
	
  //berechnet den abstand l zum vorgegeben pfad
  l = path->getDistance(x0,y0);
  if( path->Evaluate(x0,y0) > 5e-7 ) {
    l = -l;
  }
  
  giofile << l << " " << path->Evaluate(x0, y0) << " ";  

  phic = phi0 - path->getAng();
  giofile << phi0 << " " << pathAng << " " << phic << " ";
  //hier wird der winkel normalisiert
  NormalizeAngle(phic);

  u = this->u0;

  w = Compute_W(l, phic, this->a, u, &err);
  double sign = w < 0 ? -1 : 1;
  w = (fabs(w) > this->Vm / this->AXIS_LENGTH) ? sign*(this->Vm / this->AXIS_LENGTH) : w;

  giofile << w << " ";
  // berechnet die Geschwindigkeit fuer das linke und rechte rad
  vright = u - AXIS_LENGTH * w * 0.5;
  vleft  = u + AXIS_LENGTH * w * 0.5;
  return 1;
}
