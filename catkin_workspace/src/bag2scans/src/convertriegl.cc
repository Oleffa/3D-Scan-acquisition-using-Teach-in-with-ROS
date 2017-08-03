#include "slam6d/globals.icc"
#include "slam6d/point.h"
#include "riegl/scanlib.hpp"
#include "slam6d/point_type.h"
#include "slam6d/scan.h"

#include <vector>
using std::vector;
#include <string>
using std::string;
#include <fstream>
using std::ifstream;
using std::ofstream;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <iomanip>
using std::fixed;
using std::setprecision;
#include <string.h>
#define NR_SCANNERS 2


#if defined(_MSC_VER)
#   include <memory>
#else
#   include <tr1/memory>
#endif


using namespace scanlib;
//using namespace std;
using namespace std::tr1;

inline void myMMult(const double *M1, const double *M2,
         double *Mout)
{
  Mout[ 0] = M1[ 0]*M2[ 0]+M1[ 4]*M2[ 1]+M1[ 8]*M2[ 2]+M1[12]*M2[ 3];
  Mout[ 1] = M1[ 1]*M2[ 0]+M1[ 5]*M2[ 1]+M1[ 9]*M2[ 2]+M1[13]*M2[ 3];
  Mout[ 2] = M1[ 2]*M2[ 0]+M1[ 6]*M2[ 1]+M1[10]*M2[ 2]+M1[14]*M2[ 3];
  Mout[ 3] = M1[ 3]*M2[ 0]+M1[ 7]*M2[ 1]+M1[11]*M2[ 2]+M1[15]*M2[ 3];
  Mout[ 4] = M1[ 0]*M2[ 4]+M1[ 4]*M2[ 5]+M1[ 8]*M2[ 6]+M1[12]*M2[ 7];
  Mout[ 5] = M1[ 1]*M2[ 4]+M1[ 5]*M2[ 5]+M1[ 9]*M2[ 6]+M1[13]*M2[ 7];
  Mout[ 6] = M1[ 2]*M2[ 4]+M1[ 6]*M2[ 5]+M1[10]*M2[ 6]+M1[14]*M2[ 7];
  Mout[ 7] = M1[ 3]*M2[ 4]+M1[ 7]*M2[ 5]+M1[11]*M2[ 6]+M1[15]*M2[ 7];
  Mout[ 8] = M1[ 0]*M2[ 8]+M1[ 4]*M2[ 9]+M1[ 8]*M2[10]+M1[12]*M2[11];
  Mout[ 9] = M1[ 1]*M2[ 8]+M1[ 5]*M2[ 9]+M1[ 9]*M2[10]+M1[13]*M2[11];
  Mout[10] = M1[ 2]*M2[ 8]+M1[ 6]*M2[ 9]+M1[10]*M2[10]+M1[14]*M2[11];
  Mout[11] = M1[ 3]*M2[ 8]+M1[ 7]*M2[ 9]+M1[11]*M2[10]+M1[15]*M2[11];
  Mout[12] = M1[ 0]*M2[12]+M1[ 4]*M2[13]+M1[ 8]*M2[14]+M1[12]*M2[15];
  Mout[13] = M1[ 1]*M2[12]+M1[ 5]*M2[13]+M1[ 9]*M2[14]+M1[13]*M2[15];
  Mout[14] = M1[ 2]*M2[12]+M1[ 6]*M2[13]+M1[10]*M2[14]+M1[14]*M2[15];
  Mout[15] = M1[ 3]*M2[12]+M1[ 7]*M2[13]+M1[11]*M2[14]+M1[15]*M2[15];
}

void euler2matrix(double roll, double pitch, double yaw, double matrix[16]) {
  roll = rad(roll);
  yaw = rad(yaw);
  pitch = rad(pitch);

  double yawmat[16] = { cos(yaw),   sin(yaw), 0,0, 
                        -sin(yaw),  cos(yaw), 0,0, 
                        0, 0, 1, 0,
                        0, 0, 0, 1};
  
  double pitchmat[16] = { cos(pitch), 0, -sin(pitch), 0,
                          0, 1, 0, 0,
                          sin(pitch), 0, cos(pitch), 0,
                          0, 0, 0, 1};
  
  double rollmat[16] = {  1, 0, 0, 0,
                          0, cos(roll),   sin(roll), 0,
                          0, -sin(roll),  cos(roll), 0,
                          0, 0, 0, 1};


  double tmp[16];

  // Rz(yaw) * Ry(pitch) * Rx(roll)
  myMMult(yawmat, pitchmat,  tmp);
  myMMult(tmp, rollmat,  matrix);

}


class stampedpose {
  
  static double linear_deg_interp(double a, double b, double s) {
    double diff1 = fmod( b - a + 360.0, 360.0); // ensure this is positive
    double diff2 = diff1 - 360; 

    if ( fabs(diff1) < fabs(diff2) ) { // check if we should increase or decrease
      return a + s * diff1;
    }
    return a + s * diff2;

  }

  static double linear_interp(double a, double b, double s) {
    return a + s * (b-a);
  }

  static double a;
  static double f;

public:

  static void interpolate(stampedpose &first, stampedpose &second, double time, double *matrix) {
    double y1 = first.time;
    double y2 = second.time;
    double s = (time-y1)/(y2-y1);

    double yaw       = linear_deg_interp(first.yaw       , second.yaw       , s);
    double roll      = linear_deg_interp(first.roll      , second.roll      , s);
    double pitch     = linear_deg_interp(first.pitch     , second.pitch     , s);
    double longitude = linear_interp(first.longitude , second.longitude , s);
    double latitude  = linear_interp(first.latitude  , second.latitude  , s);
    double altitude  = linear_interp(first.altitude  , second.altitude  , s);


    double body2ned[16];

    euler2matrix(roll, pitch, yaw, body2ned); 

    double phi = rad(latitude);
    double lambda = rad(longitude);
    double vh = a / (sqrt(1 - f * (2 - f) * sin(phi)*sin(phi) ));

    double X = (vh + altitude) * cos(phi)*cos(lambda);
    double Y = (vh + altitude) * cos(phi)*sin(lambda);
    double Z = (vh * (1 - f * (2 - f)) + altitude) * sin(phi);
    
    double ned2ecef[16] = {
      -cos(lambda)*sin(phi),  -sin(lambda)*sin(phi),  cos(phi), 0,
      -sin(lambda),           cos(lambda),            0,        0,
      -cos(lambda)*cos(phi), -sin(lambda)*cos(phi),  -sin(phi), 0,
      X, Y, Z, 1};

    myMMult(ned2ecef, body2ned, matrix);

  }

  static void transform(double *tmat, double *point) {
    double x = tmat[0] * point[0] + tmat[4] * point[1] + tmat[8] * point[2] + tmat[12];
    double y = tmat[1] * point[0] + tmat[5] * point[1] + tmat[9] * point[2] + tmat[13];
    double z = tmat[2] * point[0] + tmat[6] * point[1] + tmat[10] * point[2] + tmat[14];
    point[0] = x;
    point[1] = y;
    point[2] = z;
  }

  double time;
  double yaw;
  double roll;
  double pitch;
  double longitude;
  double latitude;
  double altitude;

  /**
   *  Overridden ">>" operator for reading a point from a stream.
   *  Throws a runtime error if not enough data in the stream.
   */
  inline friend istream& operator>>(istream& is, stampedpose &sp) {
    if (!is.good()) throw runtime_error("Not enough elements to read for >>(istream&, stampedpose).1");
    is >> sp.time;
    if (!is.good()) throw runtime_error("Not enough elements to read for >>(istream&, stampedpose).2");
    is >> sp.latitude;
    if (!is.good()) throw runtime_error("Not enough elements to read for >>(istream&, stampedpose).3");
    is >> sp.longitude;
    if (!is.good()) throw runtime_error("Not enough elements to read for >>(istream&, stampedpose).4");
    is >> sp.altitude;
    if (!is.good()) throw runtime_error("Not enough elements to read for >>(istream&, stampedpose).5");
    is >> sp.roll;
    if (!is.good()) throw runtime_error("Not enough elements to read for >>(istream&, stampedpose).6");
    is >> sp.pitch;
    if (!is.good()) throw runtime_error("Not enough elements to read for >>(istream&, stampedpose).7");
    is >> sp.yaw;
    return is;
  }
  
};

double stampedpose::a = 6378137;
double stampedpose::f = 1.0/298.257223563;


class rxpreader
    : public pointcloud
{
public:
    rxpreader(std::string _rxpfile, string _trajectoryfile, string _outdir, double *_socs2body, unsigned int _pointtype = PointType::USE_NONE)
      : pointcloud(true)
    {
      rxpfile = "file://" + _rxpfile;

      trajectoryfile = _trajectoryfile;
      outdir = _outdir;

      for (int i = 0; i < 16; i++) socs2body[i] = _socs2body[i];

      pointtype = PointType(PointType::USE_TIME | _pointtype); 
    }

    void readFromFile(double fixtime) {
      shared_ptr<basic_rconnection> rc;
      rc = basic_rconnection::create( rxpfile );
//      cout << "rxpreader opening file " << rxpfile << endl;
      rc->open();
      decoder_rxpmarker dec(rc);
      buffer buf;
      firstpoint = true;

      /********************/
      trajectory.open(trajectoryfile.c_str());
      // read first two positions

      readnextpose();
      while ( !(pose_a.time < fixtime && pose_b.time > fixtime )) {
        readnextpose();
      }
      
      double body2ecef[16];
      stampedpose::interpolate(pose_a, pose_b, fixtime, body2ecef);
      M4inv(body2ecef, firstposeinverse);


      trajectory.close();
      /*************/


      // open the trajectory file
      trajectory.open(trajectoryfile.c_str());
      // read first two positions
      readnextpose();
      readnextpose();
      
      cerr << body2ecef << endl;
      cerr << pose_a.time << " " 
        << pose_a.latitude << " "  
        << pose_a.longitude << " "  
        << pose_a.altitude << " "  
        << pose_a.yaw << " "  
        << pose_a.pitch << " "  
        << pose_a.roll << endl;

      scannr = 0;
      pointnr = 1;
      
      prepareLine();
      firstlinepoint = true;

      for ( dec.get(buf); !dec.eoi() ; dec.get(buf) ) {
        this->dispatch(buf.begin(), buf.end());
      }
      prepareLine();

      rc->close();
    }

    ~rxpreader() {
    }

protected:

    double socs2body[16];

    bool firstpoint;
    bool firstlinepoint;
    double linematrix[16];

    std::string rxpfile;
    std::string trajectoryfile;
    std::string outdir;
    vector<double*> pbuffer;
    ifstream trajectory;

    ofstream scanfile;
    unsigned int scannr;
    unsigned int pointnr;
    double firstposeinverse[16];
    double firstlineposeinverse[16];
    double firstlinepose[16];
    
    double frametransmat[16];
    double frametransmatinv[16];

    stampedpose pose_a;
    stampedpose pose_b;

    int day;

    bool readnextpose() {
      static int counter = 0;
    
      //cerr << counter++ << endl;
      try {
        pose_a = pose_b;
        trajectory >> pose_b;
      } catch (...) {
        return false;
      }
      return true;
    }

    void setTransform(double *transmat, double *mat, double x, double y, double z) {
      transmat[0]  = -mat[5]; 
      transmat[1]  = mat[6];
      transmat[2]  = -mat[4];
      transmat[3]  = 0.0;

      transmat[4]  = mat[9];
      transmat[5]  = -mat[10]; 
      transmat[6]  = mat[8];
      transmat[7]  = 0.0;

      transmat[8]  = mat[1];
      transmat[9]  = -mat[2];
      transmat[10] = mat[0];
      transmat[11] = 0.0;

      // translation
      transmat[12] = y * 100.0; 
      transmat[13] = z * -100.0;
      transmat[14] = x * 100.0;
      transmat[15] = 1;
      /*
      transmat[0] = mat[4];                                                                                         
      transmat[1] = -mat[7]; 
      transmat[2] = -mat[1];                                                                                        
      transmat[3] = 0.0;

      transmat[4] = -mat[5];                                                                                        
      transmat[5] = mat[8];                                                                                        
      transmat[6] = mat[2];
      transmat[7] = 0.0;

      transmat[8] = -mat[3];                                                                                        
      transmat[9] = mat[6];
      transmat[10] = mat[0];                                                                                        
      transmat[11] = 0.0;

      // translation
      transmat[12] =  -y;
      transmat[13] =  z;
      transmat[14] =  x;
      transmat[15] = 1;
*/
    }

    void on_echo_transformed(echo_type echo)
    {
      target& t(targets[target_count - 1]);

      double time = t.time;

      // compute day difference between timestamps
      if (firstpoint) {
        double tdiff = pose_a.time - time;
        day = tdiff/86400;

        double tdm1 = fabs(pose_a.time - time - (day-1)*86400.0);
        double td   = fabs(pose_a.time - time -  day   *86400.0);
        double tdp1 = fabs(pose_a.time - time - (day+1)*86400.0);

        if (tdm1 < td && tdm1 < tdp1) {
          day = day-1;
        } else if (tdp1 < tdm1 && tdp1 < td) {
          day = day+1;
        }

//        cerr << "day is " << day << endl;

      }

      time = time + 86400.0*day;

      if (firstpoint) {
      cout.precision(20);
        cout << "If you want to use this data set as a fixed reference use the timestamp " << time << endl; 
      }

      scanfile.precision(20);
      cerr.precision(20);
      
      if (time < pose_a.time)
        return;

      bool nextpose = false;
      while (time > pose_b.time) {
        nextpose = true;
        if (!readnextpose()) {
          return;
        }
      }

      // the current point must now be between timestamps a and b
     /*
      if (nextpose) {
        double tm[16];
        stampedpose::interpolate(pose_a, pose_b, pose_a.time, tm);
        cerr << tm << endl;
      }*/


      double body2ecef[16];
      stampedpose::interpolate(pose_a, pose_b, time, body2ecef);

      /*
      if ( sqrt(t.vertex[0]*t.vertex[0] + t.vertex[1]*t.vertex[1] + t.vertex[2]*t.vertex[2]) < 0.8)
        return;
      if (t.deviation > 10.0) {
        return;
      }*/


      double *p = new double[pointtype.getPointDim()];
      for (unsigned int i = 0; i < 3; i++)
        p[i] = t.vertex[i];  // point is in socs
      if (pointtype.hasReflectance()) 
        p[pointtype.getReflectance()] = t.reflectance;
      if (pointtype.hasAmplitude()) 
        p[pointtype.getAmplitude()] = t.amplitude;
      if (pointtype.hasDeviation()) 
        p[pointtype.getDeviation()] = t.deviation;



      double socs2ecef[16];

      myMMult(body2ecef, socs2body, socs2ecef);
      stampedpose::transform(socs2ecef, p); // transformed point from socs into ecef

      stampedpose::transform(firstposeinverse, p);  // transformed point from ecef into local data set system as defined by the fix time

/*      if (firstpoint) {
        myMMult(firstposeinverse, socs2ecef, firstlinepose);
        M4inv(firstlinepose, firstlineposeinverse);
      } */
      
      if (firstlinepoint) { // first point in the current slice
        // remember the transformation from the data set system to the local coordinate system at this point in time
        double ecef2socs[16];
        double firstpose[16];
        M4inv(firstposeinverse, firstpose);
        M4inv(socs2ecef, ecef2socs);
        myMMult(ecef2socs, firstpose, frametransmatinv);
        // remember the inverse of the trnasformation from global to local for this slice, so we can transform them correctly using show 
        //M4inv(frametransmatinv, frametransmat);
        double mat[16];
        M4inv(frametransmatinv, mat);
        setTransform(frametransmat, mat, mat[12], mat[13], mat[14]);
        //M4inv(frametransmatinv, frametransmat);
      }
      firstlinepoint = false;
      
      cout.precision(7);
      
/*      stampedpose::transform(firstlineposeinverse, p);  // transformed point from local dataset system into the SOCS of the first point in the current line
      stampedpose::transform(socs2body, p); // transformed point into the body coordinate system at the first point in the current line
*/      

      // transform into the local coordinate system, points are now almost like in the rxp, except they have been motion compensated, 
      // i.e. points are in the coordinate system of the scanner at the time of the first reflection
      stampedpose::transform(frametransmatinv, p);
//      cout  << "ORIG " << p[0] << " "  << p[1] << " " << p[2] << endl;

/*
   // this is the points in the .3d files, i.e. how they will be written in the file
      cout  << ".3d "
            << p[1] * -100.0<< " " 
            << p[2] * 100.0<< " " 
            << p[0] * 100.0<< endl;
        
      double mp[3];
      mp[0] = p[1] * -100.0;
      mp[1] = p[2] * 100.0;
      mp[2] = p[0] * 100.0;
   
      // this is the points as they should be in show 
      
      stampedpose::transform(frametransmat, p);
      double rP[3], rPT[3], em[16];
      rP[0] = rP[1] = rP[2] = 0.0;
      rPT[0] = ::rad(0.0);
      rPT[1] = ::rad(180.0);
      rPT[2] = ::rad(180.0);
      EulerToMatrix4(rP, rPT, em);
      stampedpose::transform(em, p);

      // at the end this is the way the points should be in the global coordinate sense as far as 3dtk is concerned
      cout  << "TARGET "
            << p[1] * -100.0<< " " 
            << p[2] * 100.0<< " " 
            << p[0] * 100.0<< endl;
      
      
      // this is to test wether the setTransform is working
      double mat[16]; 
      setTransform(mat, frametransmat, frametransmat[12], frametransmat[13], frametransmat[14]);
      Point ppp(mp[0], mp[1], mp[2]);
      ppp.transform(mat);
      cout  << "SHOW " << ppp.x << " "   << ppp.y << " "     << ppp.z << endl << endl;
  

      for (int i = 0; i < 3; i++) {
        double mp[3];
        mp[0] = mp[1] = mp[2] = 0.0;
        mp[i] = 0.01;
        // this is the points in the .3d files
        cout  << ".3d "
          << mp[1] * -100.0<< " " 
          << mp[2] * 100.0<< " " 
          << mp[0] * 100.0<< endl;

        // this is the points as they should be in show 
        stampedpose::transform(frametransmat, mp);
        double rP[3], rPT[3], em[16];
        rP[0] = rP[1] = rP[2] = 0.0;
        rPT[0] = ::rad(0.0);
        rPT[1] = ::rad(180.0);
        rPT[2] = ::rad(180.0);
        EulerToMatrix4(rP, rPT, em);
        stampedpose::transform(em, mp);
        cout  << "TARGET " 
          << mp[1] * -100.0<< " " 
          << mp[2] * 100.0<< " " 
          << mp[0] * 100.0<< endl;
      
        
        double mat[16]; 
        setTransform(mat, frametransmat, frametransmat[12], frametransmat[13], frametransmat[14]);
        mp[0] = mp[1] = mp[2] = 0.0;
        mp[i] = 0.01;
        Point ppp(mp[1] * -100.0, mp[2] * 100.0, mp[0] * 100.0);
        ppp.transform(mat);
        cout  << "SHOW " << ppp.x << " "   << ppp.y << " "     << ppp.z << endl;
      }
      

      
      double rPos[3], rPosTheta[3];
      setTransform(mat, frametransmat, frametransmat[12], frametransmat[13], frametransmat[14]);

      Matrix4ToEuler(mat, rPosTheta, rPos);
      cout << mat << endl;
      cout << deg(rPosTheta[0]) << " " << deg(rPosTheta[1]) << " " << deg(rPosTheta[2]) << endl;
      EulerToMatrix4(rPos, rPosTheta, mat);
      cout << mat << endl;
      Matrix4ToEuler(mat, rPosTheta, rPos);
      cout << deg(rPosTheta[0]) << " " << deg(rPosTheta[1]) << " " << deg(rPosTheta[2]) << endl;
      exit(0);
  */        
        
      /*
      double mat[16], mat2[16]; 
      M4inv(frametransmatinv, mat);
      setTransform(mat2, mat, mat[12], mat[13], mat[14]);
      Point ppp(p[1] * -100.0, p[2] * 100.0, p[0] * 100.0);
      ppp.transform(mat2);
      cout  << "3DF2 " << ppp.x << " "   << ppp.y << " "     << ppp.z << endl;
      */

      //exit(0);
     /* 
      scanfile  << p[0] << " " 
            << p[1] << " " 
            << p[2] << endl;
            */
      

    /* 
      scanfile  << p[1] * -100.0<< " " 
            << p[2] * -100.0<< " " 
            << p[0] * -100.0<< endl;
            */
       
      scanfile  << p[1] * -100.0<< " " 
            << p[2] * 100.0<< " " 
            << p[0] * 100.0<< endl;
    
    
     /*
      scanfile  << p[1] * -100.0<< " " 
            << p[2] * -100.0<< " " 
            << p[0] * 100.0<< endl;
            */


      pointnr++;
      
      firstpoint = false;
    }

    void on_frame_stop(const frame_stop<iterator_type>& arg) {
    }
    void on_frame_start_dn(const frame_start_dn<iterator_type>& arg) { 
      basic_packets::on_frame_start_dn(arg);                                                                        
    } 
    void on_frame_start_up(const frame_start_up<iterator_type>& arg) {
      basic_packets::on_frame_start_up(arg);
    }

    void prepareLine() {
      if (pointnr!=0) {
        // TODO write pose and frames file
        pointnr=0;

        int mscannr = scannr;
       /* 
        int mscannr = 11300-scannr;
        if (mscannr < 7600) exit(0);
        */
     
        string filename = outdir + "scan" + to_string(mscannr-1,3) + ".frames";
        if (scannr > 0) {
//          M4identity(frametransmat); // TODO

          ofstream file(filename.c_str());
          file.precision(15);
          file << frametransmat << " 1" << endl;
          file << frametransmat << " 1" << endl;
          file << frametransmat << " 1" << endl;
          file.close();

          double rPosTheta[3], rPos[3];
          Matrix4ToEuler(frametransmat, rPosTheta, rPos);
          filename = outdir + "scan" + to_string(mscannr-1,3) + ".pose";
          file.open(filename.c_str());
          file.precision(15);
          file << rPos[0] << " " << rPos[1] << " " << rPos[2] << endl;
          file << deg(rPosTheta[0]) << " " << deg(rPosTheta[1]) << " " << deg(rPosTheta[2]) << endl;
          file.close();
        }
        


        filename = outdir + "scan" + to_string(mscannr,3) + ".3d";
        scanfile.close();
        cout << "opening file " << filename << endl;
        scanfile.open(filename.c_str());
        scannr++;
        firstlinepoint = true;

      }
    }

    void on_line_start_dn(const line_start_dn<iterator_type>& arg) {
      basic_packets::on_line_start_dn(arg);
      prepareLine();
    }
    void on_line_start_up(const line_start_up<iterator_type>& arg) {
      basic_packets::on_line_start_up(arg);
      prepareLine();
    }

    PointType pointtype;
};



void readFrames(string filename, double matrix[16]) {
  ifstream frames;
  double dummy;
  double mat[16];

  frames.open(filename.c_str());

  while (frames.good()) {
    try {
      frames >> mat;
      frames >> dummy;
    } catch(...) {
      break;
    }
    memcpy(matrix, mat, sizeof(double)*16);
  }

  frames.close();

}
void readUntil(double time, ifstream *scan_in, vector<Point> &points) {

  for (int i = 1; i <= NR_SCANNERS; i++) {
    while (scan_in[i-1].good()) {
      Point p;
      try {
        scan_in[i-1] >> p.deviation; //timestamp;
        scan_in[i-1] >> p.x;
        scan_in[i-1] >> p.z;
        scan_in[i-1] >> p.y;
        scan_in[i-1] >> p.reflectance;
        p.x *= 100.0;
        p.y *= 100.0;
        p.z *= 100.0;
      } catch (...) {
        break;
      }
      p.type = i;
      points.push_back(p);

      //if (p.timestamp > time)
      if (p.deviation > time)
        break;

    }
  /*  scan_in[i-1].close();
    scan_in[i-1].clear();*/
  }
}
void convert(string dir, string outdir) {

  string FileName;

  ifstream scan_in[NR_SCANNERS];
  ifstream traj_in;

  for (int i = 1; i <= NR_SCANNERS; i++) {
    FileName = dir + "scan" + to_string(i,3) + ".xyz";
    scan_in[i-1].open(FileName.c_str());
    cout << "opening " << FileName << endl;
  }
  FileName = dir + "trajectory.asc";
  traj_in.open(FileName.c_str());
  cout << "opening " << FileName << endl;



  double globmat[16], globinv[16];
  double tmpmat[16];
  bool firstmatread = false;


  double time, d;
  double rPos[3], rPosTheta[3];
  double transmat[16];
  double tinv[16];
  vector<Point> points;
  unsigned int scannr = 0;
  cout.precision(15);


  
  while (traj_in.good()) {
    traj_in >> time;
    
    traj_in >> rPos[0];
    traj_in >> rPos[2];
    traj_in >> rPos[1];
    for (int i =0;i<3;i++) rPos[i] *= 100.0;
    
    traj_in >> rPosTheta[0];
    traj_in >> rPosTheta[1];
    traj_in >> rPosTheta[2];
    for (int i =0;i<3;i++) rPosTheta[i] = 0.0; // ignore rotation
    
    if (!firstmatread) {
      EulerToMatrix4(rPos, rPosTheta, globmat);
      M4inv(globmat, globinv);
      firstmatread = true;
    }

    EulerToMatrix4(rPos, rPosTheta, transmat);
    M4inv(transmat, tinv);

    MMult(transmat, globinv, tmpmat);
    Matrix4ToEuler(tmpmat, rPosTheta, rPos);

    traj_in >> d;
    traj_in >> d;
    traj_in >> d;
    
    points.clear();

    cout << "scan " << scannr << " read until " << time << endl;
    readUntil(time, scan_in, points);

    // now write points away
    FileName = outdir + "scan" + to_string(scannr,3) + ".3d";
    ofstream scan_out(FileName.c_str());
    scan_out.precision(15);
    cout << "save to " << FileName << endl;

    for (unsigned int i = 0; i < points.size(); i++) {
      points[i].transform(tinv);
      scan_out << points[i].x << " " << points[i].y << " " << points[i].z << " " << points[i].reflectance << endl;
    }
    scan_out.close();



    // and the .pose file
    FileName = outdir + "scan" + to_string(scannr,3) + ".pose";
    scan_out.open(FileName.c_str());
    scan_out.precision(15);
    scan_out << rPos[0] << " " << rPos[1] << " " << rPos[2] << endl;
    scan_out << rPosTheta[0] << " " << rPosTheta[1] << " " << rPosTheta[2] << endl;
    scan_out.close();
    // and the .frames file
    FileName = outdir + "scan" + to_string(scannr,3) + ".frames";
    scan_out.open(FileName.c_str());
    scan_out.precision(15);
    scan_out << tmpmat << " 1" << endl;
    scan_out << tmpmat << " 1" << endl;
    scan_out << tmpmat << " 1" << endl;
    scan_out.close();

    scannr++;
  }

  for (int i = 1; i <= NR_SCANNERS; i++) {
    scan_in[i].close();
  }
  traj_in.close();
} 


void modify(string dir, string framesdir, string outdir) {

  string FileName;

  ifstream scan_in[NR_SCANNERS];
  ofstream scan_out[NR_SCANNERS];
  ifstream traj_in;

  for (int i = 1; i <= NR_SCANNERS; i++) {
    FileName = dir + "scan" + to_string(i,3) + ".xyz";
    scan_in[i-1].open(FileName.c_str());
    cout << "opening " << FileName << endl;
  }
  FileName = dir + "trajectory.asc";
  traj_in.open(FileName.c_str());
  cout << "opening " << FileName << endl;

  for (int i = 1; i <= NR_SCANNERS; i++) {
    FileName = outdir + "scan" + to_string(i,3) + ".xyz";
    scan_out[i-1].open(FileName.c_str());
    scan_out[i-1] << fixed;
    scan_out[i-1].precision(3);
    cout << "opening " << FileName << " for writing" << endl;
  }


  double globmat[16], globinv[16];
  double tmpmat[16];
  bool firstmatread = false;


  double time, d;
  double rPos[3], rPosTheta[3];
  double transmat[16];
  double tinv[16];
  vector<Point> points;
  unsigned int scannr = 0;
//  unsigned int scannr = 4999;
  cout.precision(15);
  cout << fixed;


  
  while (traj_in.good()) {
    traj_in >> time;
    
    traj_in >> rPos[0];
    traj_in >> rPos[2];
    traj_in >> rPos[1];
    for (int i =0;i<3;i++) rPos[i] *= 100.0;
    
    traj_in >> rPosTheta[0];
    traj_in >> rPosTheta[1];
    traj_in >> rPosTheta[2];
    for (int i =0;i<3;i++) rPosTheta[i] = 0.0; // ignore rotation
    
    if (!firstmatread) {
      EulerToMatrix4(rPos, rPosTheta, globmat);
      M4inv(globmat, globinv);
      firstmatread = true;
    }

    EulerToMatrix4(rPos, rPosTheta, transmat);
    M4inv(transmat, tinv);

    MMult(transmat, globinv, tmpmat);
    Matrix4ToEuler(tmpmat, rPosTheta, rPos);

    traj_in >> d;
    traj_in >> d;
    traj_in >> d;
    
    points.clear();

    cout << "scan " << scannr << " read until " << time << endl;
    readUntil(time, scan_in, points);

    // now read frames file
    double framesmat[16];
    FileName = framesdir + "scan" + to_string(scannr,3) + ".frames";
    readFrames(FileName, framesmat);
    
    
    // write modified points to new file
    cout << "read from " << FileName << endl;

    for (unsigned int i = 0; i < points.size(); i++) {
      Point p = points[i];
      p.transform(tinv);
      // transform with frames
      p.transform(framesmat);

      // transform with global
      p.transform(globmat);

      // undo lefty/righty
      double x,y,z;
      x = p.x / 100.0;
      y = p.y / 100.0;
      z = p.z / 100.0;
      p.x = x;
      p.y = z;
      p.z = y;
      scan_out[p.type - 1] << setprecision(6) <<  p.deviation << " "; //p.timestamp << " ";
      scan_out[p.type - 1].width(8);
      scan_out[p.type - 1] << setprecision(3) << p.x << " ";
      scan_out[p.type - 1].width(8);
      scan_out[p.type - 1] << p.y << " ";
      scan_out[p.type - 1].width(8);
      scan_out[p.type - 1] << p.z << " ";
      scan_out[p.type - 1].width(4);
      scan_out[p.type - 1] << (int)p.reflectance << endl;
    }

    scannr++;

    if (scannr > 19000) break; 
    
  }

  for (int i = 1; i <= NR_SCANNERS; i++) {
    scan_in[i].close();
    scan_out[i].close();
  }
  traj_in.close();
} 


void modifyTrajectory(string dir, string framesdir, string outfile) {

  string FileName;

  ofstream traj_out; 
  ifstream traj_in;

  FileName = dir + "trajectory.asc";
  traj_in.open(FileName.c_str());
  cout << "opening " << FileName << endl;

  FileName = outfile;
  traj_out.open(FileName.c_str());
  traj_out << fixed;
  traj_out.precision(3);
  cout << "opening " << FileName << " for writing" << endl;


  double globmat[16], globinv[16];
  double tmpmat[16];
  bool firstmatread = false;


  double time, d,e,f;
  double rPos[3], rPosTheta[3];
  double transmat[16];
  double tinv[16];
  vector<Point> points;
  unsigned int scannr = 4999;
  //unsigned int scannr = 0;
  cout.precision(15);
  cout << fixed;


  
  while (traj_in.good()) {
    traj_in >> time;
    
    traj_in >> rPos[0];
    traj_in >> rPos[2];
    traj_in >> rPos[1];
    for (int i =0;i<3;i++) rPos[i] *= 100.0;
    
    traj_in >> rPosTheta[0];
    traj_in >> rPosTheta[1];
    traj_in >> rPosTheta[2];
    for (int i =0;i<3;i++) rPosTheta[i] = 0.0; // ignore rotation
    
    if (!firstmatread) {
      EulerToMatrix4(rPos, rPosTheta, globmat);
      M4inv(globmat, globinv);
      firstmatread = true;
    }

    EulerToMatrix4(rPos, rPosTheta, transmat);
    M4inv(transmat, tinv);

    MMult(transmat, globinv, tmpmat);
    Matrix4ToEuler(tmpmat, rPosTheta, rPos);

    traj_in >> d;
    traj_in >> e;
    traj_in >> f;
    
    points.clear();

    // now read frames file
    double framesmat[16];
    FileName = framesdir + "scan" + to_string(scannr,3) + ".frames";
    readFrames(FileName, framesmat);
    
    
    // write modified position to file
    cout << "read from " << FileName << endl;

      Point p(0,0,0);
      //p.transform(tinv);
      // transform with frames
      p.transform(framesmat);

      // transform with global
      p.transform(globmat);

      // undo lefty/righty
      double x,y,z;
      x = p.x / 100.0;
      y = p.y / 100.0;
      z = p.z / 100.0;
      p.x = x;
      p.y = z;
      p.z = y;
      traj_out << setprecision(5) <<  "   " << time << "    ";
      traj_out.width(10);
      traj_out << setprecision(3) << p.x << "  ";
      traj_out.width(10);
      traj_out << p.y << " ";
      traj_out.width(10);
      traj_out << p.z << "";
//      traj_out.width(4);
      traj_out << "    0.000000    0.000000    0.000000  0.000  0.000  0.000" << endl;

    scannr++;

    if (scannr > 19000) break; 
    
  }

  traj_in.close();
  traj_out.close();
} 


int parseArgs(int argc, char **argv, string &dir, string &outdir, bool &xyz23d, string &framesdir) {
  if (argc == 3) {
    dir = argv[1];
    outdir = argv[2];
    xyz23d = true;
  } else if (argc == 4) {
    dir = argv[1];
    framesdir = argv[2];
    outdir = argv[3];
    xyz23d = false;
  }
  return 0;
}

  

void socs2bodySALZBURG(string projectfile, double *socs2body) {
  // TODO actually use the project file

  // under <object name="NAVIGATION DEVICES" kind="NAVDEVICES" states="expanded"> from 
  //  <field name="offset-roll" data="0" kind="double" unit="rad"/>
  //  <field name="offset-pitch" data="0" kind="double" unit="rad"/>
  //  <field name="offset-yaw" data="0" kind="double" unit="rad"/>
  double ins_calib[16];
  euler2matrix(0,0,0,  ins_calib);

  // from 'field name="matrix-imu-body" data="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 "'
  double imu2body[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  // rotational part of imu2body, in this case the same thing
  double imu2bodyrot[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  // under '<object name="LASER DEVICES" '  from <field name="matrix-socs-imu" data="1 0 0 0 0 -1 0 0 0 0 -1 0 -0.12 0.84 -0.37 1 " 
  double socs2imu[16] = {1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, -0.12, 0.84, -0.37, 1};
  // rotational part of socs2imu
  double socs2imurot[16] = {1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1};
  // translational part of socs2imu
  double socs2imutrans[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -0.12, 0.84, -0.37, 1};

  // under <object name="LASER DEVICES" kind="LASDEVICES" states="expanded">  from
  // <field name="calib-offset-roll" data="0" kind="double" unit="rad"/>
  // <field name="calib-offset-pitch" data="0" kind="double" unit="rad"/>
  // <field name="calib-offset-yaw" data="0" kind="double" unit="rad"/>
  double scx_calib[16];
  euler2matrix(0,0,0,  scx_calib);


  // now mulitply to get socs2body
  // ins_calib * imu2body * socs2imutrans * imu2bodyrot^-1 * scx_calib * imu2bodyrot * socs2imurot  
  double imu2bodyrotinverse[16]; M4inv(imu2bodyrot, imu2bodyrotinverse); // TODO check if inverse is correct with this ordering of the matrix. Probably not !!!
  double ici2b[16];
  double ici2bs2it[16];
  double ici2bs2iti2bri[16];
  double ici2bs2iti2briscxc[16];
  double ici2bs2iti2briscxci2br[16];
  myMMult( ins_calib, imu2body, ici2b );
  myMMult( ici2b, socs2imutrans, ici2bs2it );
  myMMult( ici2bs2it, imu2bodyrotinverse, ici2bs2iti2bri );
  myMMult( ici2bs2iti2bri, scx_calib, ici2bs2iti2briscxc );
  myMMult( ici2bs2iti2briscxc, imu2bodyrot, ici2bs2iti2briscxci2br); 
  myMMult( ici2bs2iti2briscxci2br, socs2imurot, socs2body); 


/*  cout << "TESTING...:" << endl; 
  cout << socs2body << endl;
  cout << socs2imu << endl;*/
}


void socs2bodyWIEN(string projectfile, double *socs2body) {
  // TODO actually use the project file

  // under <object name="NAVIGATION DEVICES" kind="NAVDEVICES" states="expanded"> from 
  //  <field name="offset-roll" data="0" kind="double" unit="rad"/>
  //  <field name="offset-pitch" data="0" kind="double" unit="rad"/>
  //  <field name="offset-yaw" data="0" kind="double" unit="rad"/>
  double ins_calib[16];
  euler2matrix( deg(-0.000488692190558412), deg(0.00343829862642883), deg(-0.000593411945678072),  ins_calib);

  // from 'field name="matrix-imu-body" data="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 "'
  double imu2body[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  // rotational part of imu2body, in this case the same thing
  double imu2bodyrot[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

  // under '<object name="LASER DEVICES" '  from <field name="matrix-socs-imu" data="1 0 0 0 0 -1 0 0 0 0 -1 0 -0.12 0.84 -0.37 1 " 
  double socs2imu[16] = 
 {-0.499999999906659,  0.866025403838329,                  0, 0,
    0.49673176504393,  0.286788218191767, -0.819152044191268, 0,
  -0.709406479875736, -0.409576022019174,  -0.57357643649061, 0,   
              0.0772,            -0.3409,            -0.1009, 1};
  // rotational part of socs2imu
  double socs2imurot[16] = 
 {-0.499999999906659,  0.866025403838329,                  0, 0,
    0.49673176504393,  0.286788218191767, -0.819152044191268, 0,
  -0.709406479875736, -0.409576022019174,  -0.57357643649061, 0,   
                 0.0,                0.0,                0.0, 1};
  // translational part of socs2imu
  double socs2imutrans[16] = 
            {     1,          0,          0, 0, 
                  0,          1,          0, 0, 
                  0,          0,          1, 0, 
             0.0772,    -0.3409,    -0.1009, 1};

  // under <object name="LASER DEVICES" kind="LASDEVICES" states="expanded">  from
  // <field name="calib-offset-roll" data="0" kind="double" unit="rad"/>
  // <field name="calib-offset-pitch" data="0" kind="double" unit="rad"/>
  // <field name="calib-offset-yaw" data="0" kind="double" unit="rad"/>
  double scx_calib[16];
  euler2matrix(deg(0.000573515192205337), deg(-0.00252095357158061), deg(0.00211184839491314),  scx_calib);


  // now mulitply to get socs2body
  // ins_calib * imu2body * socs2imutrans * imu2bodyrot^-1 * scx_calib * imu2bodyrot * socs2imurot  
  double imu2bodyrotinverse[16]; M4inv(imu2bodyrot, imu2bodyrotinverse); // TODO check if inverse is correct with this ordering of the matrix. Probably not !!!
  double ici2b[16];
  double ici2bs2it[16];
  double ici2bs2iti2bri[16];
  double ici2bs2iti2briscxc[16];
  double ici2bs2iti2briscxci2br[16];
  myMMult( ins_calib, imu2body, ici2b );
  myMMult( ici2b, socs2imutrans, ici2bs2it );
  myMMult( ici2bs2it, imu2bodyrotinverse, ici2bs2iti2bri );
  myMMult( ici2bs2iti2bri, scx_calib, ici2bs2iti2briscxc );
  myMMult( ici2bs2iti2briscxc, imu2bodyrot, ici2bs2iti2briscxci2br); 
  myMMult( ici2bs2iti2briscxci2br, socs2imurot, socs2body); 


/*  cout << "TESTING...:" << endl; 
  cout << socs2body << endl;
  cout << socs2imu << endl;*/
}


int main(int argc, char *argv[]) {

  double socs2body[16];

  string dir            = "/home/jelseber/mls_salzburg_radar/";
  string trajectoryfile = "05_INS-GPS_PROC/01_POS/110427_155446.pos";
  string rxpfile        = "03_RIEGL_RAW/02_RXP/Scanner 1/110427_155457_Scanner 1.rxp";
  string projectfile    = "mls_salzburg_radar.rpp";
  trajectoryfile = dir + trajectoryfile;
  rxpfile = dir + rxpfile;
  projectfile = dir + projectfile;

//  socs2bodySALZBURG(projectfile, socs2body);

  
  if (argc != 4) {
    cerr << "Needs exactly one rxp file, one output directory as parameter and one timestamp which defines the coordinate system..." << endl;
    exit(0);
  }
  rxpfile = argv[1];
  string outdir = argv[2];
  double timestamp = atof(argv[3]);



  trajectoryfile = "/mnt/windowsC/WIEN/WIEN_VMX-450_Nuechter/05_INS-GPS_PROC/01_POS/11215_WIEN_VMX-450_WIEN_ETRS89.txt";
  socs2bodyWIEN(projectfile, socs2body);


  rxpreader reader(rxpfile, trajectoryfile, outdir, socs2body);
  reader.readFromFile(timestamp);
}


