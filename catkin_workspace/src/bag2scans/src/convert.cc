#include "slam6d/globals.icc"
#include "slam6d/point.h"

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
      scan_out[p.type - 1] << setprecision(6) << p.deviation << " "; // p.timestamp << " ";
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
}

int main(int argc, char *argv[]) {

  string dir, outdir, framesdir;
  bool xyz23d;

  parseArgs(argc, argv, dir, outdir, xyz23d, framesdir);

  if (xyz23d) {
    cout << "Converting ... "  << dir << " " << outdir << endl;
    convert(dir, outdir);
  } else {
    cout << "Modifying ... " << dir << " with " << framesdir << "  and writing to " << outdir << endl;
    modify(dir, framesdir, outdir);
    //modifyTrajectory(dir, framesdir, outdir);
  }

}


