#ifndef __CONTINUOUS_H_
#define __CONTINUOUS_H_


#include <vector>
using std::vector;

#include "slam6d/scan.h"
#include "slam6d/icp6Dquat.h"
#include "slam6d/icp6D.h"
#include "slam6d/graphSlam6D.h"
#include "graphSlam6DL.h"
#include "lum6Deuler.h"

#include "linescan.h"

inline void transform(double *point, const double *alignxf)
{
  double x_neu, y_neu, z_neu;
  x_neu = point[0] * alignxf[0] + point[1] * alignxf[4] + point[2] * alignxf[8];
  y_neu = point[0] * alignxf[1] + point[1] * alignxf[5] + point[2] * alignxf[9];
  z_neu = point[0] * alignxf[2] + point[1] * alignxf[6] + point[2] * alignxf[10];
  point[0] = x_neu + alignxf[12];
  point[1] = y_neu + alignxf[13];
  point[2] = z_neu + alignxf[14];
} 

// TODO fix rotation distribution to something smarter
inline void linearDistributeError(vector<LineScan*> &linescans, int begin, int end, const double  *transmatnew, bool globalanim=false) {
  int length = end - begin;
  double *transmatold = linescans[end]->transMat;
  double transmatnew2[16];
  double rPb[3], rPTb[3];
  double rPe[3], rPTe[3];
  Matrix4ToEuler(transmatold, rPTb, rPb);
  Matrix4ToEuler(transmatnew, rPTe, rPe);
  // compute linear differences between old and corrected pose
  double dP[6]; 
  dP[0] = dP[1] = dP[2] = dP[3] = dP[4] = dP[5] = 0.0;

  dP[0] = rPe[0]  - rPb[0] ;
  dP[1] = rPe[1]  - rPb[1] ;
  dP[2] = rPe[2]  - rPb[2] ;
  dP[3] = rPTe[0] - rPTb[0];
  dP[4] = rPTe[1] - rPTb[1];
  dP[5] = rPTe[2] - rPTb[2];

  /*
  cout << "end " << end << endl;
  cout << "rP target (old): " << rPe[0] << " " << rPe[1] << " " << rPe[2] << endl;;
  cout << "rP end (old): " << rPb[0] << " " << rPb[1] << " " << rPb[2] << endl;;
  cout << "TM target (old): " << transmatnew << endl;
  */
  rPb[0] =  rPb[0] + dP[0];
  rPb[1] =  rPb[1] + dP[1];
  rPb[2] =  rPb[2] + dP[2];
  rPTb[0] =  rPTb[0] + dP[3];
  rPTb[1] =  rPTb[1] + dP[4];
  rPTb[2] =  rPTb[2] + dP[5];
  EulerToMatrix4(rPb, rPTb, transmatnew2);
/*  cout << "TM end (old): " << linescans[end]->transMat << endl;
  cout << "rP end (new): " << rPb[0] << " " << rPb[1] << " " << rPb[2] << endl;;
  cout << "TM end (new)" << transmatnew2 << endl;*/
  



  /// compute transformation to apply to all other scans
  double transMatDIFF[16];
  double tinv2[16];
  M4inv(linescans[end]->transMat, tinv2);
  MMult(transmatnew2, tinv2, transMatDIFF);
  // done

  double *transMat;
 
  /// for a nice animation 
  if (!globalanim) {
    double id[16]; M4identity(id);
    // first show scans before they are matched and moved
    for (unsigned int j = 0;j<2;j++) {
      for (int i = 0; i < begin; i++)  { 
        linescans[i]->transform(id, Scan::ICPINACTIVE, 1);
      }
      for (int i = begin; i <= end; i++)  { 
        linescans[i]->transform(id, Scan::ICP, 1);
      }
      for (unsigned int i = end+1; i < linescans.size(); i++)  { 
        linescans[i]->transform(id, Scan::INVALID, 1);
      }
    }

    // next, show them after deformation. scans 0 to begin are not moved so plot again 
    for (int i = 0; i < begin; i++)  { 
      linescans[i]->transform(id, Scan::ICPINACTIVE, 1);
    }
  }
  ///////////

  for (int i = begin; i <= end; i++)  {
    transMat = linescans[i]->transMat;
    Matrix4ToEuler(transMat, rPTb, rPb);
    double t = ( (double)(i-begin) / (double) (length) );
    rPb[0] = rPb[0] + t * dP[0];
    rPb[1] = rPb[1] + t * dP[1];
    rPb[2] = rPb[2] + t * dP[2];
    rPTb[0] = rPTb[0] + t * dP[3];
    rPTb[1] = rPTb[1] + t * dP[4];
    rPTb[2] = rPTb[2] + t * dP[5];
  
    if (globalanim && i == begin) {
      linescans[i]->transformToEuler(rPb, rPTb, Scan::INVALID, 3);
    } else {
      linescans[i]->transformToEuler(rPb, rPTb, Scan::ICP, 1);
    }
  }


// transform all subsequent scans in the same manner
  for (unsigned int i = end+1; i < linescans.size(); i++)  {
    if (!globalanim) {
      linescans[i]->transform(transMatDIFF, Scan::INVALID, 1 );
    } else {
      linescans[i]->transform(transMatDIFF, Scan::INVALID, 3 );
    }
  }
}

inline void outputScans(vector<LineScan*> &linescans, int begin, int end, const char *filename, const double *transMat = 0 ) {
  if (begin < 0) begin = 0;
  if ((unsigned int)end >= linescans.size()) end = linescans.size() -1;
  ofstream fout(filename);
  double p[3];

//  double *transMat;
  double tinv[16];
  if (transMat == 0) {
    transMat = linescans[begin]->transMat;
  }

  M4inv(transMat, tinv);
  for (int i = begin; i <= end; i++)  {
    transMat = linescans[i]->transMat;
    transMat = linescans[i]->frames_mat;
    
    for (int j = 0; j < linescans[i]->nrpts; j++) {
      p[0] = linescans[i]->points[j][0];
      p[1] = linescans[i]->points[j][1];
      p[2] = linescans[i]->points[j][2];

      transform(p, transMat);
      transform(p, tinv);

      fout << p[0] << " " << p[1] << " " << p[2] << " " << linescans[i]->points[j][3] << endl;
    }
  }


}


/**
 * Creates a slam6D scan from the linescans begin-end, with the pose of index as reference coordinate system
 *
 */
Scan *joinLines(vector<LineScan*> &linescans, int begin, int end, int &index, double voxelsize = -1.0, int nrpts = 0 );

/**
 * Does ICP matching with a 3D scan created around linescan "index" (with "width" linescans in either direction)
 *
 * @param earliest is the index of the earliest linescan to match against
 * @param latest is the index of the latest linescan to match against
 *
 * Essentially we create 2 scans, that are rigidly matched using icp
 * the difference in the pose estimates of the "index" linescan is then distributed along the linescans in between
 *
 */
void preRegistration(vector<LineScan*> &linescans, int learliest, int llatest, int earliest, int latest, icp6D *icp, double voxelsize, int nr_octpts );

void Registration(vector<LineScan*> &linescans, graphSlam6D *slam, Graph *&gr, icp6D *icp=0 ,  int slamiters = 25, int clpairs = 100, int scansize = -1, double voxelsize = -1.0, int nr_octpts = 1);

void SemiRigidRegistration(vector<LineScan*> &linescans, graphSlam6DL *slam, Graph *&gr, int iterations, int slamiters, double mdm, int scaninterval, int scansize, double voxelsize, int nr_octpts);


double SSRR(vector<LineScan*> &linescans, graphSlam6D *slam, graphSlam6DL *slaml, Graph *&gr, icp6D *icp, int slamiters, int clpairs, double mdm, int scaninterval, int scansize, double voxelsize, int nr_octpts);
#endif
