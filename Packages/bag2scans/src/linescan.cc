#include "continuousreg.h"
#include "linescan.h"
#include "lboctree.h"
#include <algorithm>

#ifdef OPENMP
#include <omp.h>
#endif
using std::min;
using std::max;


vector <LineScan *>  LineScan::allLineScans;
bool LineScan::outputFrames = false;
unsigned int LineScan::counter = 0;
unsigned int LineScan::mindist = 180;
unsigned int LineScan::maxdist = 5080;


void LineScan::dumpFrames() {
  if (fileNr != -1) {
    string filename = basedir + "scan" + to_string(fileNr, 3) + ".frames";

    ofstream fout(filename.c_str());
    if (!fout.good()) {
      cerr << "ERROR: Cannot open file " << filename << endl;
      exit(1);
    }

    // write into file
    fout << sout.str();

    fout.close();
    fout.clear();
  }

}

LineScan::~LineScan() {
  if (outputFrames && fileNr != -1) {
    string filename = basedir + "scan" + to_string(fileNr, 3) + ".frames";

    ofstream fout(filename.c_str());
    if (!fout.good()) {
      cerr << "ERROR: Cannot open file " << filename << endl;
      exit(1);
    }

    // write into file
    fout << sout.str();

    fout.close();
    fout.clear();
  }

  // delete Scan from ScanList
/*  vector <Scan*>::iterator Iter;
  for(Iter = allScans.begin(); Iter != allScans.end();) {
    if (*Iter == this) {
      allScans.erase(Iter);
      break;
    } else {
      Iter++;
    }
  }*/

  for (int i = 0; i < nrpts; i++) {
    delete[] points[i];
  }
  delete [] points;

}

bool LineScan::readFromFile(int file_nr, string &dir, PointType &pointtype) {
  index = counter++;
  fileNr = file_nr;
  basedir = dir;
  
  ////////// ground truth
  string gtposeFileName = basedir + "ground_truth/scan" + to_string(fileNr,3) + ".pose";
  ifstream gtpose_in(gtposeFileName.c_str());
  for (unsigned int i = 0; i < 3; gtpose_in >> gtrPos[i++]);
  for (unsigned int i = 0; i < 3; gtpose_in >> gtrPosTheta[i++]);
  for (unsigned int i = 0; i < 3; i++) gtrPosTheta[i] = rad(gtrPosTheta[i]);
  EulerToMatrix4(gtrPos, gtrPosTheta, ground_truth_mat);
  //////////////////
  
  ////////// .frames file
  string frameFileName = basedir + "scan" + to_string(fileNr,3) + ".frames";
  ifstream frames_in(frameFileName.c_str());
  while(frames_in.good()) {
    for (unsigned int i = 0; i < 17; frames_in >> frames_mat[i++]);
  }
  //Matrix4ToEuler(tMatrix, rPosTheta, rPos);
  //////////////////////


  string scanFileName = basedir + "scan" + to_string(fileNr,3) + ".3d";
  string poseFileName = basedir + "scan" + to_string(fileNr,3) + ".pose";
  ifstream scan_in, pose_in;
  scan_in.open(scanFileName.c_str());
  pose_in.open(poseFileName.c_str());

  // read 3D scan
  if (!pose_in.good() && !scan_in.good()) return false; // no more files in the directory
  if (!pose_in.good()) { cerr << "ERROR: Missing file " << poseFileName << endl; exit(1); }
  if (!scan_in.good()) { cerr << "ERROR: Missing file " << scanFileName << endl; exit(1); }
  //cout << "Processing Scan " << scanFileName;

  for (unsigned int i = 0; i < 3; pose_in >> rPos[i++]);
  for (unsigned int i = 0; i < 3; pose_in >> rPosTheta[i++]);

  // convert angles from deg to rad
  for (unsigned int i = 0; i < 3; i++) rPosTheta[i] = rad(rPosTheta[i]);

  // overread the first line
  char dummy[255];
  scan_in.getline(dummy, 255);

  vector<Point> Ppoints;

  while (scan_in.good()) {
    Point p;
    try {
      scan_in >> p;
      if (pointtype.hasReflectance())
        scan_in >> p.reflectance;
      if (pointtype.hasAmplitude())
        scan_in >> p.amplitude;
      if (pointtype.hasDeviation())
        scan_in >> p.deviation;

//      if ( sqrt(p.x*p.x + p.y*p.y + p.z*p.z) > 150 ) 
      Ppoints.push_back(p);

    } catch (...) {
      break;
    }
  }
  scan_in.close();
  scan_in.clear();
  pose_in.close();
  pose_in.clear();


  nrpts = Ppoints.size();
  points = new double*[nrpts];

  for (int i = 0; i < nrpts; i++) {
    points[i] = pointtype.createPoint<double>(Ppoints[i]);
  }
  EulerToMatrix4(rPos, rPosTheta, orig_transMat);
/*  rPos[2] = fileNr * 0.10; 
  rPosTheta[1] = 0.3 * (rand() / (double)RAND_MAX); */
/*  rPos[0] = 0.0;
  rPos[1] = 10.0;
  rPos[2] = -20.0;*/

  EulerToMatrix4(rPos, rPosTheta, transMat);
  Ppoints.clear();

  /**************/
  double p[3];
  p[0] = points[0][0];
  p[1] = points[0][1];
  p[2] = points[0][2];
  ::transform(p, transMat);

  p[0] = p[0] - rPos[0];
  p[2] = p[2] - rPos[2];
//  double l = sqrt( p[0] * p[0] + p[2]*p[2]) * 0.01;
//  cout << rPos[0] << " " << rPos[2] << "  " << rPos[0] + p[0]/l << " " << rPos[2]+ p[2]/l << " " << atan2( p[0], p[2]) <<  endl;
  theta = atan2( p[0], p[2]);

  /*************/

  allLineScans.push_back(this);

  return true;
}

void LineScan::resetTransform() {
  Matrix4ToEuler(orig_transMat, rPosTheta, rPos);
  EulerToMatrix4(rPos, rPosTheta, transMat);
  EulerToMatrix4(rPos, rPosTheta, orig_transMat);

}

void LineScan::transformToEuler(double rP[3], double rPT[3], const Scan::AlgoType type, int islum)
{
  double tinv[16];
  double alignxf[16];
  M4inv(transMat, tinv);
  transform(tinv, Scan::INVALID, -1);
  EulerToMatrix4(rP, rPT, alignxf);
  transform(alignxf, type, islum);
}


void LineScan::transform(const double alignxf[16], const Scan::AlgoType type, int islum) {
  // update transformation
  double tempxf[16];
  MMult(alignxf, transMat, tempxf);
  memcpy(transMat, tempxf, sizeof(transMat));
  Matrix4ToEuler(transMat, rPosTheta, rPos);
  
  
  // store transformation
  int  found = 0;
  int end_loop = 0;
//  if (type != Scan::INVALID) {

    switch (islum) {
      case -1:
        // write no tranformation
        break;
      case 0:
        end_loop = (int)allLineScans.size();
        for (int iter = 0; iter < end_loop; iter++) {
          if (allLineScans[iter]->sout.good()) {
            allLineScans[iter]->sout << allLineScans[iter]->transMat;
            if (allLineScans[iter] == this ) {
              found = iter;
              allLineScans[iter]->sout << type << endl;
            } else {
              if (found == 0) {
                allLineScans[iter]->sout << Scan::ICPINACTIVE << endl;
              } else {
                allLineScans[iter]->sout << Scan::INVALID << endl;
              }
            }
          } else {
            cerr << "ERROR: Cannot store frames." << endl;
            exit(1);
          }
        }
        break;
      case 1:
        if (sout.good()) {
          sout << transMat << type << endl;
        } else {
          cerr << "ERROR: Cannot store frames." << endl;
          exit(1);
        }
        break;
      case 2:
        end_loop = (int)allLineScans.size();
        for (int iter = 0; iter < end_loop; iter++) {
          if (allLineScans[iter] == this) {
            found = iter;
            if (sout.good()) {
              sout << transMat << type << endl;
            } else {
              cerr << "ERROR: Cannot store frames." << endl;
              exit(1);
            }

            if (allLineScans[0]->sout.good()) {
              allLineScans[0]->sout << allLineScans[0]->transMat << type << endl;
            } else {
              cerr << "ERROR: Cannot store frames." << endl;
              exit(1);
            }
            continue;
          }
          if (found != 0) {
            allLineScans[iter]->sout << allLineScans[iter]->transMat << Scan::INVALID << endl;
          }
        }
        break;
      case 3:
        break;
      default:
        cerr << "invalid point transformation mode" << endl;
    }
//  }

}
  

double *LineScan::FindClosest(double *p, double maxdist2, int threadNum) {
  //p is the query point to be found

  // TODO implement the correct search for corresponding points 
  return p;
}


LScan::LScan(int _begin, int _end, int rep) {
  representative = min( max(0,rep), (int)LineScan::allLineScans.size()-1) ;
  begin = min( max(0,_begin), (int)LineScan::allLineScans.size()-1) ;
  end = min( max(0,_end), (int)LineScan::allLineScans.size()-1) ;
  scans.insert(scans.begin(), LineScan::allLineScans.begin() + begin, LineScan::allLineScans.begin() + end);
  memcpy(transMat, LineScan::allLineScans[representative]->transMat, 16*sizeof(double));
}

/*
LScan::LScan(vector<LineScan*> &_scans, unsigned int rep) {
  representative = rep;
  scans.insert( scans.begin(), _scans.begin(), _scans.end());
}*/

LScan::~LScan() {

}

void LScan::transform(const double alignxf[16], const Scan::AlgoType type, int islum) {
  cerr << "ERROR in LScan::transform()  Function not implemented!!!" << endl;

  double tempxf[16];
  MMult(alignxf, transMat, tempxf);
  memcpy(transMat, tempxf, sizeof(transMat));
  //Matrix4ToEuler(transMat, rPosTheta, rPos);


/*  for (unsigned int i = 0; i < scans.size(); i++) {
    scans[i]->transform(alignxf, type, islum);
  }*/
}
void LScan::writeAllPtPairs(vector<LScan *> &scans, int rnd, double max_dist_match2, string filename) {
  double centroid_m[3];
  double centroid_d[3];
  vector<PtPair> pairs;
  ofstream fout(filename.c_str());
  
  int k = 0; {
//  for (unsigned int k = 0; k < scans.size(); k++) {
//    int l = 0; {
    for (unsigned int l = 0; l < scans.size(); l++) {
      getPtPairs(&pairs, scans[k], scans[l], 0, rnd, max_dist_match2, centroid_m, centroid_d);
      for (unsigned int j = 0; j < pairs.size(); j++) {
        fout << 
          pairs[j].p1.x << " " << pairs[j].p1.y << " " << pairs[j].p1.z 
          << " " << 
          pairs[j].p2.x << " " << pairs[j].p2.y << " " << pairs[j].p2.z 
          << endl;
      }
    }
  }
/*
  for (vector<LineScan*>::iterator it = LTarget->scans.begin(); it != LTarget->scans.end(); it++) {
    for (vector<LineScan*>::iterator its = LSource->scans.begin(); its != LSource->scans.end(); its++) {
      LineScan *Source = *its;
      LineScan *Target = *it;

      if ( ((int)Source->index - (int)LineScan::maxdist <= (int)Target->index && (int)Target->index <= (int)Source->index - (int)LineScan::mindist) ||
          ((int)Source->index + (int)LineScan::mindist <= (int)Target->index && (int)Target->index <= (int)Source->index + (int)LineScan::maxdist) ) {

        Mcorr::iterator corrs = Source->correspondences.find(Target->index);
        if (corrs == Source->correspondences.end() ) continue;

        vlppair &corpts = corrs->second;

        if (!corpts.empty()) {
          fout << (int)Target->index << " " << (int)Source->index << endl;  
        }
      }
    }
  }*/

  fout.close();
  fout.clear();


}
void LScan::writePtPairs(LScan *LSource, LScan* LTarget, int rnd, double max_dist_match2, string filename) {
  double centroid_m[3];
  double centroid_d[3];
  vector<PtPair> pairs;
  ofstream fout(filename.c_str());
  getPtPairs(&pairs, LSource, LTarget, 0, rnd, max_dist_match2, centroid_m, centroid_d);
  for (unsigned int j = 0; j < pairs.size(); j++) {
    fout << 
      pairs[j].p1.x << " " << pairs[j].p1.y << " " << pairs[j].p1.z 
      << " " << 
      pairs[j].p2.x << " " << pairs[j].p2.y << " " << pairs[j].p2.z 
      << endl;
  }
/*
  for (vector<LineScan*>::iterator it = LTarget->scans.begin(); it != LTarget->scans.end(); it++) {
    for (vector<LineScan*>::iterator its = LSource->scans.begin(); its != LSource->scans.end(); its++) {
      LineScan *Source = *its;
      LineScan *Target = *it;

      if ( ((int)Source->index - (int)LineScan::maxdist <= (int)Target->index && (int)Target->index <= (int)Source->index - (int)LineScan::mindist) ||
          ((int)Source->index + (int)LineScan::mindist <= (int)Target->index && (int)Target->index <= (int)Source->index + (int)LineScan::maxdist) ) {

        Mcorr::iterator corrs = Source->correspondences.find(Target->index);
        if (corrs == Source->correspondences.end() ) continue;

        vlppair &corpts = corrs->second;

        if (!corpts.empty()) {
          fout << (int)Target->index << " " << (int)Source->index << endl;  
        }
      }
    }
  }*/

  fout.close();
  fout.clear();


}

//#define GROUND_TRUTH_PAIRS
  
void LScan::getPtPairs(vector <PtPair> *pairs, LScan *Source, LScan* Target,  int thread_num,
              int rnd, double max_dist_match2, double *centroid_m, double *centroid_d) {
#ifdef GROUND_TRUTH_PAIRS
  centroid_m[0] = 0.0;
  centroid_m[1] = 0.0;
  centroid_m[2] = 0.0;
  centroid_d[0] = 0.0;
  centroid_d[1] = 0.0;
  centroid_d[2] = 0.0;
  int np = 0;

  int beg1,end1, beg2,end2;
  beg1 = Source->getBegin();
  end1 = Source->getEnd();
  beg2 = Target->getBegin();
  end2 = Target->getEnd();

  int s,e;
  e = -1;

  if (beg1 <= beg2) {
    s = beg2;
    if ( beg2 <= end1) {
      e = end1;
    } else if ( end2 <= end1) {
      e = end2;
    }
  } else {
    s = beg1;
    if ( beg1 <= end2) {
      e = end2;
    } else if ( end1 <= end2) {
      e = end1;
    }
  }
//  for (int j = 0; j < LineScan::allLineScans.size(); j++) {
//  for (int j = s; j < e; j++) { 
//  for (int j = beg1; j < end1; j++) {  // use beg1 and end1 if no links are j > 0
//cout << "BEG END2 " << beg2 << " " << end2 << endl;
//cout << "BEG END1 " << beg1 << " " << end1 << endl;
  for (int j = beg2; j <= end2; j++) {  // use beg2 and end2 if no links are i > 0
    LineScan *scan = LineScan::allLineScans[j];
    double **points = scan->getPoints();
    for (unsigned int i = 0; i < scan->getNrPoints(); i++) {
      double s[3];
      s[0] = points[i][0];
      s[1] = points[i][1];
      s[2] = points[i][2];
      double t[3];
      t[0] = points[i][0];
      t[1] = points[i][1];
      t[2] = points[i][2];
      ::transform(t, scan->transMat);
      ::transform(s, scan->ground_truth_mat);

      PtPair myPair(s, t);
      pairs->push_back(myPair);

      centroid_d[0] += t[0];
      centroid_d[1] += t[1];
      centroid_d[2] += t[2];
      centroid_m[0] += s[0];  
      centroid_m[1] += s[1];
      centroid_m[2] += s[2];
      np++;
    }
  }
  centroid_d[0] /= np;
  centroid_d[1] /= np;
  centroid_d[2] /= np;
  centroid_m[0] /= np;  
  centroid_m[1] /= np;
  centroid_m[2] /= np;


#else
  centroid_m[0] = 0.0;
  centroid_m[1] = 0.0;
  centroid_m[2] = 0.0;
  centroid_d[0] = 0.0;
  centroid_d[1] = 0.0;
  centroid_d[2] = 0.0;


  for (vector<LineScan*>::iterator it = Target->scans.begin(); it != Target->scans.end(); it++) {
//    for (vector<LineScan*>::iterator its = Source->scans.begin(); its != Source->scans.end(); its++) {
    for (vector<LineScan*>::iterator its = LineScan::allLineScans.begin(); its != LineScan::allLineScans.end(); its++) {
      LineScan::getPtPairs(pairs, *its, *it, thread_num, rnd, max_dist_match2, centroid_m, centroid_d);
    }
  }
  ///////////////////
  
  centroid_m[0] /= pairs->size();
  centroid_m[1] /= pairs->size();
  centroid_m[2] /= pairs->size();
  centroid_d[0] /= pairs->size();
  centroid_d[1] /= pairs->size();
  centroid_d[2] /= pairs->size();
  return;
  ////////////////

  // this part is for adding a "stay where you are" weight
  int beg1,end1, beg2,end2;
  beg1 = Source->getBegin();
  end1 = Source->getEnd();
  beg2 = Target->getBegin();
  end2 = Target->getEnd();

  int s,e;
  e = -1;

  if (!(end1 < beg2 || end2 < beg1)) { // no overlap
    vector<int> ss;
    ss.push_back(beg1);
    ss.push_back(beg2);
    ss.push_back(end1);
    ss.push_back(end2);
    sort(ss.begin(), ss.end());
    s = ss[1];
    e = ss[2];

    /*
       if (beg1 <= beg2) {
       s = beg2;
       if ( beg2 <= end1) {
       e = end1;
       } else if ( end2 <= end1) {
       e = end2;
       }
       } else {
       s = beg1;
       if ( beg1 <= end2) {
       e = end2;
       } else if ( end1 <= end2) {
       e = end1;
       }
       }
       */

    unsigned int counter = 0;
    unsigned int max_counter = 1000;  // number of point pairs that "anchor" the scan
    for (int j = s; j < e; j++) {
      LineScan *scan = LineScan::allLineScans[j];
      double **points = scan->getPoints();
      for (unsigned int i = 0; i < scan->getNrPoints(); i++) {
        double s[3];
        s[0] = points[i][0];
        s[1] = points[i][1];
        s[2] = points[i][2];
        ::transform(s, scan->transMat);

        PtPair myPair(s, s);
        pairs->push_back(myPair);

        centroid_d[0] += s[0];
        centroid_d[1] += s[1];
        centroid_d[2] += s[2];
        centroid_m[0] += s[0];  
        centroid_m[1] += s[1];
        centroid_m[2] += s[2];

        counter++;
        if (counter > max_counter) break;
      }
      if (counter > max_counter) break;
    }

    /*
       for (vector<LineScan*>::iterator it = Target->scans.begin(); it != Target->scans.end(); it++) {
       LineScan *target = *it;
       for (int i = 0; i < target->nrpts; i++) {
       if (rnd > 1 && rand(rnd) != 0) continue;  // take about 1/rnd-th of the numbers only

       double p[3];
       p[0] = target->points[i][0];
       p[1] = target->points[i][1];
       p[2] = target->points[i][2];

       double x_neu, y_neu, z_neu;
    // transform point into the global coordinate system
    x_neu = p[0] * target->transMat[0] + p[1] * target->transMat[4] + p[2] * target->transMat[8];
    y_neu = p[0] * target->transMat[1] + p[1] * target->transMat[5] + p[2] * target->transMat[9];
    z_neu = p[0] * target->transMat[2] + p[1] * target->transMat[6] + p[2] * target->transMat[10];
    p[0] = x_neu + target->transMat[12];
    p[1] = y_neu + target->transMat[13];
    p[2] = z_neu + target->transMat[14];

    double *closest = LineScan::FindClosest(p, max_dist_match2, thread_num);
    if (closest) {
    centroid_d[0] += p[0];
    centroid_d[1] += p[1];
    centroid_d[2] += p[2];
    centroid_m[0] += closest[0];  
    centroid_m[1] += closest[1];
    centroid_m[2] += closest[2];
    PtPair myPair(closest, p);

    pairs->push_back(myPair);
    }
    }
    }
    */
  }
  centroid_m[0] /= pairs->size();
  centroid_m[1] /= pairs->size();
  centroid_m[2] /= pairs->size();
  centroid_d[0] /= pairs->size();
  centroid_d[1] /= pairs->size();
  centroid_d[2] /= pairs->size();
#endif

  return;
}

void LScan::getOdomPairs(vector <PtPair> *pairs, LScan *Source, LScan* Target,
              double *centroid_m, double *centroid_d, bool ground_truth) {
  centroid_m[0] = 0.0;
  centroid_m[1] = 0.0;
  centroid_m[2] = 0.0;
  centroid_d[0] = 0.0;
  centroid_d[1] = 0.0;
  centroid_d[2] = 0.0;
  
  LineScan *target = LineScan::allLineScans[Target->getRepresentative()];
  LineScan *source = LineScan::allLineScans[Source->getRepresentative()];
  
/*  LineScan *target = LineScan::allLineScans[Source->getRepresentative()];
  LineScan *source = LineScan::allLineScans[Target->getRepresentative()];*/

  double *orig_mat_prev;
  double *orig_mat;

  if (ground_truth) {
    orig_mat_prev = source->ground_truth_mat;
    orig_mat = target->ground_truth_mat;
  } else {
    orig_mat_prev = source->orig_transMat;
    orig_mat = target->orig_transMat;
  }

  double *mat_prev = source->transMat;

  double ompi[16];
  double odo[16];
  double glob2odo[16];
/*
  M4inv(orig_mat_prev, ompi);
  MMult( orig_mat, ompi, odo);
  MMult( odo, mat_prev, glob2odo);
  */

 /* 
  M4inv(orig_mat, ompi);
  MMult( orig_mat_prev, ompi, glob2odo);
  M4inv(glob2odo, odo);*/

  M4inv(orig_mat_prev, ompi);
  MMult( orig_mat, ompi, odo);

/*  
  M4inv(orig_mat_prev, ompi);
  MMult( mat_prev, ompi, odo);
  MMult( odo, orig_mat, glob2odo);
  */

if (ground_truth) {
  cout << "OMPI " << ompi << endl;
  cout << "MP   " << mat_prev << endl;
  cout << "G2OD " << glob2odo << endl;
}
    
    /*
cout << "OMP  " << orig_mat_prev[14] << endl;
cout << "OMT  " << orig_mat[14] << endl;
cout << "G2OD " << glob2odo[14] << endl;
cout << "MATT " << target->transMat[14] << endl;
cout << "MATP " << source->transMat[14] << endl;
  */
  unsigned int counter = 0;
  unsigned int max_counter = 9;  // number of point pairs that "anchor" the scan
  

//  cout << glob2odo <<  "  " << orig_mat << " " << orig_mat_prev << " : " << mat_prev << endl; 

  for (unsigned int i = 0; i < 100; i++) {
    double s[3], t[3];
    t[0] = s[0] = (rand() / (double)RAND_MAX) * 100.0;
    t[1] = s[1] = (rand() / (double)RAND_MAX) * 100.0;
    t[2] = s[2] = (rand() / (double)RAND_MAX) * 100.0;
//    cout << t[0] << " " << s[0] << endl;
    
//    ::transform(s, glob2odo);
    ::transform(s, source->transMat);
    ::transform(s, odo);
    ::transform(t, target->transMat);
    PtPair myPair(s, t);
    pairs->push_back(myPair);
    centroid_m[0] += s[0];  
    centroid_m[1] += s[1];
    centroid_m[2] += s[2];
    centroid_d[0] += t[0];
    centroid_d[1] += t[1];
    centroid_d[2] += t[2];
  }

  for (unsigned int i = 0; i < max_counter/3; i++) {
    double s[3], t[3];
    t[0] = s[0] = 1.0; 
    t[1] = s[1] = 0.0;
    t[2] = s[2] = 0.0;
    
//    ::transform(s, glob2odo);
    ::transform(s, source->transMat);
    ::transform(s, odo);
    ::transform(t, target->transMat);
    PtPair myPair(s, t);
    pairs->push_back(myPair);
    centroid_m[0] += s[0];  
    centroid_m[1] += s[1];
    centroid_m[2] += s[2];
    centroid_d[0] += t[0];
    centroid_d[1] += t[1];
    centroid_d[2] += t[2];
  }
  for (unsigned int i = 0; i < max_counter/3; i++) {
    double s[3], t[3];
    t[0] = s[0] = 0.0; 
    t[1] = s[1] = 1.0;
    t[2] = s[2] = 0.0;
    
//    ::transform(s, glob2odo);
    ::transform(s, source->transMat);
    ::transform(s, odo);
    ::transform(t, target->transMat);
    PtPair myPair(s, t);
    pairs->push_back(myPair);
    centroid_m[0] += s[0];  
    centroid_m[1] += s[1];
    centroid_m[2] += s[2];
    centroid_d[0] += t[0];
    centroid_d[1] += t[1];
    centroid_d[2] += t[2];
  }
  for (unsigned int i = 0; i < max_counter/3; i++) {
    double s[3], t[3];
    t[0] = s[0] = 0.0; 
    t[1] = s[1] = 0.0;
    t[2] = s[2] = 1.0;

//    ::transform(s, glob2odo);
    ::transform(s, source->transMat);
    ::transform(s, odo);
    ::transform(t, target->transMat);
    PtPair myPair(s, t);
    pairs->push_back(myPair);
    centroid_m[0] += s[0];  
    centroid_m[1] += s[1];
    centroid_m[2] += s[2];
    centroid_d[0] += t[0];
    centroid_d[1] += t[1];
    centroid_d[2] += t[2];
  }

  centroid_m[0] /= pairs->size();
  centroid_m[1] /= pairs->size();
  centroid_m[2] /= pairs->size();
  centroid_d[0] /= pairs->size();
  centroid_d[1] /= pairs->size();
  centroid_d[2] /= pairs->size();

  return;
}

  
void LScan::transformToEuler(double rP[3], double rPT[3], const Scan::AlgoType type, int islum ) {
  // TODO this is called by lum6dEuler

  EulerToMatrix4(rP, rPT, transMat);

/*  double tinv[16];
  double alignxf[16];
  M4inv(transMat, tinv);
  transform(tinv, Scan::INVALID, -1);
  EulerToMatrix4(rP, rPT, alignxf);
  transform(alignxf, type, islum);*/
}


void LScan::resetTransform() {
  double rPosTheta[3], rPos[3];
  Matrix4ToEuler(repscan->orig_transMat, rPosTheta, rPos);
  EulerToMatrix4(rPos, rPosTheta, transMat);

  for (unsigned int i = 0; i < scans.size(); i++) {
    scans[i]->resetTransform();
  }

}


void LineScan::getPtPairs(vector <PtPair> *pairs, LineScan *Source, LineScan* Target, int thread_num, int rnd, double max_dist_match2, double *centroid_m, double *centroid_d) {

  // check if anything should be done
  if ( ((int)Source->index - (int)maxdist <= (int)Target->index && (int)Target->index <= (int)Source->index - (int)mindist) ||
       ((int)Source->index + (int)mindist <= (int)Target->index && (int)Target->index <= (int)Source->index + (int)maxdist) ) {
  
    Mcorr::iterator corrs = Target->correspondences.find(Source->index);
    if (corrs == Target->correspondences.end() ) return;
    
    vlppair &corpts = corrs->second;

    // for each corresponding point
    for (unsigned int i = 0; i < corpts.size(); i++) {
      double *s = corpts[i].first;
      double *t = corpts[i].second;

      PtPair myPair(s, t);
      pairs->push_back(myPair);

      centroid_d[0] += t[0];
      centroid_d[1] += t[1];
      centroid_d[2] += t[2];
      centroid_m[0] += s[0];  
      centroid_m[1] += s[1];
      centroid_m[2] += s[2];

    }


  }
}

void LineScan::findPtPairs(int i, double max_dist_match2) {
  if (i < 0 || i >= (int)allLineScans.size())
    return; 

  vector<lppair> vpairs;

  LineScan *target = allLineScans[i];
  for (int ii = 0; ii < target->nrpts; ii++) {
    double *p = target->points[ii];

    double *closest = 0;

    double mdm2 = max_dist_match2;
    for (int j = 0; j < this->nrpts; j++) {
      double *q = this->points[j];
      double dist = Dist2(p, q);

      if (dist < mdm2) {
        closest = q;
        mdm2 = dist;
      }
    }
    if (closest) {
      vpairs.push_back(lppair(closest, p) );
    }
  }
  if (!vpairs.empty()) {
    correspondences[i] = vpairs;
  }

}

void LineScan::findPtPairs(double max_dist_match2) {
  for (int i = (int)(this->index) - (int)maxdist; i <= (int)(this->index) - (int)(mindist); i++) {
    findPtPairs(i, max_dist_match2);
//    cout << "  _ " << i << endl;
  }
  for (int i = (int)this->index + (int)mindist; i <= (int)this->index + (int)maxdist; i++) {
    findPtPairs(i, max_dist_match2);
//    cout << "   " << i << endl;
  }
}

void LineScan::computePtPairs(double max_dist_match2) {
omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel for schedule(dynamic)
  for (unsigned int i = 0; i < allLineScans.size(); i++) {
    allLineScans[i]->findPtPairs(max_dist_match2);
    cout << i << endl;
    
  }
}
void LineScan::computePtPairsOct(double max_dist_match2) {
  LBOctTree<double> *tree = new LBOctTree<double>(allLineScans, 10.0);
  cout << "tree created" << endl;
  int count = 0;

  vector<double*> query_pts;
  cout << "get random" << endl;
  srandom(0);
  tree->GetOctTreeRandom(query_pts);
//  srandom(0);
//  tree->GetOctTreeRandom(query_pts, 5);
  for (unsigned int i = 0; i < allLineScans.size(); i++) {
    allLineScans[i]->clearQuery();
  }
  cout << "add queries" << endl;
  for (unsigned int i = 0; i < query_pts.size(); i++) {
    int index = query_pts[i][3];
    allLineScans[index]->addQuery(query_pts[i]);

  }

/*  vector<double*> pts;
  tree->AllPoints(pts);
  for (int i = 0; i < pts.size(); i++)
    cerr << pts[i][0] << " " <<pts[i][1] << " " <<pts[i][2] << endl; 
  exit(0);*/
  cout << "find closest points" << endl;
  for (unsigned int i = 0; i < allLineScans.size(); i++) {
    int r = tree->FindClosestPoints(allLineScans[i], max_dist_match2, LineScan::mindist, LineScan::maxdist, 0);
//    cout << i << " with " << r << " out of " << allLineScans[i]->nrpts << endl;
    count += r;
  }
  //printPtPairs();
  //exit(0);

  
  //cout << "found " << count << " ptpairs " << endl; 

  cout << "deleting  tree" << endl;
  delete tree;
//  exit(0);
}

void LineScan::addPtPair(lppair &pair, unsigned int index) {
  correspondences[index].push_back(pair);
}

void LineScan::printPtPairs() {
  for (unsigned int l = 0; l < allLineScans.size(); l++) {
    Mcorr &correspondences = allLineScans[l]->correspondences;
    for (Mcorr::iterator it = correspondences.begin(); it!=correspondences.end(); it++) {
//      if (l >= 900 || it->first >= 900) continue;
/*      if (! it->second.empty()) {
        cerr << l << " " << it->first << endl;
      }*/
      for (unsigned int i = 0; i < it->second.size(); i++) {
        double *p = it->second[i].first;
        double *q = it->second[i].second;
//        cerr << p[3] << " " << q[3] 
        cerr << p[0] << " " << p[1] << " " << p[2] << " " << q[0] << " " << q[1] << " " << q[2] 
        //<< " " << l << " " << it->first  
          << endl;
      }
      
    }
  }
}

void LineScan::clearPtPairs() {

  for (Mcorr::iterator it = correspondences.begin(); it!=correspondences.end(); it++) {
    for (unsigned int i = 0; i < it->second.size(); i++) {
      double *q = it->second[i].second;
      double *p = it->second[i].first;
      delete[] q;
      delete[] p;
    }
  }

  correspondences.clear();
}
