#include "continuousreg.h"
#include "slam6d/graphSlam6D.h"
#include "slam6d/Boctree.h"
#include "lboctree.h"
#include "lum6Deuler.h"
#include "ghelix6DQ2.h"
#include "slam6d/basicScan.h"


/**
 * Creates a slam6D scan from the linescans begin-end, with the pose of index as reference coordinate system
 *
 */
Scan *joinLines(vector<LineScan*> &linescans, int begin, int end, int &index, double voxelsize, int nrpts)
{
  if (begin < 0) begin = 0;
  if (end >= (int)linescans.size()) end = linescans.size() -1;
  if (index < 0) index = 0;
  if (index >= (int)linescans.size()) index = linescans.size() -1;

  cout << "Create Scan between " << begin << " and " << end << "  with " << index << " as rep." << endl;
  vector<double *> pts;
 
  // transform points into coordinate system of index and store copies in pts
  double *transMat;
  double tinv[16];
  double rPos[3], rPosTheta[3];

  transMat = linescans[index]->transMat;
  Matrix4ToEuler(transMat, rPosTheta, rPos);
  M4inv(transMat, tinv);

  for (int i = begin; i <= end; i++) {
    transMat = linescans[i]->transMat;
    for (int j = 0; j < linescans[i]->nrpts; j++) {
      // TODO optimize
      double *p = new double[3];
      p[0] = linescans[i]->points[j][0];
      p[1] = linescans[i]->points[j][1];
      p[2] = linescans[i]->points[j][2];

      transform(p, transMat);
      transform(p, tinv);
      pts.push_back(p);
    }
  }


  if (voxelsize > 0.0) {
    vector<double*> tmp = pts;
    BOctTree<double> *oct = new BOctTree<double>(tmp, voxelsize);
    vector<double*> reduced;
    reduced.clear();
    if (nrpts > 0) {
      if (nrpts == 1) {
        oct->GetOctTreeRandom(reduced);
      }else {
        oct->GetOctTreeRandom(reduced, nrpts);
      }
    } else {
      oct->GetOctTreeCenter(reduced);
    }

    for (unsigned int i = 0; i < pts.size(); i++) {
      delete[] pts[i];
    }
    pts.clear();

    vector<double*> reduced2;
    for (unsigned int i = 0; i < reduced.size(); i++) {
      double *p = new double[3];
      p[0] = reduced[i][0];
      p[1] = reduced[i][1];
      p[2] = reduced[i][2];
      reduced2.push_back(p);
    }
    reduced.clear();

    delete oct;
    return new BasicScan(rPos, rPosTheta, reduced2);
  }

  return new BasicScan(rPos, rPosTheta, pts);
}


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
void preRegistration(vector<LineScan*> &linescans, 
    int learliest, int llatest, 
    int fearliest, int flatest, 
    icp6D *icp, double voxelsize, int nr_octpts) {

  // representative linescan for first scan
  int findex = fearliest + (flatest-fearliest)/2;
  int lindex = learliest + (llatest-learliest)/2;

  Scan *first = joinLines(linescans, fearliest, flatest, findex, voxelsize, nr_octpts); 
  Scan *last = joinLines(linescans, learliest, llatest, lindex, voxelsize, nr_octpts); 
//  Scan *last = joinLines(linescans, index-width, index+width, index); 


  vector<Scan*> scans;
  scans.push_back(first);
  scans.push_back(last);

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 0; i < (int)scans.size(); i++) {
      scans[i]->setSearchTreeParameter(BOCTree);
      scans[i]->createSearchTree();
    }

  cout << "Doing icp..." << endl;
  icp->doICP(scans); // align scans to minimize odometry related errors
  
  cout << "DISTRIBUTE " << findex << " to " << lindex << endl;

  linearDistributeError(linescans, findex, lindex, last->get_transMat()); 

  // TODO delete Scans
  delete first;
  delete last;
}



/**
 * Does slam6d matching with 3D scans 
 *
 * @param earliest is the index of the earliest linescan to match against
 * @param latest is the index of the latest linescan to match against
 *
 * Essentially we create sequential scans, that are rigidly matched using slam6d
 * the differences in the pose estimates of the scans is then distributed along the linescans in between
 *
 */
void Registration(vector<LineScan*> &linescans, graphSlam6D *slam, Graph *&gr, icp6D *icp, int slamiters, int clpairs, int scansize, double voxelsize, int nr_octpts) 
{
  vector<Scan*> scans;
  vector<int> scan_reps;

  // TODO implement automatic sccan segmentation on basis of theta parameter 
  if (scansize <= 0) {
  
  } else {
    int nr_scans = linescans.size() / scansize;

    for (int i = 0; i < nr_scans; i++) {
      int begin = i * scansize;
      int end = begin + scansize - 1;
      int representative = begin + scansize/2;
      Scan *s = joinLines(linescans, begin, end, representative, voxelsize, nr_octpts); 
      scans.push_back(s);
      scan_reps.push_back(representative);

    }

    if (nr_scans * scansize < (int)linescans.size()) {
      int begin = nr_scans*scansize;
      int end = linescans.size() -1; 
      int representative = begin + (end-begin)/2; 

      Scan *s = joinLines(linescans, begin, end, representative, voxelsize, nr_octpts); 
      scans.push_back(s);
      scan_reps.push_back(representative);
      
    }
  }


#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
  for (int i = 0; i < (int)scans.size(); i++) {
    scans[i]->setSearchTreeParameter(BOCTree);
    scans[i]->createSearchTree();
  }


  if (icp)
    icp->doICP(scans); // align scans to minimize odometry related errors
  //slam->matchGraph6Dautomatic(scans, slamiters, clpairs, 0);

  
  if (gr == 0) {
    gr = slam->computeGraph6Dautomatic(scans, clpairs);
  }

  for (int i = 0; i < slamiters; i++) {
    cout << i << endl;
    double ret = slam->doGraphSlam6D(*gr, scans, 1);
    if (ret < 0.001) {
      break;
    }
  }


  // reset tranformations...
  for (unsigned int i = 0; i < linescans.size(); i++)  {
//    linescans[i]->resetTransform();
  }

/*  for (unsigned int i = 0; i < scans.size(); i++) {
    int first = scan_reps[i];
    for (unsigned int j = 0; j < 16; j++)
      cout << "tmat[" << i << "][" << j << "] = " << scans[i]->get_transMat()[j] << ";" << endl;
  }*/

  for (unsigned int i = 1; i < scans.size(); i++) {
    int first = scan_reps[i-1];
    int last = scan_reps[i];

    linearDistributeError(linescans, first, last, scans[i]->get_transMat(), true); 
   // cout << "distribute " << first << " - " << last << " with scan " << i << " transmat " << endl;
  }

  cout << "deleting scans" << endl;
/*  for (unsigned int s = 0; s < scans.size(); s++) {
    string fname = "scan" + to_string(s+1, 3) + ".3d";
    ofstream fout(fname.c_str());
    for (unsigned int i = 0; i < scans[s]->get_points_red_size(); i++)  {
      fout << scans[s]->get_points_red()[i][0] << " " << scans[s]->get_points_red()[i][1] << " " << scans[s]->get_points_red()[i][2] << endl;
    }
  }*/

  // TODO delete Scans
  for (unsigned int i = 0; i < scans.size(); i++)
    delete scans[i];
  scans.clear();
  scan_reps.clear();
 
  // reset tranformations...
  //for (unsigned int i = 0; i < linescans.size(); i++)  {
  //  linescans[i]->resetTransform();
  //}

//  outputScans(linescans, 0, index); 

}




/**
 * Does slam6d matching with 3D scans 
 *
 * @param earliest is the index of the earliest linescan to match against
 * @param latest is the index of the latest linescan to match against
 *
 * Essentially we create sequential scans, that are rigidly matched using slam6d
 * the differences in the pose estimates of the scans is then distributed along the linescans in between
 *
 */
void SemiRigidRegistration(vector<LineScan*> &linescans, graphSlam6DL *slam, Graph *&gr, int iterations, int slamiters, double mdm, int scaninterval, int scansize, double voxelsize, int nr_octpts) 
{
  //////////////////////////
  double id[16];
  M4identity(id);
  for (int i = 0; i < linescans.size(); i++)  {
    linescans[i]->transform(id, Scan::ICP, 1 );
    linescans[i]->transform(id, Scan::ICP, 1 );
  }
  ///////////////////////
  vector<LScan*> scans;

  int nr_scans = linescans.size() / scaninterval + 1;


  for (int i = 0; i < nr_scans; i++) {
    int representative = i * scaninterval; 
    int begin =  representative - scansize;
    int end = representative + scansize; 
    cout << "create scan " << begin << " to " << end << " with " << representative << endl; 
    LScan *s = new LScan(begin, end, representative); 
    scans.push_back(s);
    ////////////////////
/*    string filename = (string)"scan" + to_string(i, 3) + ".frames";
    string fn = "scan" + to_string(i, 3) + ".3d";
    cout << s->get_transMat() << endl;
    outputScans(linescans, begin, end, fn.c_str(),  s->get_transMat()); 
    ofstream fout(filename.c_str(), std::ios::app);
    for (unsigned int j = 0; j < 3; j++) {
      fout << s->get_transMat() << " 2" << endl;
    }
    fout.close();
    fout.clear();*/
    ////////////////////
  }

//  LScan *s1 = scans[0];
//  LScan *s2 = scans[5];
 /* 
  scans.clear();
  scans.push_back(s1);
  scans.push_back(s2);
  */


  cout << "done with scans" << endl;


  // create slam graph
  if (gr == 0) {
    gr = new Graph();
    for (unsigned int i = 0; i < scans.size(); i++) {
      for (unsigned int j = 0; j < scans.size(); j++) {
        if (i==j) {
          continue;
        }
        if (i > j ) continue;
//        if (j > i + 1) continue;
//        if (i > 0 && j > 0) continue;
//        if (i > 0) continue;
//        if (j > 0) continue;
        gr->addLink(i, j);
        cout << i << " " << j << endl;
      }
    }
  }
  cout << "done with graph" << endl;


  for (int iters = 0; iters < iterations; iters++) {
////////////////
/*
    string fn1 = "scan" + to_string(iters,0) + ".pose";
    ofstream fout1(fn1.c_str());
    fout1 << linescans[s2->getRepresentative()]->rPos[0] << " " << linescans[s2->getRepresentative()]->rPos[1] << " " << linescans[s2->getRepresentative()]->rPos[2] << endl;
    fout1 << deg(linescans[s2->getRepresentative()]->rPosTheta[0]) << " " << deg(linescans[s2->getRepresentative()]->rPosTheta[1]) << " " << deg(linescans[s2->getRepresentative()]->rPosTheta[2]) << endl;
    fout1.close();
    cout << s2->getRepresentative() << " " << linescans[s2->getRepresentative()]->rPos[0] 
      <<" "<< linescans[500]->rPos[0] <<  endl;
*/

    string fn = "trajectoryI" + to_string(iters,1) + ".txt";
    ofstream fout(fn.c_str());
  for (unsigned int i = 0; i < linescans.size(); i++)  {
    fout << i << " " << linescans[i]->rPos[2] << endl;
   // fout << linescans[i]->rPos[0] << " " << linescans[i]->rPos[1] << " " << linescans[i]->rPos[2] << endl;
  }
  fout.close();
  /**/
////////////////



    cout << "Iteration " << iters << " of semi rigid matching" << endl;
  cout << "finding ptpairs" << endl;

  LineScan::clearAllPtPairs();
//  LineScan::computePtPairs(5.0);
  LineScan::computePtPairsOct(mdm*mdm);
  cout << "done with finding ptpairs" << endl;

  string fnc = "testcorr" + to_string(iters ,3) + ".3d";
//  LScan::writePtPairs(scans[0], scans[1], 0, mdm*mdm, fnc.c_str());
  LScan::writeAllPtPairs(scans, 0, mdm*mdm, fnc.c_str());
  /*
  ofstream foutc(fnc.c_str());
  for (int j = 0; j < LineScan::allLineScans.size(); j++) { 
//  for (int j = s2->getBegin(); j < s2->getEnd(); j++) { 
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
      foutc << s[0] << " " << s[1] << " " << s[2] << " " << t[0] << " "<< t[1] << " " << t[2] << endl;
    }
  }
  foutc.close();*/

    for (int i = 0; i < slamiters; i++) {
      cout << i << endl;
      double ret = slam->doGraphSlam6D(*gr, scans, 1, LineScan::allLineScans);
//      double ret = slam->doGraphSlam6D(*gr, scans, 1);
      if (ret < 0.001) {
        break;
      }
    }

    // reset tranformations...
    for (unsigned int i = 0; i < linescans.size(); i++)  {
//      linescans[i]->resetTransform();
    }
/*    
    for (unsigned int i = 0; i < scans.size(); i++) {
      string filename = (string)"scan" + to_string(i, 3) + ".frames";
      ofstream fout(filename.c_str(), std::ios::app);
      for (unsigned int j = 0; j < 3; j++) {
        fout << scans[i]->get_transMat() << " 2" << endl;
      }
      fout.close();
      fout.clear();
    }
    */
    // the following is for applying the transformations that have been calculated for the LScans to the underlying linescans
  /*  
    // transform scans before the first LScan's representative
    for (int j = 0; j <= scans[0]->getRepresentative() ; j++)  {
      double id[16];
      M4identity(id);
      linescans[j]->transform(id, Scan::ICP, 1 );
    }

//cout << "S2 " <<     s2->get_transMat() << endl;
    for (unsigned int i = 1; i < scans.size(); i++) {
      int first = scans[i-1]->getRepresentative();
      int last = scans[i]->getRepresentative();
      linearDistributeError(linescans, first, last, scans[i]->get_transMat(), true); 
//      linearDistributeError(linescans, first, last, LineScan::allLineScans[last]->ground_truth_mat, true); 
    }
    // transform scans after the last LScan's representative
    for (int j = scans[scans.size() - 1]->getRepresentative() + 1; j < LineScan::allLineScans.size(); j++)  {
      double id[16];
      M4identity(id);
      linescans[j]->transform(id, Scan::ICP, 1 );
    }
  */


    /*
    double id[16];
    M4identity(id);
    for (int i = 0; i <= scans[1]->getBegin() -1; i++)  {
      linescans[i]->transform(id, Scan::ICP, 1 );
    }
    
    cout << "apply transforms" << endl;
    for (unsigned int i = 1; i < scans.size(); i++) {
      int begin = scans[i]->getBegin();
      int end = scans[i]->getEnd();
      int rep = scans[i]->getRepresentative();
      const double *transmatnew = scans[i]->get_transMat();
      double *transmatold = linescans[rep]->transMat;

      /// compute transformation to apply to scans
      double transMatDIFF[16];
      double tinv2[16];
      M4inv(transmatold, tinv2);
      MMult(transmatnew, tinv2, transMatDIFF);
      cout << transmatold << endl;
      cout << transMatDIFF << endl;
      cout << linescans[rep]->transMat << endl;
      // done
      for (int j = begin; j <= end; j++)  {
        linescans[j]->transform(transMatDIFF, Scan::ICP, 1 );
      }
      cout << linescans[rep]->transMat << endl;
      cout << transmatnew << endl;
    }
    */
  

  }

  cout << "deleting scans" << endl;

  // TODO delete Scans
  for (unsigned int i = 0; i < scans.size(); i++)
    delete scans[i];
  scans.clear();

  delete gr;
  
/*  for (unsigned int i = 0; i < linescans.size(); i++)
    delete linescans[i];
  linescans.clear();*/
 
  // reset tranformations...
  //for (unsigned int i = 0; i < linescans.size(); i++)  {
  //  linescans[i]->resetTransform();
  //}

//  outputScans(linescans, 0, index); 

}





double SSRR(vector<LineScan*> &linescans, graphSlam6D *slam, graphSlam6DL *slaml, Graph *&gr, icp6D *icp, int slamiters, int clpairs, double mdm, int scaninterval, int scansize, double voxelsize, int nr_octpts) 
{

  vector<LScan*> lscans;
  vector<Scan*> sscans;
  vector<int> scan_reps;
  int nr_scans = linescans.size() / scaninterval + 1;

  for (int i = 0; i < nr_scans; i++) {
    int representative = i * scaninterval; 
    int begin =  representative - scansize;
    int end = representative + scansize; 
    //cout << "create scan " << begin << " to " << end << " with " << representative << endl; 
    LScan *s = new LScan(begin, end, representative); 
    lscans.push_back(s);
      
    Scan *ss = joinLines(linescans, begin, end, representative, voxelsize, nr_octpts); 
    sscans.push_back(ss);
    scan_reps.push_back(representative);
      
    //string fn = "/tmp/scan" + to_string(lscans.size()-1, 3) + ".3d";
    //outputScans(linescans, begin, end, fn.c_str(), linescans[representative]->transMat);
  }

#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
  for (int i = 0; i < (int)lscans.size(); i++) {
    sscans[i]->setSearchTreeParameter(BOCTree);
    sscans[i]->createSearchTree();
  }


  if (icp)
    icp->doICP(sscans); // align scans to minimize odometry related errors

  
  if (gr == 0) {
    gr = slam->computeGraph6Dautomatic(sscans, clpairs);
  }

  for (int i = 0; i < slamiters; i++) {
    /*
    for (unsigned int j = 0; j< sscans.size(); j++) {
      string fn = "/tmp/scan" + to_string(j, 3) + ".frames";
      ofstream of(fn.c_str(), std::ios_base::app);
      of << sscans[j]->get_transMat() << " 2" << endl;

      of.close();
    }*/
    cout << i << ": ";
    double ret = slam->doGraphSlam6D(*gr, sscans, 1);
    if (ret < 0.001) {
      break;
    }
  }


  // reset tranformations...
  for (unsigned int i = 0; i < linescans.size(); i++)  {
//    linescans[i]->resetTransform();
  }

  cout << "distributing errors..." << endl;
/*  for (unsigned int j = 0; j< sscans.size(); j++) {
    double rPos[3], rPosTheta[3];
      for (int k = 0; k < 3; k++) {
        rPos[k]      = sscans[j]->get_rPos()[k];
        rPosTheta[k] = sscans[j]->get_rPosTheta()[k];
      }
    lscans[j]->transformToEuler(rPos, rPosTheta, Scan::LUM, 1);
  }*/
/*
  LineScan::clearAllPtPairs();
//  LineScan::computePtPairs(5.0);
  LineScan::computePtPairsOct(mdm*mdm);
  cout << "done with finding ptpairs" << endl;
  */

  double ret = slaml->doGraphSlam6D(*gr, lscans, 1, LineScan::allLineScans, &sscans);

  //////////////////////////////////////
  // print some information
  slam->doGraphSlam6D(*gr, sscans, 1);
  double tm[16];
  for (unsigned int i = 0; i < sscans.size(); i++)  {
    string fname = "scan" + to_string(i, 3) + ".frames";
    ofstream ofile;
    ofile.open(fname.c_str(), ios::app);
    ofile << sscans[i]->get_transMat() << endl;
    ofile.close();
  }
  //////////////////////////////////////

  cout << "deleting scans" << endl;

  // TODO delete Scans
  for (unsigned int i = 0; i < sscans.size(); i++)
    delete sscans[i];
  sscans.clear();
  scan_reps.clear();
  
  for (unsigned int i = 0; i < lscans.size(); i++)
    delete lscans[i];
  lscans.clear();

  return ret/(LineScan::allLineScans.size()); 
}
