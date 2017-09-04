#include "slam6d/point_type.h"
#include "slam6d/globals.icc"
#include "slam6d/scan.h"
#include "slam6d/icp6Dquat.h"
#include "slam6d/icp6D.h"
#include "slam6d/graphSlam6D.h"
#include "slam6d/lum6Deuler.h"
#include "simplereg.h"
#include "gapx6D.h"
#include "graphSlam6DL.h"
#include "lum6Deuler.h"
#include "ghelix6DQ2.h"
#include "linescan.h"
#include "continuousreg.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#include <iomanip>
#include <iostream>
#include <fstream>
#include <queue>
using std::queue;
#include <vector>
using std::vector;
#include <sstream>
using std::stringstream;
using std::string;
using std::cout;
using std::endl;
using std::ifstream;

#include <getopt.h>

#include <signal.h>


//  Handling Segmentation faults and CTRL-C
void sigSEGVhandler (int v)
{
  static bool segfault = false;
  if(!segfault) {
    segfault = true;
    cout << endl
      << "# **************************** #" << endl
      << "  Segmentation fault or Crtl-C" << endl
      << "# **************************** #" << endl
      << endl
      << "Saving registration information in .frames files" << endl;
    for (unsigned int i = 0; i < LineScan::allLineScans.size(); i++) {
      if (i % 2) cout << ".";
      cout.flush();
      delete LineScan::allLineScans[i];
    }
    cout << endl;
  }
  exit(-1);
}


void usage(char* prog)
{
  const string bold("\033[1m");
  const string normal("\033[m");
  cout << endl
    << bold << "USAGE " << normal << endl
    << "   " << prog << " [options] directory" << endl << endl;
  cout << bold << "OPTIONS" << normal << endl

  << bold << "  -R, --reflectance, --reflectivity" << normal << endl
  << "         write reflectivity values " << endl
  << endl
  << bold << "  -A, --amplitude" << endl << normal
  << "         write amplitude values " << endl
  << endl
  << bold << "  -D, --deviation" << endl << normal
  << "         write deviation values " << endl
  << endl
  << bold << "  -d" << normal << " NR, " << bold << "--dist=" << normal << "NR   [default: 10]" << endl
  << "         sets the maximal point-to-point distance for matching to <NR> 'units'" << endl
  << "         (unit of scan data, e.g. cm)" << endl

  << endl << endl;

  exit(1);
}

void parseArgs(int argc,char **argv, string &dir, int &start, int &end, unsigned int &types, double &red, int &octree, double &mdm, int &mni, int &interval, int &size, int &slamiterations, bool &prereg) {

  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  static struct option longopts[] = {
    { "start",           required_argument,   0,  's' },
    { "end",             required_argument,   0,  'e' },
    { "prereg",          no_argument,         0,  'p' },
    { "reflectance",     no_argument,         0,  'R' },
    { "reflectivity",    no_argument,         0,  'R' },
    { "amplitude",       no_argument,         0,  'A' },
    { "deviation",       no_argument,         0,  'D' },
    { "reduce",          required_argument,   0,  'r' },
    { "octree",          optional_argument,   0,  'O' },
    { "dist",            required_argument,   0,  'd' },
    { "iter",            required_argument,   0,  'i' },
    { "SLAMiter",            required_argument,   0,  'I' },
    { "interval",            required_argument,   0,  'N' },
    { "size",            required_argument,   0,  'S' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  while ((c = getopt_long(argc, argv,"N:S:I:r:O:gw:ofLli:d:s:e:Rp", longopts, NULL)) != -1)
    switch (c)
   {
   case 's':
     start = atoi(optarg);
     if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
     break;
   case 'e':
     end = atoi(optarg);
     if (end < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
     if (end < start) { cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
     break;
   case 'p':
     prereg = true; 
     break;
   case 'R':
     types |= PointType::USE_REFLECTANCE;
     break;
   case 'A':
     types |= PointType::USE_AMPLITUDE;
     break;
   case 'D':
     types |= PointType::USE_DEVIATION;
     break;
   case 'r':
     red = atof(optarg);
     break;
   case 'O':
     if (optarg) {
       octree = atoi(optarg);
     } else {
       octree = 1;
     }
     break;
   case 'd':
     mdm = atof(optarg);
     break;
   case 'i':
     mni = atoi(optarg);
     break;
   case 'I':
     slamiterations = atoi(optarg);
     break;
   case 'N':
     interval = atoi(optarg);
     break;
   case 'S':
     size = atoi(optarg);
     break;
   default:
     usage(argv[0]);
     break;
   }

  if (optind != argc-1) {
    cerr << "\n*** Directory missing ***" << endl;
    usage(argv[0]);
  }
  dir = argv[optind];

  if (dir[dir.length()-1] != '/') dir = dir + "/";
}






int main(int argc, char** argv) {

  signal (SIGSEGV, sigSEGVhandler);
  signal (SIGINT,  sigSEGVhandler);


  string basedir;
  int start = 0;
  int end = -1;
  unsigned int types = PointType::USE_NONE;
  double red = -1.0;
  int octree = 1;
  double mdm = 10.0;
  int iterations = 10;
  int slamiterations = 15;

  int interval = 100;
  int size = 100;

  bool prereg = false;

  parseArgs(argc, argv, basedir, start, end, types, red, octree, mdm, iterations,  interval, size, slamiterations, prereg);

  PointType pointtype = PointType(types);


  vector<LineScan*>  linescans;
    
  string scanFileName;
  string poseFileName;


//  cout << "reading scans..." << endl;
  for (int i =  start; i <= end; i++) {
    LineScan *ls = new LineScan();

    if (ls->readFromFile(i, basedir, pointtype) ) {
      linescans.push_back(ls);
    } else {
      cout << "Read scans " << start << " to " << i << endl;
      break;
    }
  }

//  int interval = 100;        // all 100 linescans we create a 3d scan
  int width = interval;  //  left and right number of linescans in each 3d scan
  int mindist = 360;         //  minimum number of linescans where a match is possible 
  int maxdist = 1080;         //  maximum number of linescans where a match is possible 

  icp6Dminimizer *my_icp6Dminimizer = new icp6D_QUAT(false);
  icp6D *icp = new icp6D(my_icp6Dminimizer, mdm, 300, true, false, 1, true, -1, 0.00000001);

///////////////////////////////////////////////////////
  int preinterval = 720;
  int parts = linescans.size() / preinterval;
  ///////////////   sequential registration
  if (prereg) {
    for (unsigned int i = 0; i < parts; i++) {
      int index = i * preinterval; // current linescan that we wish to match and correct with previous linescans

//      for (unsigned int iters = 0; iters < iterations; iters++) {
        preRegistration(linescans, 
            index + preinterval +1, index + 2* preinterval,  
            //index, size, 
            index, index + preinterval,
            //index-size-mindist - 720, index-size - mindist, 
            icp, red, octree);
      }
//    }
    for (int i = 0; i < linescans.size(); i++)  {
      linescans[i]->dumpFrames();
    }
  }
  /*
  for (unsigned int i = 0; i <= parts; i++) {
    int index = i * preinterval; // current linescan that we wish to match and correct with previous linescans
    cout << "register " << index << " (+- " << size << ")  against " << index-size-mindist - 720 << " to " << index-size - mindist << endl;

    if ( index-size-mindist < 400) continue;

    for (unsigned int iters = 0; iters < iterations; iters++) {
      preRegistration(linescans, index, size, index-size-mindist - 720, index-size - mindist, icp);
    }
  }
  for (int i = 0; i < linescans.size(); i++)  {
    linescans[i]->dumpFrames();
  }*/
  /////////////////




  ////////// global semi rigid registration  // mdm was 5 for some reason?
//  graphSlam6DL *luml = new simpleRegZ(my_icp6Dminimizer, mdm, mdm);
 // graphSlam6DL *luml = new simpleReg(my_icp6Dminimizer, mdm, mdm);
  graphSlam6DL *lml = new gapx6DL(my_icp6Dminimizer, mdm, mdm);
  graphSlam6DL *luml = new lum6DEulerL(my_icp6Dminimizer, mdm, mdm);
 // graphSlam6DL *luml = new ghelix6DQ2L(my_icp6Dminimizer, mdm, mdm);
  luml->set_quiet(true);


 ///////////////////

  ////////// global registration
  { 
  graphSlam6D *my_graphSlam6D = new lum6DEuler(my_icp6Dminimizer, mdm, mdm);
  my_graphSlam6D->set_quiet(true);

  Graph *gr = 0;
  
  ////////////////////////// first frame
  double id[16];
  M4identity(id);
  for (int i = 0; i < linescans.size(); i++)  {
    linescans[i]->transform(id, Scan::ICP, 1 );
    linescans[i]->transform(id, Scan::ICP, 1 );
  }
  
  double odomweight = 0.05 * sqr(LineScan::allLineScans.size() );
  ///////////////////////

  for (unsigned int iters = 0; iters < iterations; iters++) {
    gr = 0;
    luml->setOdomWeight(odomweight);
    //Registration(linescans, my_graphSlam6D, gr, 0, 100, 500, 360, red, octree ); 
    double pd = SSRR(linescans, my_graphSlam6D, luml, gr, 0, 
        slamiterations, 
        500,  // clpairs
        mdm, 
        interval,  // scan interval
        size,  // scan size
        red, octree ); 
    cout << "Matching with SLAM and distributing error... " << iters << endl;

    for (int i = 0; i < linescans.size(); i++)  {
      linescans[i]->dumpFrames();
    }
/*
    if (pd < 0.1 ) {
      odomweight *= 0.1;
      mdm *= 0.90;
      my_graphSlam6D->set_mdmll(mdm);

      cout << "Reducing Odometry weight to " << odomweight << endl;
      cout << "Reducing maximal distance match to " << mdm << endl;
    }
    if (odomweight < 10000.0) {
      cout << "very small odometry weight..." << endl;
      interval /=2;
      odomweight = 10.0 * sqr(LineScan::allLineScans.size() );
//      break;
    if (interval < 10 )
      break;
    }*/
  }
  LineScan::setWriteFrames(true);
  for (int i = 0; i < linescans.size(); i++) {
    delete linescans[i];
  }
 return 0; 
  }
  ///////////

  
  Graph *gr = 0;

  LineScan::setMinDist(360);
  LineScan::setMaxDist(1020);

  cout << "Semi rigid Matching with SLAM and distributing error... with " << iterations << " iterations" << endl;
  
  SemiRigidRegistration(linescans, luml, gr, iterations, 1, // slamiterations
      mdm, // miniml distance
/*
      5, // scaninterval
      360, // scansize
  */    
//      5, 2, 
//      10, 50,
//      60, 29,
//      60, 40,
//      100, 50,
//      150, 70,
//      300, 150,
//      600, 290,
//        800, 200,      
//        400, 200,
        50, 250,        
        //100, 10,      
      red, octree ); 
     
  ///////////
/*  for (unsigned int scandist = linescans.size()/2; scandist >= 50; scandist /= 2) {
    gr = 0;
    mdm *= 0.7;
  SemiRigidRegistration(linescans, luml, gr, iterations, 1, // slamiterations
      mdm, // miniml distance

      scandist, // scaninterval
      300, // scansize
      
//      600, 250,
      red, octree ); 
  }*/


/////////////////////////  for the correction test, i.e. scans 5500 - 6500
/*  iterations=10;
  for (unsigned int i = 0; i < iterations; i++) {
    preRegistration(linescans, 800, 150, 0, 300, icp);
  }*/
 // outputScans(linescans, 0, linescans.size()-1, "scan001.3d");
///////////////////////////

  LineScan::setWriteFrames(true);
  for (int i = 0; i < linescans.size(); i++) {
    delete linescans[i];
  }









//////// transform every point to global coordinate system

//  cout << "good" << endl;
/*
  double *transMat;
  double tinv[16];
  transMat = linescans[0]->transMat;
  M4inv(transMat, tinv);

  for (unsigned int i = 0; i < 100; i++)  {
    transMat = linescans[i]->transMat;
    for (int j = 0; j < linescans[i]->nrpts; j++) {
      transform(linescans[i]->points[j], transMat);
      transform(linescans[i]->points[j], tinv);

      cout << linescans[i]->points[j][0] << " " << linescans[i]->points[j][1] << " " << linescans[i]->points[j][2] << endl;
    }

  }
*/

//  double matbegin[16] = {0.751787, 0.00932798, 0.65934, 0, -0.0110208, 0.999938, -0.00158055, 0, -0.659314, -0.00607821, 0.751843, 0, -125.348, -0.185222, 618.558, 1};
//  double *matbegin = (*(linescans.end()-1))->transMat;
////////////////
/*
  double *matbegin = linescans[700]->transMat;
  double matend[16] = {0.719216, 0.011557, 0.69469, 0, -0.0131426, 0.999909, -0.00302807, 0, -0.694662, -0.00695217, 0.719303, 0, -128.627, 0.0643921, 617.499, 1};

  double rPb[3], rPTb[3];
  double rPe[3], rPTe[3];

  Matrix4ToEuler(matbegin, rPTb, rPb);
  Matrix4ToEuler(matend, rPTe, rPe);
  
  double dP[6]; 
  dP[0] = rPb[0] - rPe[0];
  dP[1] = rPb[1] - rPe[1];
  dP[2] = rPb[2] - rPe[2];
  dP[3] = rPTb[0] - rPTe[0];
  dP[4] = rPTb[1] - rPTe[1];
  dP[5] = rPTb[2] - rPTe[2];


  double *transMat;
  double tinv[16];
  transMat = linescans[0]->transMat;
  M4inv(transMat, tinv);

  for (unsigned int i = 0; i < linescans.size(); i++)  {
    transMat = linescans[i]->transMat;
    Matrix4ToEuler(transMat, rPTb, rPb);

//    cout << "scan " << i << " " << transMat << endl;
    if (i <= 1000) {
      rPb[0] = rPb[0] - ( (double)i / (double) (linescans.size()-1) ) * dP[0];
      rPb[1] = rPb[1] - ( (double)i / (double) (linescans.size()-1) ) * dP[1];
      rPb[2] = rPb[2] - ( (double)i / (double) (linescans.size()-1) ) * dP[2];
      rPTb[0] = rPTb[0] - ( (double)i / (double) (linescans.size()-1) ) * dP[3];
      rPTb[1] = rPTb[1] - ( (double)i / (double) (linescans.size()-1) ) * dP[4];
      rPTb[2] = rPTb[2] - ( (double)i / (double) (linescans.size()-1) ) * dP[5];
    } else {
      rPb[0] = rPb[0] - dP[0];
      rPb[1] = rPb[1] - dP[1];
      rPb[2] = rPb[2] - dP[2];
      rPTb[0] = rPTb[0] - dP[3];
      rPTb[1] = rPTb[1] - dP[4];
      rPTb[2] = rPTb[2] - dP[5];
    }
    EulerToMatrix4(rPb, rPTb, transMat);

//    cout << "scan " << i << " " << transMat << endl;
    for (int j = 0; j < linescans[i]->nrpts; j++) {
      transform(linescans[i]->points[j], transMat);
      transform(linescans[i]->points[j], tinv);

      cout << linescans[i]->points[j][0] << " " << linescans[i]->points[j][1] << " " << linescans[i]->points[j][2] << endl;
    }

  }


*/
 
}
