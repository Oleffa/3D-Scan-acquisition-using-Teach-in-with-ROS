#include "slam6d/point_type.h"
#include "slam6d/globals.icc"
#include "slam6d/scan.h"
#include "slam6d/icp6Dquat.h"
#include "slam6d/icp6D.h"
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
  << bold << "  -a, --amplitude" << endl << normal
  << "         write amplitude values " << endl
  << endl
  << bold << "  -d, --deviation" << endl << normal
  << "         write deviation values " << endl
  << endl

  << endl << endl;

  exit(1);
}

void parseArgs(int argc,char **argv, string &dir, int &start, int &end, int &reference, unsigned int &types) {

  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  static struct option longopts[] = {
    { "reference",           required_argument,   0,  'r' },
    { "start",           required_argument,   0,  's' },
    { "end",             required_argument,   0,  'e' },
    { "reflectance",     no_argument,         0,  'R' },
    { "reflectivity",    no_argument,         0,  'R' },
    { "amplitude",       no_argument,         0,  'A' },
    { "deviation",       no_argument,         0,  'D' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  while ((c = getopt_long(argc, argv,"gw:ofLld:s:e:r:R", longopts, NULL)) != -1)
    switch (c)
   {
   case 'r':
     reference = atoi(optarg);
     if (reference < 0) { cerr << "Error: Cannot use a negative scan number as reference.\n"; exit(1); }
     break;
   case 's':
     start = atoi(optarg);
     if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
     break;
   case 'e':
     end = atoi(optarg);
     if (end < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
     if (end < start) { cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
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

  string basedir;
  int reference = 0;
  int start = -1;
  int end = -1;
  unsigned int types = PointType::USE_NONE;


  parseArgs(argc, argv, basedir, start, end, reference, types);

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
//      cout << "stopping at scan " << i << endl;
      break;
    }
  }


  LineScan *ls = new LineScan();
  ls->readFromFile(reference, basedir, pointtype);

  outputScans(linescans, 0, linescans.size()-1, "scan000.3d", ls->transMat); 

 
}
