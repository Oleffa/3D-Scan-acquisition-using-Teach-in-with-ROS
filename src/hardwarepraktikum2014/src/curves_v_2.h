#ifndef __MY_CURVES_H
#define __MY_CURVES_H__

#include <math.h>
#include <iostream>
#include <fstream>

#define CN_NOT_ACT -2

using namespace std;

class CCurve{
  protected:
    double *tx;
    double *ty;
    double A, B, C;

    int count;
    int cur;
    int add_c;
    int c_err;

    void recompute_coeffs(int i){
      A = ty[i+1]-ty[i];
      B = tx[i]-tx[i+1];
      C = -A*tx[i]-B*ty[i];
    }

  public:
    CCurve(int num = 0):count(num), add_c(0){
      if(count<=0){
        count = 0;
        c_err = 1;
        cur = -1;
      }else{
        tx = new double[count];
        ty = new double[count];
        c_err = 0;
        cur = -1;
      }
    }

    ~CCurve(){
      if(!c_err){
        delete [] tx;
        delete [] ty;
      }
    }

    int WriteToFile(const char* fname){
      ofstream pth;
      int i;
      if(c_err){
        return 0;
      }

      pth.open(fname);
      if(!pth)
        return 0;

      for(i = 0;i<count;i++)
        pth<<tx[i]<<" "<<ty[i]<<endl;

      pth.close();
      return 1;
    }
	
	//Loads a file with name fname
	//checks if file is valid etc and also reads the number of datasets from the file
int LoadFromFile(const char* fname){
	ifstream file1;
	char buff[255];
	int i = 0;
// set count to 0 before reading each file
	count = 0;


	if(file1.is_open()){cout << "file open already" << endl;}
	file1.open(fname);
    if(!file1){
		file1.close();
		cout << "ERROR: File not found!" << endl;
        return 0;
    }
	//old text, was reding first line as number of datasets
    //file1>>count;
	//now count lines, so adding the number of datasets is obsolete
	//TODO is count == 0??
	string line;
	
	while(getline(file1, line)){
		count = count + 1;
	}
	//cout << "Number of Datasets found: " << count << endl;
	
    file1.getline(buff,255);
	//If the file has ended return 0 and end the process
    if(file1.eof()){
		file1.close();
        return 0;
    }

    if(!c_err){
		delete [] tx;
        delete [] ty;
        //count = 0;
        c_err = 1;
        cur = -1;
    };

    tx = new double[count];
    ty = new double[count];
    do{
		//read the x and y position from the path file
		tx[i] = ty[i] = nan("NAN");
        file1>>tx[i];
        file1>>ty[i];
        file1.getline(buff,255);
		//if all data has been sucessfully read
        if(!isnan(tx[i]) && !isnan(ty[i])) {
			//tx[i] *= 0.01;
			//ty[i] *= 0.01;
			i++;
        }else {
			break;
        }
    }
	while(!file1.eof() && i<count);
    if(i!=count){
		return 0;
	}
    c_err = 0;
	cout << "File" << fname << "loaded succefully" << endl;
    return (count!=0);
};

double Evaluate(double x, double y){
	if(c_err) return nan("NAN");
	return A*x+B*y+C;
}

    //return tan(dy/dx)
    double getAng(){
		if(c_err || (cur == -1)) {
			return nan("NAN");
		}
    return atan2(ty[cur+1]-ty[cur],tx[cur+1]-tx[cur]);
    }

    double getTan(){
      if(c_err || (cur == -1))
        return nan("NAN");

      return tan(getAng());
    }

    int initTraversal(){
      if(c_err){
        return 0;
      }

      cur = 0;
      return 1;
    }

    //looped = Anzahl Schleifendurchgaenge
    int getNext(int looped = 0){
      static int ft = 1;
      static int cnt = 0;
      int ret_val = 1;

      if(c_err || (cur == -1))
        return 0;

      switch(cur) {
        case 0:
          if(ft==1){
            ft = 0;
          }else{
            cur++;
          }
          break;
        default:
          if(cur<count-2){
            cur++;
          }else{
            if(looped){
              cur = 0;
              cnt++;
              if(cnt == looped){
                ret_val = 0;
              };
            }else{
              ret_val = 0;
            }
          }
      };

      if(ret_val){
        recompute_coeffs(cur);
      }else{
        ft = 1;
        cur = -1;
        cnt = 0;
      }

      return ret_val;
    }

    int getPrev(int looped = 0){
      static int ft = 1;
      int ret_val = 1;

      if(c_err || (cur == -1))
        return 0;

      switch(cur) {
        case 0:
          (ft==1)?(ft = 0):(cur++);
          break;
        default:
          if(cur>0){
            cur--;
          }else{
            (looped)?(cur = count-2):(ret_val = 0);
          }
      };

      if(ret_val){
        recompute_coeffs(cur);
      }else{
        ft = 1;
        cur = -1;
      }

      return ret_val;
    }

    int addNextPoint(double x, double y){
      if(!c_err){
        if(add_c<count){
          tx[add_c] = x;
          ty[add_c] = y;
          add_c++;
          return 1;
        }else
          return 0;
      }else
        return 0;
    }

    int getSegmentNumber(){
      if(c_err){
        return -1;
      }

      return cur;
    }

    int pointInn(double x, double y){
      double a1, b1, c1, c2, c3;
	//cout << "pointInn Test" <<endl;
      if(c_err){
        return 0;
      }
	//cout << "pointInn OK" <<endl;
      a1 = tx[cur+1]-tx[cur];
      b1 = ty[cur+1]-ty[cur];
      c1 = -a1*tx[cur] - b1*ty[cur];
      c2 = -a1*tx[cur+1]-b1*ty[cur+1];
      c3 = -a1*x-b1*y;

      if(c2>=c1){
        if(c3>=c1)
          return 1;
        else
          return 0;
      }else{
        if(c2<=c1){
          if(c3>=c2)
            return 1;
          else
            return 0;
        }else
          return 0;
      }
    }

    double getDistance(double x, double y){
      double res;
      if(c_err){
        return nan("NAN");
      }

      res = fabs(A*x+B*y+C)/sqrt(A*A+B*B);
      return res;
    }

    inline int getCount(){
      return this->count;
    }

    double getDistanceToEnd(double x, double y){
      double t1, t2, res;

      if(c_err){
        return nan("NAN");
      }

      t1 = this->getDistance(x,y);
      t2 = (tx[cur+1]-x)*(tx[cur+1]-x)+(ty[cur+1]-y)*(ty[cur+1]-y);
      res = sqrt(t2-t1*t1);
      return res;
    }
};

#endif
