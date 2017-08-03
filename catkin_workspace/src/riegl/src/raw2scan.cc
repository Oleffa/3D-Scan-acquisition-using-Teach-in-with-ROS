#include <riegl/scanlib.hpp>

#include <iomanip>
#include <iostream>
#include <exception>
#include <cmath>
#include <limits>

#include <string.h>
// You might need to adjust the below include path, depending on your
// compiler setup and which tr1 implementation you are using.
#if defined(_MSC_VER)
#   include <memory>
#else
#   include <tr1/memory>
#endif

using namespace scanlib;
using namespace std;
using namespace std::tr1;

#include <stdlib.h>


//#define BSIZE 16777216
#define BSIZE 5100


class importer
    : public pointcloud
{
    ofstream o;
    
    char buffer[BSIZE];
    char *pos;

    char dir[1024];

public:
    importer(const char *_dir)
        : pointcloud(false) // set this to true if you need gps aligned timing
         
    {
      count = 0;
      strcpy(dir, _dir);
      openstream();
    }


    ~importer() {
      o.close();
    }

private:

    void openstream() {
      char fname[1024];
      sprintf(fname, "%s/scan%03d.3d", dir, count);
      cout << "Opening file: " << fname << endl ;
      o.open(fname);
      o << std::setprecision(10);
      pos  = buffer;
    }

protected:
    int count;

    void on_frame_stop(const frame_stop<iterator_type>& arg) {
      cout << "findished scan " << count  << endl;
      pointcloud::on_frame_stop(arg);
      count++;
      cout << "Closing crrent file. "  << endl ;
      o.close();
      openstream();
    }
    
    // overridden from pointcloud class
    void on_echo_transformed(echo_type echo)
    {
      target& t(targets[target_count - 1]);
      sprintf(pos, "%16.8f %16.8f %16.8f\n", (t.vertex[1] * -100.0), (t.vertex[2]*100.0), (t.vertex[0]*100.0));
      pos += 51; // 16 + 1 + 16 + 1 + 16 + 1
      if (pos + 51 > buffer + BSIZE) // next write would overflow
      {
        o.write(buffer, (pos - buffer));
        pos = buffer;
      }

      //o.write(buffer, );
      //o << (t.vertex[0] * 100.0) << " " << (t.vertex[2]*100.0) << " " << (t.vertex[1]*100.0) << endl;
    }

    // overridden from basic_packets
    // this function gets called when a the scanner emits a notification
    // about an exceptional state.
    void on_unsolicited_message(const unsolicited_message<iterator_type>& arg) {
        basic_packets::on_unsolicited_message(arg);
        // in this example we just print a warning to stderr
        cerr << "WARNING: " << arg.message << endl;
        // the following line would put out the entire content of the packet
        // converted to ASCII format:
        // cerr << arg << endl;
    }
};

int main(int argc, char* argv[])
{
    try {

        if (argc == 3) {
            shared_ptr<basic_rconnection> rc;

            string filename = "file:";
            filename = filename + argv[1];
            rc = basic_rconnection::create(filename.c_str());
            rc->open();
            
            decoder_rxpmarker dec(rc);
            importer     imp(argv[2]);
            buffer       buf;

            for ( dec.get(buf); !dec.eoi(); dec.get(buf) ) {
                imp.dispatch(buf.begin(), buf.end());
            }

            rc->close();
            return 0;
        }

        cerr << "Usage: " << argv[0] << " <filename of the rxp to convert>   <directory>" << endl;
        return 1;
    }
    catch(exception& e) {
        cerr << e.what() << endl;
        return 1;
    }
    catch(...) {
        cerr << "unknown exception" << endl;
        return 1;
    }

    return 0;
}
