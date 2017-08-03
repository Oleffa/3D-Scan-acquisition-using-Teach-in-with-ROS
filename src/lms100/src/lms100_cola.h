#ifndef __LMS100_H__
#define __LMS100_H__


#include <netdb.h>
#include <sys/types.h>
#include <vector>
#include <netinet/in.h>
#include <iostream>
#include <termios.h>
#include <sys/ioctl.h>


#define BUF_SIZE 1024

////////////////////////////////////////////////////////////////////////////////
class lms100_cola
{
  public:
    lms100_cola (const char* _hostname, int _portno, int _verbose);

    void wait();

    int MainSetup();
    void MainQuit();

    // Creates socket, connects
    int Connect ();
    int Disconnect ();

    void SaveResult (FILE* file);

    // Configuration parameters
    int SetResolutionAndFrequency (float freq, float ang_res, float angle_start, float angle_stop);

    int StartMeasurement (bool intensity = true);
    int StopMeasurement  ();

    int SetUserLevel  (int8_t userlevel, const char* password);
    int GetMACAddress (char** macadress);

    int SetIP         (char* ip);
    int SetGateway    (char* gw);
    int SetNetmask    (char* mask);
    int SetPort       (uint16_t port);

    int ResetDevice            ();
    int TerminateConfiguration ();

    int SendCommand   (const char* cmd);
    int ReadResult    ();
    // for "Variables", Commands that only reply with one Answer message
    int ReadAnswer    ();
    // for "Procedures", Commands that reply with a Confirmation message and an Answer message
    int ReadConfirmationAndAnswer ();

    int EnableRIS (int onoff);
    int SetMeanFilterParameters (int num_scans);
    int SetRangeFilterParameters (float *ranges);
    int EnableFilters (int filter_mask);

    // turns a string holding an ip address into long
    unsigned char* ParseIP (char* ip);

    void ParseResult();

    int getNRMeas();
    double* getMeas();
  private:
    // assembles STX's, length field, message, checksum ready to be sent. Cool.
    int assemblecommand (unsigned char* command, int len);

    const char* hostname;
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    // Internal Parameters:
    int verbose;
    int ExtendedRIS;
    int MeanFilterNumScans;
    float RangeFilterTopLimit;
    float RangeFilterBottomLimit;
    int FilterMask;

    // for reading:
    unsigned char buffer[4096*16];

    // for sending:
    unsigned char command[BUF_SIZE];
    int commandlength;

    double measurements[1081]; // maximal number of measurements
    int nr_measurements;//(int)ceil((data_packet.max_angle - data_packet.min_angle)/data_packet.resolution);

};

#endif
