/**
 * Sick LMS100 driver
 *
 * Adapted from Stefan Stiene, Institute of Computer Science, University of Osnabrueck.
 * Jan Elseberg, Smart Systems Group, Jacobs University Bremen gGmbH, Germany.
 */

#include <stddef.h>
#include <sys/socket.h>
#include <netdb.h>
#include "lms100_cola.h"
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <ctime>
#include <stdlib.h>

#include <stdio.h>
#include <fcntl.h>

using namespace std;



// ------------- methods for connecting from lms111-code --------------- //
int has_to_stop;

int _kbhit() {
    static const int STDIN = 0;
    static int initialized = 0;

    if (! initialized) {
        // Use termios to turn off line buffering
        struct termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = 1;
    }

    int bytesWaiting;
    ioctl(STDIN, FIONREAD, &bytesWaiting);
    return bytesWaiting;
}

void sigquit_handler(int q __attribute__((unused))){
    has_to_stop=1;
}
// --------------------------------------------------------------------- //

////////////////////////////////////////////////////////////////////////////////
// Constructor.
lms100_cola::lms100_cola (const char* _hostname, int _portno, int _verbose ) {
    portno   = _portno;
    hostname = _hostname;
    verbose  = _verbose;
    memset (command, 0, BUF_SIZE);

    nr_measurements = 541;
}

////////////////////////////////////////////////////////////////////////////////
// Connect to the LMS400 unit using hostname:portno
// Returns 0 if connection was successful, -1 otherwise
int
lms100_cola::Connect ()
{
    // Create a socket
    sockfd = socket (AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0)
        return (-1);

    // Get the network host entry
    server = gethostbyname ((const char *)hostname);
    if (server == NULL)
        return (-1);

    // Fill in the sockaddr_in structure values
    bzero ((char *) &serv_addr, sizeof (serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port   = htons (portno);
    bcopy ((char *)server->h_addr,
           (char *)&serv_addr.sin_addr.s_addr,
           server->h_length);

    // Attempt to connect
    if (connect (sockfd, (const sockaddr*)&serv_addr, sizeof (serv_addr)) < 0)
        return (-1);

    return (0);

}

////////////////////////////////////////////////////////////////////////////////
// Disconnect from the LMS400 unit
// Returns 0 if connection was successful, -1 otherwise
int
lms100_cola::Disconnect ()
{
    return (close (sockfd));
}

////////////////////////////////////////////////////////////////////////////////
// Enable/Disable extended RIS (Remission Information System) detectivity
int
lms100_cola::EnableRIS (int onoff)
{
    char cmd[40];
    sprintf (cmd, "sWN MDblex %i", onoff);
    SendCommand (cmd);

    if (ReadAnswer () != 0)
        return (-1);
    ExtendedRIS = onoff;
    return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Set the mean filter parameters
int
lms100_cola::SetMeanFilterParameters (int num_scans)
{
    char cmd[40];
    sprintf (cmd, "sWN FLmean 0 %i", num_scans);
    SendCommand (cmd);

    if (ReadAnswer () != 0)
        return (-1);
    MeanFilterNumScans = num_scans;
    return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Set the range filter parameters
int
lms100_cola::SetRangeFilterParameters (float *ranges)
{
    char cmd[40];
    sprintf (cmd, "sWN FLrang %+f %+f", ranges[0], ranges[1]);
    SendCommand (cmd);

    if (ReadAnswer () != 0)
        return (-1);
    RangeFilterBottomLimit = ranges[0];
    RangeFilterTopLimit    = ranges[1];
    return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Enable filters using a filter mask
int
lms100_cola::EnableFilters (int filter_mask)
{
    char cmd[40];
    sprintf (cmd, "sWN FLsel %+i", filter_mask);
    SendCommand (cmd);

    if (ReadAnswer () != 0)
        return (-1);
    FilterMask = filter_mask;
    return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Takes a string containing an ip adress and returns an array of 4 u_chars
unsigned char*
lms100_cola::ParseIP (char* ip)
{
    char* tmp = (char*) malloc (strlen (ip) + 1);
    unsigned char* _ip = (unsigned char*) malloc (4);

    strcpy (tmp, ip);
    _ip[0] = atoi (strtok (tmp, "."));
    for (int i = 1; i < 4; i++)
        _ip[i] = atoi (strtok (NULL, "."));

    free (tmp);
    return _ip;
}

////////////////////////////////////////////////////////////////////////////////
// Set the desired userlevel by logging in with the appropriate password
int
lms100_cola::SetUserLevel (int8_t userlevel, const char* password)
{
    char cmd[255];
    sprintf (cmd, "sMN SetAccessMode %d %s", userlevel, password);
    SendCommand (cmd);
    return (ReadConfirmationAndAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Fills string pointed to by macadress with the MAC adress read from the sensor
int
lms100_cola::GetMACAddress (char** macaddress)
{
    char *mac = (char*) malloc (20);
    int index = 0;
    char* tmp;

    SendCommand ("sRN EImac ");
    if (ReadAnswer () != 0)
        return (-1);

    strtok ((char*) buffer, " ");
    strtok (NULL, " ");

    for (int i = 0; i < 6; i++)
    {
        tmp = strtok (NULL, "-");
        strncpy (&mac[index], tmp, 2);
        index += 2;
        mac[index++] = ':';
    }

    mac[--index] = 0;
    *macaddress = mac;
    return (0);
}

////////////////////////////////////////////////////////////////////////////////
// Set the IP address of the LMS400
int
lms100_cola::SetIP (char* ip)
{
    unsigned char* ip_str;
    ip_str = ParseIP (ip);
    char cmd[80];

    sprintf (cmd, "sWN EIip %X %X %X %X", ip_str[0], ip_str[1], ip_str[2], ip_str[3]);
    free (ip_str);
    SendCommand (cmd);

    return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Set the gateway address for the Ethernet interface
int
lms100_cola::SetGateway (char* gw)
{
    unsigned char* gw_str;
    gw_str = ParseIP (gw);
    char cmd[80];

    sprintf (cmd, "sWN EIgate %X %X %X %X", gw_str[0], gw_str[1], gw_str[2], gw_str[3]);
    free (gw_str);
    SendCommand (cmd);

    return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Set the subnet mask for the Ethernet interface
int
lms100_cola::SetNetmask (char* mask)
{
    unsigned char* mask_str;
    mask_str = ParseIP (mask);
    char cmd[80];

    sprintf (cmd, "sWN EImask %X %X %X %X", mask_str[0], mask_str[1], mask_str[2], mask_str[3]);
    free (mask_str);
    SendCommand (cmd);

    return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Set port for TCP/IP communication
int
lms100_cola::SetPort (uint16_t port)
{
    char cmd[80];

    sprintf (cmd, "sWN EIport %04X", port);
    SendCommand (cmd);

    return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Reset the LMS400 unit
int
lms100_cola::ResetDevice ()
{
    const char* cmd = "sMN mDCreset ";
    SendCommand (cmd);

    return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Terminate configuration and change back to userlevel 0
int
lms100_cola::TerminateConfiguration ()
{
    const char* cmd = "sMN Run";
    SendCommand (cmd);

    return (ReadConfirmationAndAnswer ());
}
////////////////////////////////////////////////////////////////////////////////
// Set both resolution and frequency without going to a higher user level (?)
#include <ros/ros.h>
int
lms100_cola::SetResolutionAndFrequency (float freq, float ang_res,
                                        float angle_start, float angle_stop)
{
    nr_measurements = (angle_stop - angle_start) / ang_res + 1; 
    char cmd[80];
    sprintf (cmd, "sMN mLMPsetscancfg %f 1 %f %f %f",
             freq, ang_res, angle_start, angle_stop); // sMN mLMPsetscancfg
    SendCommand (cmd);

    int error = ReadAnswer ();

    /*
    // If no error, parse the results
    if (error == 0)
    {
        strtok ((char*)buffer, " ");
        strtok (NULL, " ");
        int ErrorCode = strtol (strtok (NULL, " "), NULL, 10);

        if ((ErrorCode != 0) && (verbose))
            printf (">> Warning: got an error code %d\n", ErrorCode);
    }
    */

    return (error);
}

////////////////////////////////////////////////////////////////////////////////
// Start a measurement for both distance and intensity or just distance.
int
lms100_cola::StartMeasurement (bool intensity)
{

    SendCommand ("sMN LMCstartmeas");

    return (ReadAnswer ());
}


////////////////////////////////////////////////////////////////////////////////
// Stop a measurement
int
lms100_cola::StopMeasurement ()
{

    SendCommand ("sMN LMCstopmeas");

    return (ReadAnswer ());
}

////////////////////////////////////////////////////////////////////////////////
// Send a command to the laser unit. Returns -1 on error.
int
lms100_cola::SendCommand (const char* cmd)
{
    assemblecommand ((unsigned char *) cmd, strlen (cmd));

    n = write (sockfd, command, commandlength);
    if (n < 0)
        return (-1);
    if (verbose)
    {
        printf (">> Sent: \"%s\" %d %u\n", cmd, commandlength, (unsigned int)strlen(cmd));
    }
    return (0);
}

void lms100_cola::wait() 
{
  char cmd[80];
  sprintf (cmd, "sMN mLMPsetscancfg %f 1 %f %f %f",
      50.0, 0.5, -45.0, 225.0); // sMN mLMPsetscancfg  // keep sending default setup

  int flags = fcntl(sockfd, F_GETFL);
  fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

  char onechar;
  // Look for STX
  while(true)
  {
    SendCommand (cmd);
    onechar = 0x00;
    n = read (sockfd, &onechar, 1);

    if (onechar == 0x02) // found a STX
    {
      break;
    }
    // wait half a second
    printf("Sick is not ready yet, wait a while...\n");
    usleep(500000);
  }


  fcntl(sockfd, F_SETFL, flags & ~O_NONBLOCK);
  printf("Sick is ready \n");
}

////////////////////////////////////////////////////////////////////////////////
// Read a result from the laser unit.
int
lms100_cola::ReadResult ()
{

    char onechar;
    // Look for STX
    while(true)
    {
        onechar = 0x00;


        n = read (sockfd, &onechar, 1);


        if (n < 0)
        {
            cout << "-11" << endl;
            return (-1);
        }

        if (onechar == 0x02) // found a STX
        {
            break;
        }
        //cout << onechar << endl;
    }

    int current = 0;
    // Read data packet and look for ETX
    /*
    while(true)
    {
        onechar = 0x00;
        n = read (sockfd, &onechar, 1);
        if (n < 0)
        {
            return (-1);
        }
        if (n == 1)
        {
            buffer[current] = onechar;
            current++;
        }
        if (onechar == 0x03) // found a ETX
        {
            buffer[current] = 0x00;
            break;
        }
    }*/

    char mybuf[3000];
    int EL = 0;
    while(true)
    {
        onechar = 0x00;
        n = read (sockfd, mybuf, 3000);
//printf("%d   %d\n",n, current);
        if (n < 0)
        {
            return (-1);
        }
        if (n > 0)
        {
          memcpy(buffer+current, mybuf, n);
          
          // check for end signal
          for (int i = n-1; i >= 0; i--)
            if (mybuf[i] == 0x03) {
              EL = i+1;            
              break;
            }
          // terminate string
          if (EL != 0) {
            buffer[current+EL] = 0x00;
            break;
          }
          // not done with message yet
          current += n;
        }
    }

    return (0);

}

////////////////////////////////////////////////////////////////////////////////
// Read an answer from the laser unit
int
lms100_cola::ReadAnswer ()
{
    return ReadResult ();
}

////////////////////////////////////////////////////////////////////////////////
// Read a confirmation and an answer from the laser unit
int
lms100_cola::ReadConfirmationAndAnswer ()
{
    ReadResult ();
    if (buffer[0] == 's' && buffer[1] == 'F' && buffer[2] == 'A')
        return (-1);
    else
        return ReadResult ();
}


void lms100_cola::SaveResult (FILE* file)
{
    fprintf(file,"%s\n",buffer);
}

////////////////////////////////////////////////////////////////////////////////
// adds a header and the checksum to the command to be sent
int
lms100_cola::assemblecommand (unsigned char* cmd, int len)
{

    //unsigned char checksum = 0;
    int index = 0;

    command[0]  = 0x02;  // Messages start with 4 STX's

    for (index = 0; index < len; index++)
    {
        command[index+1]  = cmd[index];
    }
    command[1 + len] = 0x03;
    command[2 + len] = 0x00;

    commandlength = 2 + len;
    return (0);

}


int lms100_cola::MainSetup()
{

    SendCommand ("sMN LMCstartmeas");
    SendCommand("sEN LMDscandata 1");

    return(0);
}

void lms100_cola::MainQuit()
{
    SendCommand("sEN LMDscandata 0");
    Disconnect();
}

void lms100_cola::ParseResult() {

    char buf[8];
    int pos = 0;
    int ranges_count = 0;
    for(unsigned int i = 118; i < strlen((char*)buffer); i++) {
        if(buffer[i] == ' '){
            if(ranges_count < nr_measurements){
                measurements[ranges_count++] =
                    static_cast<double>((strtol((const char*)buf, NULL, 16))) / 1000.0;
            }
            pos = 0;
            memset(buf,0,8);
        }
        else buf[pos++] = buffer[i];
    }
}
int lms100_cola::getNRMeas() {
  return nr_measurements;
}

double* lms100_cola::getMeas() {
  return measurements;
}


