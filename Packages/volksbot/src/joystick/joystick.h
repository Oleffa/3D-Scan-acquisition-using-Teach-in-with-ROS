#ifndef JOYSTICK_HH
#define JOYSTICK_HH
#include "stdio.h"
#include <unistd.h>
#include <stdlib.h>
#include <fcntl.h>
#include <stdint.h>

#define BUTTON_TYPE 1 
#define AXIS_TYPE 2

#define JS_MAX_VALUE 32768;
#define JS_MIN_VALUE -32767;


struct js_event {
  uint32_t time;     /* event timestamp in milliseconds */
  int16_t value;     /* value */
  uint8_t type;      /* event type */
  uint8_t number;    /* axis/button number */
};


class Joystick {

  public:
    Joystick(const char *filename);
    Joystick();
    ~Joystick();
    
    void waitforevents();
    void waitforevent();

    virtual void handleButton(uint8_t number, bool pressed, uint32_t time) = 0;
    virtual void handleAxis(uint8_t number, int16_t value, uint32_t time) = 0;
    
  private:
    struct js_event je;
    void init(const char *filename);
    int fd;
};

#endif
