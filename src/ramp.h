#ifndef UserTypes_h
#define UserTypes_h
#include "Arduino.h"

struct data_t {
    uint32_t time;    ///<Time of recording
    int16_t x;        ///<Acceleration toward X
    int16_t y;        ///<Acceleration toward Y
    int16_t z;        ///<Acceleration toward Z
    int16_t lx;       ///<Acceleration toward X
    int16_t ly;       ///<Acceleration toward Y
    int16_t lz;       ///<Acceleration toward Z
    int16_t gx;       ///<Gyro on X
    int16_t gy;       ///<Gyro on Y
    int16_t gz;       ///<Gyro on Z
};

void acquireData(data_t* data);
void rampSetup();
#endif  // UserTypes_h
