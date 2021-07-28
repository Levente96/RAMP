/**
   Copyright 2020 Levente Csóka
**/
#include "ramp.h"
#include <Wire.h>
#include <H3LIS331DL.h>
#include "wiring_private.h"

const int       MIN_VALUE                = -32768;       ///< Minimum value of unsigned int
TwoWire         Wire2(&sercom1, 11, 13);                 ///< Separate i2c for h3lis
H3LIS331DL      h3lis;                                   ///< Handler of the Accelerometer

void acquireData(data_t* data)
{
    data->time = millis();
    int16_t x = MIN_VALUE;
    int16_t y = MIN_VALUE;
    int16_t z = MIN_VALUE;
    int16_t gx = MIN_VALUE;
    int16_t gy = MIN_VALUE;
    int16_t gz = MIN_VALUE;
    int16_t lx = MIN_VALUE;
    int16_t ly = MIN_VALUE;
    int16_t lz = MIN_VALUE;
    Wire.beginTransmission(0x18);
    Wire.write(0x28);
    Wire.endTransmission();
    Wire.requestFrom(0x18, 6);
    x = Wire.read() << 8 | Wire.read();
    y = Wire.read() << 8 | Wire.read();
    z = Wire.read() << 8 | Wire.read();
    Wire.endTransmission();
    
    Wire2.beginTransmission(0x68);
    Wire2.write(0x43);
    Wire2.endTransmission();
    Wire2.requestFrom(0x68, 6);
    gx = Wire2.read() << 8 | Wire2.read();
    gy = Wire2.read() << 8 | Wire2.read();
    gz = Wire2.read() << 8 | Wire2.read();
    Wire2.endTransmission();
    /*Wire2.beginTransmission(0x68);
    Wire2.write(0x3B);
    Wire2.endTransmission();
    Wire2.requestFrom(0x68, 6);
    lx = Wire2.read() << 8 | Wire2.read();
    ly = Wire2.read() << 8 | Wire2.read();
    lz = Wire2.read() << 8 | Wire2.read();
    Wire2.endTransmission();*/

    data->lx = lx;
    data->ly = ly;
    data->lz = lz;
    data->gx = gx;
    data->gy = gy;
    data->gz = gz;
    data->x = x;
    data->y = y;
    data->z = z;
}

// Sensor setup
void rampSetup()
{
    Wire.begin();
    Wire.setClock(400000L);
    Wire2.begin();
    pinPeripheral(11, PIO_SERCOM);
    pinPeripheral(13, PIO_SERCOM);
    Wire2.setClock(400000L);
    Serial.println("Setting up MPU...");
    Wire2.beginTransmission(0x68);
    Wire2.write(0x6B);
    Wire2.write(0x00);
    Wire2.endTransmission();
    

    Serial.println("Setting up H3LIS...");
    h3lis.init(H3LIS331DL_ODR_1000Hz, H3LIS331DL_NORMAL, H3LIS331DL_FULLSCALE_2);
    h3lis.importPara(0, 0, 0);
    Serial.println("Sensor setup finished...");
}
