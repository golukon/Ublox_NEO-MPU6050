/* ============================================
  UBLOX_NEO-MPU6050 code is placed under the MIT license
  Copyright (c) 2023 golukon

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

#include "Simple_MPU6050_modif.h"
#include "SparkFun_u-blox_GNSS_Arduino_Library_modif.h"

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

//               X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
#define OFFSETS  -1012,    2,    1897,      86,     45,     -9

// constants
#define CONST_G 9.80655f
#define CONST_PI 3.14159265
#define TIME_MPU_CALIBRATE 20000  //ms
#define SQR_AMPL_ACCEL_GPS_MIN 0.09f
#define MUL_NEW_ACCEL_ACCURACY 1.04f
#define POS_GPS_ACCURACY 150000 //mm
#define SPEED_GPS_ACCURACY 3000 //mm/s
#define MAX_ACCEL_ACCURACY 2.0f
#define DEC_ACCEL_ACCURACY 1.1f
#define PERIOD_ACCEL_ACCURACY 60000 //ms
#define PERIOD_CALC_ACCEL 6000 //ms


Simple_MPU6050 mpu;
SFE_UBLOX_GNSS GNSS;
Quaternion MpuToGps;
VectorFloat aWorldMpu;
uint32_t lasttime, timeincaccu, timequatGPSMPU;
int32_t lastvn, lastve, lastvd, lastlat, lastlon, lastaltit, lastsubh, sumGPS[3] = {0, 0, 0};
float accurmax, sumMPU[3] = {0.0f, 0.0f, 0.0f};
bool enabledUart, firstsr;

//uint32_t countaccel = 0;      //accur calibrate
//float sumaccel[3] = {0, 0, 0}, sumgyro[3] = {0, 0, 0};

/*             _________________________________________________________*/

//***************************************************************************************
//******************                Print Functions                **********************
//***************************************************************************************

#define spamtimer(t) for (static uint32_t SpamTimer; (uint32_t)(millis() - SpamTimer) >= (t); SpamTimer = millis()) // (BLACK BOX) Ya, don't complain that I used "for(;;){}" instead of "if(){}" for my Blink Without Delay Timer macro. It works nicely!!!

void GetWorldAccel(int16_t *gyro, int16_t *accel, int32_t *quat, uint16_t SpamDelay = 100) {
  spamtimer(SpamDelay) {// non blocking delay before printing again. This skips the following code when delay time (ms) hasn't been met
    if (millis() < TIME_MPU_CALIBRATE) return;
    Quaternion q;
    VectorFloat gravity;
    VectorInt16 aa, aaReal, aaWorld;
    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.SetAccel(&aa, accel);
    mpu.GetLinearAccel(&aaReal, &aa, &gravity);
    mpu.GetLinearAccelInWorld(&aaWorld, &aaReal, &q);

    aWorldMpu.x = aaWorld.x / 16384.0f * CONST_G;
    aWorldMpu.y = aaWorld.y / 16384.0f * CONST_G;
    aWorldMpu.z = aaWorld.z / 16384.0f * CONST_G;

    //aWorldMpu.x = aaReal.x; //accur calibrate
    //aWorldMpu.y = aaReal.y;
    //aWorldMpu.z = aaReal.z;
    //gyroW.x = gyro[0];
    //gyroW.y = gyro[1];
    //gyroW.z = gyro[2];

    //Serial.print(aaReal.x); //test calibrate
    //Serial.print(' ');
    //Serial.print(aaReal.y);
    //Serial.print(' ');
    //Serial.println(aaReal.z);

    //Serial.print(gyro[0]);
    //Serial.print(' ');
    //Serial.print(gyro[1]);
    //Serial.print(' ');
    //Serial.println(gyro[2]);
  }
}

void resetAV(){
  timequatGPSMPU = 0;
  firstsr = true;
  for (uint8_t i = 0; i < 3; ++i)
  {
    sumGPS[i] = 0;
    sumMPU[i] = 0.0f;
  }
}

void PrintCoord(char* contr, int32_t coord, bool lat = true){
  uint8_t latdeg, latminnat; //lat, but can be lon
  uint32_t latmindr;
  char nswe;
  latdeg = abs(coord) / 10000000;
  latminnat = ((abs(coord) % 10000000) * 60) / 10000000;
  latmindr = (((abs(coord) % 10000000) * 60) % 10000000) / 100;
  if (!lat)
  {
    *contr ^= '0' + (uint8_t)(latdeg / 100);
    Serial.print(latdeg / 100);
    latdeg %= 100;
  }
  if (latdeg < 10)
  {
    Serial.print('0');
  }
  Serial.print(latdeg);
  if (latminnat < 10)
  {
    Serial.print('0');
  }
  Serial.print(latminnat);
  Serial.print('.');
  if (latmindr < 10000)
  {
    Serial.print('0');
  }
  if (latmindr < 1000)
  {
    Serial.print('0');
  }
  if (latmindr < 100)
  {
    Serial.print('0');
  }
  if (latmindr < 10)
  {
    Serial.print('0');
  }
  Serial.print(latmindr);
  
  *contr ^= '0' + (uint8_t)(latdeg / 10);
  *contr ^= '0' + (uint8_t)(latdeg % 10);
  *contr ^= '0' + (uint8_t)(latminnat / 10);
  *contr ^= '0' + (uint8_t)(latminnat % 10);
  *contr ^= '.';
  *contr ^= '0' + (uint8_t)(latmindr / 10000);
  *contr ^= '0' + (uint8_t)((latmindr / 1000) % 10);
  *contr ^= '0' + (uint8_t)((latmindr / 100) % 10);
  *contr ^= '0' + (uint8_t)((latmindr / 10) % 10);
  *contr ^= '0' + (uint8_t)(latmindr % 10);
  if (lat)
  {
    if (coord < 0)
    {
      nswe = 'S';
    }
    else
    {
      nswe = 'N';
    }
  }
  else
  {
    if (coord < 0)
    {
      nswe = 'W';
    }
    else
    {
      nswe = 'E';
    }
  }
  *contr ^= ',';
  *contr ^= nswe;
  *contr ^= ',';
  Serial.print(',');
  Serial.print(nswe);
  Serial.print(',');
}
//***************************************************************************************
//******************              Callback Function                **********************
//***************************************************************************************


void MPUCallback(int16_t *gyro, int16_t *accel, int32_t *quat) {
  uint8_t Spam_Delay = 100; // Built in Blink without delay timer preventing Serial.print SPAM
  GetWorldAccel(gyro, accel, quat, Spam_Delay);
}

void UbloxCallback(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  uint32_t dt;
  VectorFloat normMPU, normGPS;
  float suba;
  
  dt = millis() - lasttime;
  lasttime = millis();

  if ((ubxDataStruct->flags.bits.gnssFixOK == 1) && (ubxDataStruct->sAcc < SPEED_GPS_ACCURACY))
  {
    if (timequatGPSMPU < PERIOD_CALC_ACCEL)
    {
      if (!firstsr)
      {
        timequatGPSMPU += dt;
        sumGPS[0] += ubxDataStruct->velN - lastvn;
        sumGPS[1] += ubxDataStruct->velE - lastve;
        sumGPS[2] += ubxDataStruct->velD - lastvd;

        sumMPU[0] += aWorldMpu.x * dt;
        sumMPU[1] += aWorldMpu.y * dt;
        sumMPU[2] += aWorldMpu.z * dt;
      }
      firstsr = false;
    }
    else
    {
      VectorFloat aWorldGPSsr(sumGPS[0] / (float)timequatGPSMPU, sumGPS[1] / (float)timequatGPSMPU, sumGPS[2] / (float)timequatGPSMPU);
      VectorFloat aWorldMpusr(sumMPU[0] / (float)timequatGPSMPU, sumMPU[1] / (float)timequatGPSMPU, sumMPU[2] / (float)timequatGPSMPU);
      float sqr = aWorldGPSsr.getMagnitudeSqr();
      suba = sqr / aWorldMpusr.getMagnitudeSqr(); //accelerations ratio GPS and MPU
      if ((suba < accurmax) && (suba > 1 / accurmax) && (sqr > SQR_AMPL_ACCEL_GPS_MIN))
      {
        digitalWrite(PC13, LOW); //Capture MPU accelerate
        accurmax = suba * MUL_NEW_ACCEL_ACCURACY;
        timeincaccu = millis();
        normMPU = aWorldMpusr.getNormalized();
        normGPS = aWorldGPSsr.getNormalized();
        MpuToGps.getRotate(normGPS.x, normGPS.y, normGPS.z, normMPU.x, normMPU.y, normMPU.z);
      }
      resetAV();
      firstsr = false;
    }
    lastvn = ubxDataStruct->velN;
    lastve = ubxDataStruct->velE;
    lastvd = ubxDataStruct->velD;
  }
  else
  {
    VectorFloat aWorldGPS = aWorldMpu.getRotated(&MpuToGps);

    lastvn += aWorldGPS.x * (int32_t)dt;
    lastve += aWorldGPS.y * (int32_t)dt;
    lastvd += aWorldGPS.z * (int32_t)dt;

    resetAV();
    //sumaccel[0] += aWorldGPS.x; //Accur calibrate
    //sumaccel[1] += aWorldGPS.y;
    //sumaccel[2] += aWorldGPS.z;
    //sumgyro[0] += gyroW.x;
    //sumgyro[1] += gyroW.y;
    //sumgyro[2] += gyroW.z;
    //++countaccel;
    //Serial.print(sumaccel[0] / countaccel, 1);
    //Serial.print(' ');
    //Serial.print(sumaccel[1] / countaccel, 1);
    //Serial.print(' ');
    //Serial.println(sumaccel[2] / countaccel, 1);
  }
  if ((ubxDataStruct->flags.bits.gnssFixOK == 1) && ubxDataStruct->hAcc < POS_GPS_ACCURACY)
  {
    lastlat = ubxDataStruct->lat;
    lastlon = ubxDataStruct->lon;
    if (ubxDataStruct->fixType = 3)
    {
      lastaltit = ubxDataStruct->hMSL;
      lastsubh = ubxDataStruct->height - lastaltit;
    }

    if (!enabledUart)
    {
      GNSS.setUART1Output(COM_TYPE_NMEA);
      enabledUart = true;
    }
    if (millis() - timeincaccu > PERIOD_ACCEL_ACCURACY)     //If there is no MPU -> GPS capture for 30 seconds, lowering the accuracy
    {
      if (accurmax < MAX_ACCEL_ACCURACY) accurmax *= DEC_ACCEL_ACCURACY;
      digitalWrite(PC13, HIGH);
    }
  }
  else
  {
    lastlat += lastvn * (int32_t)dt / 11112;
    lastlon += (double)(lastve * (int32_t)dt) / 11112.0 / cos((double)lastlat * 0.0000001 * CONST_PI / 180.0);
    lastaltit -= lastvd * (int32_t)dt / 1000; 

    /*?VectorFloat aWorldGPS = aWorldMpu.getRotated(&MpuToGps);
    Serial.print(aWorldGPS.x, 1);
    Serial.print(' ');
    Serial.print(aWorldGPS.y, 1);
    Serial.print(' ');
    Serial.print(aWorldGPS.z, 1);
    Serial.print(' ');
    Serial.print(aWorldMpu.x, 1);
    Serial.print(' ');
    Serial.print(aWorldMpu.y, 1);
    Serial.print(' ');
    Serial.println(aWorldMpu.z, 1);*/

    if(enabledUart)
    {
      GNSS.setUART1Output(0);
      enabledUart = false;
    }
    char control;

    control = 'G';
    control ^= 'N';
    control ^= 'G';
    control ^= 'G';
    control ^= 'A';
    control ^= ',';
    control ^= ',';
    Serial.print("$GNGGA,,");
    PrintCoord(&control, lastlat, true);  //print latitude
    PrintCoord(&control, lastlon, false); //print longitude
    control ^= '1';
    control ^= ',';
    control ^= '0';
    control ^= '1';
    control ^= ',';
    control ^= '1';
    control ^= '5';
    control ^= '0';
    control ^= '.';
    control ^= '0';
    control ^= ',';
    Serial.print("1,01,150.0,");
    uint32_t tempheight = lastaltit / 1000;
    Serial.print(tempheight);
    do 
    {
      control ^= '0' + (uint8_t)(tempheight % 10);
      tempheight /= 10;
    }
    while (tempheight > 0);
    Serial.print(".0,M,");
    control ^= '.';
    control ^= '0';
    control ^= ',';
    control ^= 'M';
    control ^= ',';
    if (lastsubh < 0)
    {
      control ^= '-';
      Serial.print('-');
      tempheight = (uint32_t)(-lastsubh) / 1000;
    }
    else{
      tempheight = lastsubh / 1000;
    } 
    Serial.print(tempheight);
    do 
    {
      control ^= '0' + (uint8_t)(tempheight % 10);
      tempheight /= 10;
    }
    while (tempheight > 0);
    control ^= '.';
    control ^= '0';
    control ^= ',';
    control ^= 'M';
    control ^= ',';
    control ^= ',';
    Serial.print(".0,M,,*");
    if ((uint8_t)control < 16)
    {
      Serial.print('0');
    }
    Serial.println((uint8_t)control, HEX);
  }
}

//***************************************************************************************
//******************                Setup and Loop                 **********************
//***************************************************************************************

void setup() {
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);
  accurmax = MAX_ACCEL_ACCURACY;
  lastaltit = 10000; //Height more than 0
  lastsubh = 0;
  lastlat = 0;
  lastlon = 0;
  lastvn = 0;
  lastve = 0;
  lastvd = 0;
  lasttime = 0;
  timeincaccu = 0;
  timequatGPSMPU = 0;
  enabledUart = true;
  firstsr = true;

  // initialize serial communication
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  mpu.begin();
  mpu.Set_DMP_Output_Rate_Hz(11);           // Set the DMP output rate from 200Hz to 5 Minutes.
  mpu.SetAddress(MPU6050_DEFAULT_ADDRESS);
  mpu.load_DMP_Image(OFFSETS); // Does it all for you
  mpu.on_FIFO(MPUCallback);

  while (GNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    digitalWrite(PC13, LOW);    // turn the LED on by making the voltage LOW
    delay(500);              // wait for a 0.5 second
    digitalWrite(PC13, HIGH);   // turn the LED off (HIGH is the voltage level)
    delay(500);              // wait for a 0.5 second
  }

  GNSS.setAutoPVTcallbackPtr(&UbloxCallback); // Enable automatic NAV PVT messages with callback to printPVTdata
}

void loop() {
  GNSS.checkUblox(); // Check for the arrival of new data and process it.
  GNSS.checkCallbacks();
  mpu.dmp_read_fifo(false); // Must be in loop  No Interrupt pin required at the expense of polling the i2c buss 
}