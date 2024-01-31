#include "Sensors/Sensors.h"
//using namespace BLA;
using namespace Eigen;

Sensors::Sensors(Vector3f magHard, Matrix3f magSoft, Vector3f aBias) {
  this->magCal_hard = magHard;
  this->magCal_soft = magSoft;
  this->accelBias = aBias;
  magAvail = false; baroAvail = false;
};

void Sensors::init() {    //initialize 9DoF IMU settings and turn on baro and high-G accel
  IMU.settings.gyro.enabled = true;
  IMU.settings.gyro.scale = 500; //500dps
  IMU.settings.gyro.sampleRate = 5; //476hz
  IMU.settings.gyro.lowPowerEnable = false;
  IMU.settings.gyro.HPFEnable = false;

  IMU.settings.accel.enabled = true;
  IMU.settings.accel.scale = 16;    //16G
  IMU.settings.accel.sampleRate = 5; //476hz

  IMU.settings.mag.enabled = true;
  IMU.settings.mag.scale = 4;     //4 Gauss

  IMU.begin(0x6B,0x1E,Wire);

  Wire.beginTransmission(0x1E);   //set fast ODR and configure for 560hz as in https://community.st.com/t5/mems-sensors/lsm9ds1-data-sheet-says-that-the-fast-odr-mode-enables/m-p/334058
  Wire.write(0x20);
  Wire.write(0b00110010);
  Wire.endTransmission();
  Wire.beginTransmission(0x1E);
  Wire.write(0x23);
  Wire.write(0b00000100);
  Wire.endTransmission();

  IMU_HighG.begin();    //highG needs to be looked at, values are weird

  barometer.connect();
  barometer.ReadProm();

  Wire.setClock(400000);

  float xcal = 0, ycal = 0, zcal = 0, magMag = 0, mbaseX = 0, mbaseY = 0, mbaseZ = 0, abaseX = 0, abaseY = 0, abaseZ = 0;
  for(int i = 0; i < 250; i++) {
    while (!IMU.gyroAvailable()) {delay(1);};
    IMU.readGyro();
    gX = (IMU.calcGyro(IMU.gx) );
    gY = (IMU.calcGyro(IMU.gy) );
    gZ = (IMU.calcGyro(IMU.gz) );
    xcal += gX; ycal += gY; zcal += gZ;

    IMU.readMag();
    mag = magCal_soft * Vector<float,3> {(float)IMU.mx-magCal_hard(0),(float)IMU.my-magCal_hard(1),(float)IMU.mz-magCal_hard(2)};
    mX = -IMU.calcMag(mag(0));
    mY = IMU.calcMag(mag(1));
    mZ = -IMU.calcMag(mag(2));
    magMag = sqrtf(mX*mX+mY*mY+mZ*mZ);  //normalize magnetometer reading
    mX /= magMag; mY /= magMag; mZ /= magMag;
    mbaseX += mX; mbaseY += mY; mbaseZ += mZ;

    IMU.readAccel();
    aX = IMU.calcAccel(IMU.ax)*9.80665-accelBias(0);
    aY = IMU.calcAccel(IMU.ay)*9.80665-accelBias(1);
    aZ = -IMU.calcAccel(IMU.az)*9.80665-accelBias(2);
    abaseX += aX; abaseY += aY; abaseZ += aZ;
  }
  magBase = {mbaseX/250.f,mbaseY/250.f,mbaseZ/250.f};
  aBase = {abaseX/250.f,abaseY/250.f,abaseZ/250.f};
  xOfst = xcal/250.f; yOfst = ycal/250.f; zOfst = zcal/250.f;
  Serial.println("calibrated!");
}

void Sensors::baroData() {
  unsigned long value=0;
  unsigned long c=0;
  switch(baroStep) {
    case 0:
      if (micros()-timeBaro >= 25000) {
        barometer.send_cmd(MS5xxx_CMD_ADC_CONV+MS5xxx_CMD_ADC_D2+MS5xxx_CMD_ADC_1024);
        baroStep = 1;
        timeBaro = micros();
      }
      break;
    case 1:
      if(micros()-timeBaro >= 4000) {
        barometer.send_cmd(MS5xxx_CMD_ADC_READ); // read out values
        Wire.requestFrom(0x76, 3);
        c = Wire.read();
        value = (c<<16);
        c = Wire.read();
        value += (c<<8);
        c = Wire.read();
        value += c;
        d2 = value;
        barometer.send_cmd(MS5xxx_CMD_ADC_CONV+MS5xxx_CMD_ADC_D1+MS5xxx_CMD_ADC_1024);
        baroStep = 2;
      }
      break;
    case 2:
      if (micros()-timeBaro >= 8000) {
        barometer.send_cmd(MS5xxx_CMD_ADC_READ); // read out values
        Wire.requestFrom(0x76, 3);
        c = Wire.read();
        value = (c<<16);
        c = Wire.read();
        value += (c<<8);
        c = Wire.read();
        value += c;
        d1 = value;
        barometer.Readout(d1,d2);
        baroStep = 0;
        prs = barometer.GetPres();
        baroAvail = true;
      }
  }
}

float cutoff_freq = 50.0f;  // Cutoff frequency in Hz
float RC = 1.0/(2*PI*cutoff_freq);


void Sensors::getData() {
  //baroData();
  if (IMU.gyroAvailable()) {
    IMU.readGyro();
    gX = (IMU.calcGyro(IMU.gx) - xOfst)/180.f;    //for some godforsaken reason the gyro is in some nonexistent unit pi*deg/s
    gY = (IMU.calcGyro(IMU.gy) - yOfst)/180.f;
    gZ = -(IMU.calcGyro(IMU.gz) - zOfst)/180.f;
  }
  if (IMU.accelAvailable()) {
    IMU.readAccel();
    aX = IMU.calcAccel(IMU.ax)*9.80665-accelBias(0);
    aY = IMU.calcAccel(IMU.ay)*9.80665-accelBias(1);
    aZ = -IMU.calcAccel(IMU.az)*9.80665-accelBias(2);
  }
  if (IMU.magAvailable()) {
    IMU.readMag();
    mag = magCal_soft * Vector<float,3> {(float)IMU.mx-magCal_hard(0),(float)IMU.my-magCal_hard(1),(float)IMU.mz-magCal_hard(2)};
    mX = -IMU.calcMag(mag(0));
    mY = IMU.calcMag(mag(1));
    mZ = -IMU.calcMag(mag(2));
    // Serial.print(mX,5); Serial.print(", "); Serial.print(mY,5); Serial.print(", "); Serial.println(mZ,5);

    if(magFilter) {
      // Apply Butterworth filter
      float alpha = dt/(RC+dt);
      float filtered_mX = alpha * mX + (1 - alpha) * prev_filtered_mX + alpha * (prev_mX - mX);
      float filtered_mY = alpha * mY + (1 - alpha) * prev_filtered_mY + alpha * (prev_mY - mY);
      float filtered_mZ = alpha * mZ + (1 - alpha) * prev_filtered_mZ + alpha * (prev_mZ - mZ);

      // Store current readings for the next iteration
      prev_mX = mX;
      prev_mY = mY;
      prev_mZ = mZ;
      prev_filtered_mX = filtered_mX;
      prev_filtered_mY = filtered_mY;
      prev_filtered_mZ = filtered_mZ;

      // Use filtered readings
      mX = filtered_mX;
      mY = filtered_mY;
      mZ = filtered_mZ;
    }
    float magMag = sqrtf(mX*mX+mY*mY+mZ*mZ);  //normalize magnetometer reading
    mX /= magMag; mY /= magMag; mZ /= magMag;
    magAvail = true;
  }
}

float Sensors::altCalc() {
  return ((288.15/-0.0065)*(pow(prs/101325.,0.1902632)-1.))-altInit;    //barometric formula solved for altitude in meters
}