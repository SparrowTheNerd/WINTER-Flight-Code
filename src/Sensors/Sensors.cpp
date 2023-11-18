#include "Sensors/Sensors.h"
using namespace BLA;

Sensors::Sensors() {
  magCal_hard = {0,0,0};
  magCal_soft = { 1.f,  0,  0,
                    0, 1.f,  0,
                    0,  0, 1.f};
};

void Sensors::init() {    //initialize 9DoF IMU settings and turn on baro and high-G accel
  IMU.begin(0x6B,0x1E,Wire);
  IMU.settings.gyro.enabled = true;
  IMU.settings.gyro.scale = 500; //500dps
  IMU.settings.gyro.sampleRate = 3; //119hz
  IMU.settings.gyro.lowPowerEnable = false;
  IMU.settings.gyro.HPFEnable = false;

  IMU.settings.accel.enabled = true;
  IMU.settings.accel.scale = 16;    //16G
  IMU.settings.accel.sampleRate = 3; //119hz

  IMU.settings.mag.enabled = true;
  IMU.settings.mag.scale = 4;     //4 Gauss
  IMU.settings.mag.sampleRate = 7;    //80hz
  IMU.settings.mag.XYPerformance = 3;   //high performance & continuous
  IMU.settings.mag.ZPerformance = 3;
  IMU.settings.mag.operatingMode = 0;

  IMU_HighG.begin();    //highG needs to be looked at, values are weird

  barometer.connect();
  barometer.ReadProm();

  Wire.setClock(400000);

  float xcal = 0, ycal = 0, zcal = 0;
  for(int i = 0; i < 250; i++) {
    while (!IMU.gyroAvailable()) {delay(1);};
    IMU.readGyro();
    gX = (IMU.calcGyro(IMU.gx) );
    gY = (IMU.calcGyro(IMU.gy) );
    gZ = (IMU.calcGyro(IMU.gz) );
    xcal += gX; ycal += gY; zcal += gZ;
  }
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
void Sensors::getData() {
  //baroData();
  if (IMU.gyroAvailable()) {
    IMU.readGyro();
    gX = DEG_TO_RAD*(IMU.calcGyro(IMU.gx) - xOfst);
    gY = DEG_TO_RAD*(IMU.calcGyro(IMU.gy) - yOfst);
    gZ = DEG_TO_RAD*(IMU.calcGyro(IMU.gz) - zOfst);
  }
  // if (IMU.accelAvailable()) {
  //   IMU.readAccel();
  //   aX = IMU.calcAccel(IMU.ax)*9.80665;
  //   aY = IMU.calcAccel(IMU.ay)*9.80665;
  //   aZ = IMU.calcAccel(IMU.az)*9.80665;
  // }
  // if (IMU.magAvailable()) {   //using mag calibration, eq ref https://www.digikey.com/en/maker/projects/how-to-calibrate-a-magnetometer/50f6bc8f36454a03b664dca30cf33a8b
  //   IMU.readMag();
  //   Matrix<3> mag = magCal_soft * Matrix<3> {(float)IMU.mx-magCal_hard(0),(float)IMU.my-magCal_hard(1),(float)IMU.mz-magCal_hard(2)};
  //   mX = IMU.calcMag(mag(0));
  //   mY = IMU.calcMag(mag(1));
  //   mZ = IMU.calcMag(mag(2));
  //   magAvail = true;
  // }
}

float Sensors::altCalc() {
  return ((288.15/-0.0065)*(pow(prs/101325.,0.1902632)-1.))-altInit;    //barometric formula solved for altitude in meters
}