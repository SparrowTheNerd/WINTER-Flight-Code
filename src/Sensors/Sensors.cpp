#include "Sensors/Sensors.h"
using namespace BLA;

Sensors::Sensors(Matrix<3> magHard, Matrix<3,3> magSoft) {
    this->magCal_hard = magHard;
    this->magCal_soft = magSoft;
};

void Sensors::init() {    //initialize 9DoF IMU settings and turn on baro and high-G accel
  IMU.begin(0x6B,0x1E,Wire);
  IMU.settings.gyro.enabled = true;
  IMU.settings.gyro.scale = 500; //2000dps
  IMU.settings.gyro.sampleRate = 5; //476hz
  IMU.settings.gyro.lowPowerEnable = false;
  IMU.settings.gyro.HPFEnable = false;

  IMU.settings.accel.enabled = true;
  IMU.settings.accel.scale = 16;    //16G
  IMU.settings.accel.sampleRate = 5; //476hz

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

  float xCal, yCal, zCal;
  for (int i = 0; i < 500; i++) {
    while(!IMU.gyroAvailable()) {delayMicroseconds(500); };
    IMU.readGyro();
    xCal += (IMU.calcGyro(IMU.gx));
    yCal += (IMU.calcGyro(IMU.gy));
    zCal += (IMU.calcGyro(IMU.gz));   
  }
  xOfst = xCal/500.f; yOfst = yCal/500.f; zOfst = zCal/500.f;
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
  baroData();
  if (IMU.gyroAvailable()) {
    IMU.readGyro();
    gX = DEG_TO_RAD*(IMU.calcGyro(IMU.gx) - xOfst);
    gY = DEG_TO_RAD*(IMU.calcGyro(IMU.gy) - yOfst);
    gZ = DEG_TO_RAD*(IMU.calcGyro(IMU.gz) - zOfst);
    // gX = IMU.gx; gY = IMU.gy; gZ = IMU.gz;
  }
  if (IMU.accelAvailable()) {
    IMU.readAccel();
    aX = IMU.calcAccel(IMU.ax)*9.80665 + 0.0635;
    aY = IMU.calcAccel(IMU.ay)*9.80665 - 0.1735;
    aZ = IMU.calcAccel(IMU.az)*9.80665 + 0.20075;
    // aX = IMU.ax; aY = IMU.ay; aZ = IMU.az;
    //Serial.print(aX,3); Serial.print(", "); Serial.print(aY,3); Serial.print(", "); Serial.println(aZ+9.80665,3);
  }
  if (IMU.magAvailable()) {   //using mag calibration, eq ref https://www.digikey.com/en/maker/projects/how-to-calibrate-a-magnetometer/50f6bc8f36454a03b664dca30cf33a8b
    IMU.readMag();
    Matrix<3> hardCal = {IMU.calcMag(IMU.mx-magCal_hard(0)),IMU.calcMag(IMU.my-magCal_hard(1)),IMU.calcMag(IMU.mz-magCal_hard(2))};
    Matrix<3> mag = magCal_soft * hardCal;
    mX = -(mag(0));
    mY = (mag(1));
    mZ = (mag(2));
    //mX = IMU.mx; mY = IMU.my; mZ = IMU.mz;
    magAvail = true;
  }
}

float Sensors::altCalc() {
  return ((288.15/-0.0065)*(pow(prs/101325.,0.1902632)-1.))-altInit;    //barometric formula solved for altitude in meters
}