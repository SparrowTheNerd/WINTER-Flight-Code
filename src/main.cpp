//X axis roll, Y axis pitch, Z axis yaw

#include <Arduino.h>
#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <MS5xxx.h>
#include <SdFat.h>
//#include <Kalman.h>
#include <BasicLinearAlgebra.h>
//using namespace BLA;

#include "Kalman/RocketKalman.h"
RocketKalman * kalman = new RocketKalman(50);

#define BZR     PC8
#define CS_SD   PC0
#define gXOfst 1.301940492
#define gYOfst 0.859327296
#define gZOfst 0.528033635

#define Nstate  10      //xyz pos, xyz vel, angle quat
#define Nobs    10      //xyz accel, xyz gyro, xyz mag, baro
//measurement stddevs
#define stddev_xl   0.180   //180 milli G's from LSM9DS1 datasheet
#define stddev_gy   2.0     //2 deg/s, roughly measured
#define stddev_mg   0.1     //1 Gauss from LSM9DS1 datasheet, will improve post calibration
#define stddev_ps   10.    //400Pa, around 10ft from MS5607 datasheet
//model stddev (~1/inertia)
// #define m_pos       0.1
// #define m_vel       0.1
// #define m_ang       0.5
// #define m_rol       0.5

BLA::Matrix<4> uXl; // cntrl vector for accel
BLA::Matrix<4> uGy; // cntrl vector for gyros
BLA::Matrix<3> zMg; // obs vector for magns
BLA::Matrix<1> zBm; // obs vector for press
BLA::Matrix<10> x; //x_n,n
BLA::Matrix<10> x_prior; //x_n,n-1
BLA::Matrix<3,10> Hmg;  //mag obs matrix
BLA::Matrix<1,10> Hbm;  //baro obs matrix
BLA::Matrix<10,4> G;    //cntrl matrix
BLA::Matrix<10,10> F;   //state transn matrix
BLA::Matrix<10,10> P;   //covariance matrix
BLA::Matrix<10,10> P_prior;  //covariance prior prediction
BLA::Matrix<3,3> Rmg;   //mag meas covar matrix
BLA::Matrix<1,1> Rbm;   //baro meas covar matrix
BLA::Matrix<10,3> Kmg;  //mag kalman gain
BLA::Matrix<10>   Kbm;  //baro kalman gain
BLA::Matrix<3> mgBase;  //initial magnetometer vector, for comparing against

Adafruit_ADXL375 IMU_HighG = Adafruit_ADXL375(1,&Wire);
MS5xxx barometer(&Wire);
LSM9DS1 IMU;
SdFat SD;
File file;

void imuInit();
void barometerInit();
void getSensorData();
void KalmanInit();
void KalmanFilter();
void baroData();
BLA::Matrix<3> axisAngles();
uint32_t Time, timeBaro;
bool magAvail;  //is there mag data available?
bool baroAvail = false; //is there baro data available?
char fileName[] = "FLIGHTDATA00.bin";
float dT;
float angX, angY, angZ;
float gX, gY, gZ, aX, aY, aZ, mX, mY, mZ;
float altInit = 0.;
double prs, tmp;
struct sensData {   
  float time, gX, gY, gZ, aX, aY, aZ, mX, mY, mZ, prs, tmp;
} data;

struct quaternion {
  float r;          //real value
  float i, j, k;    //imaginary values
};

BLA::Matrix<3,1> magCal_hard = {-19.03*8,28.14*8,-66.63*8}; //hard and soft iron calibrations
BLA::Matrix<3,3> magCal_soft = { 0.961, 0.026,-0.024,
                                 0.026, 1.010,-0.002,
                                -0.024,-0.002, 1.032};
BLA::Matrix<3,1> mag;

float degToRad(float deg) {
  return deg*DEG_TO_RAD;
}
float radToDeg(float rad) {
  return rad*RAD_TO_DEG;
}
void setup() {
  pinMode(BZR,OUTPUT);
  analogWriteResolution(16);  //set pwm resolution to 16 bit (0-65535)
  analogWriteFrequency(1000); //set pwm frequency to 1KHz

  SerialUSB.begin(); //start serial port
  while(!SerialUSB);

  Wire.begin(uint32_t(PB9_ALT0),uint32_t(PB8_ALT0));

  imuInit();    //set IMU settings
  //barometerInit();
  barometer.connect();
  barometer.ReadProm();
  
  // IMU_HighG.begin();    //highG needs to be looked at, values are weird
  Wire.setClock(400000);

  KalmanInit();

  timeBaro = micros();
  Time = micros();
  delay(1);
}

int counter = 0;
void loop() {
  dT = (float)(micros()-Time)/1000000.;
  Time = micros();
  getSensorData();
  KalmanFilter();

  // if(counter == 50) {
  //   BLA::Matrix<3> angles = axisAngles();
  //   Serial.print("X: "); Serial.print(x(0)); Serial.print("  Y: "); Serial.print(x(1)); Serial.print("  Z: "); Serial.print(x(2)); Serial.print(" Xs: "); Serial.print(x(3)); Serial.print("  Y: "); Serial.print(angles(0),5);
  //   Serial.print("  R: "); Serial.print(angles(1),5); Serial.print("  P: "); Serial.println(angles(2),5);
  //   counter = 0;
  // }
  // counter += 1;
  magAvail = false;
  baroAvail = false;    //reset sensor availability for next loop
}

float altCalc() {
  return ((288.15/-0.0065)*(pow(prs/101325.,0.1902632)-1.))-altInit;
}
BLA::Matrix<4> localToGlobal(BLA::Matrix<4> meas, quaternion q) {     //use a-priori state to transform meas from rkt to global
  BLA::Matrix<4,4> transform = {  1.-2.*(q.i*q.i+q.k*q.k), 2.*(q.i*q.j-q.k*q.r), 2.*(q.i*q.k+q.j*q.r),  0.,
                                  2.*(q.i*q.j+q.k*q.r), 1.-2.*(q.i*q.i+q.k*q.k), 2.*(q.j*q.k-q.i*q.r),  0.,
                                  2.*(q.i*q.k-q.j*q.r), 2.*(q.j*q.k+q.i*q.r), 1.-2.*(q.i*q.i+q.j*q.j),  0.,
                                              0.      ,           0.        ,         0.            ,   1.};
  return transform*meas;
}

int baroStep = 0;
unsigned long d1, d2;
void baroData() {
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
        zBm = altCalc();
        baroAvail = true;
      }
  }
}
void getSensorData() {
  baroData();
  if (IMU.gyroAvailable()) {
    IMU.readGyro();
    gX = degToRad(IMU.calcGyro(IMU.gx) - gXOfst);
    gY = degToRad(IMU.calcGyro(IMU.gy) - gYOfst);
    gZ = degToRad(IMU.calcGyro(IMU.gz) - gZOfst);
  }
  if (IMU.accelAvailable()) {
    IMU.readAccel();
    aX = IMU.calcAccel(IMU.ax)*9.8066;
    aY = IMU.calcAccel(IMU.ay)*9.8066;
    aZ = IMU.calcAccel(IMU.az)*9.8066;
  }
  if (IMU.magAvailable()) {   //using mag calibration, eq ref https://www.digikey.com/en/maker/projects/how-to-calibrate-a-magnetometer/50f6bc8f36454a03b664dca30cf33a8b
    IMU.readMag();
    mag = magCal_soft * BLA::Matrix<3,1> {(float)IMU.mx-magCal_hard(0),(float)IMU.my-magCal_hard(1),(float)IMU.mz-magCal_hard(2)};
    mX = IMU.calcMag(mag(0));
    mY = IMU.calcMag(mag(1));
    mZ = IMU.calcMag(mag(2));
    magAvail = true;
  }
  //Serial.print(x(6),5); Serial.print(", "); Serial.print(x(7),5); Serial.print(", "); Serial.print(x(8),5); Serial.print(", "); Serial.println(x(9),5);
  uXl = localToGlobal(BLA::Matrix<4> {aX,aY,aZ,1.}, quaternion {x(6),x(7),x(8),x(9)}); //accel meas to global frame
  uGy = localToGlobal(BLA::Matrix<4> {gX,gY,gZ,1.}, quaternion {x(6),x(7),x(8),x(9)}); //gyros meas to global frame
  //uXl(0) -= 9.8066; //subtract gravity
  Serial << uXl;
  Serial.println(" ");
}
void HmgCalc(quaternion q) {  //calculate values of the mag obs matrix given state vector quats
  Hmg = { 0, 0, 0, 0, 0, 0, 0, 0, 0, radToDeg(atan2(2*(q.r*q.i+q.j*q.k),1-2*(q.i*q.i+q.j*q.j))),
          0, 0, 0, 0, 0, 0, 0, 0, 0,           radToDeg(asin(2*(q.r*q.j-q.i*q.k))),
          0, 0, 0, 0, 0, 0, 0, 0, 0, radToDeg(atan2(2*(q.r*q.k+q.i*q.j),1-2*(q.j*q.j+q.k*q.k)))};
}
void kalmanGain() {
  /* magnetometer section */
  BLA::Matrix<10,3> Hmg_T = ~Hmg;
  Kmg = P_prior*Hmg_T*Inverse(Hmg*P_prior*Hmg_T+Rmg);
  /* barometer section */
  BLA::Matrix<10> Hbm_T = ~Hbm;
  Kbm = P_prior*Hbm_T*Inverse(Hbm*P_prior*Hbm_T+Rbm);
}
void kalUpdate() {    //state update using mag and baro & update covariance
  x = x_prior;
  if(magAvail) {
    float magMag = sqrtf(mX*mX+mY*mY+mZ*mZ);    //magnetometer magnitude
    zMg(0) = (acos(mX/magMag)-acos(mgBase(0)));
    zMg(1) = (acos(mY/magMag)-acos(mgBase(1)));
    zMg(2) = (acos(mZ/magMag)-acos(mgBase(2)));
    x += Kmg*(zMg - Hmg*x_prior);
  }
  if(baroAvail) {
    x += Kbm*(zBm - Hbm*x_prior);
  }

  BLA::Matrix<10,10> I; I.Fill(0.);
  for(int i=0; i<10; i++) {   //identity matrix
    I(i,i) = 1.;
  }
  P = (I-Kbm*Hbm)*P_prior*~(I-Kbm*Hbm) + Kbm*Rbm*~Kbm;
  P+= (I-Kmg*Hmg)*P_prior*~(I-Kmg*Hmg) + Kmg*Rmg*~Kmg;
}
void kalExtrapolate() {   //extrapolation / prediction function
  float tht = (uGy(0));
  float phi = (uGy(1));
  float psi = (uGy(2));
  G = { dT*dT/2,    0   ,    0   ,  0 ,
           0   , dT*dT/2,    0   ,  0 ,
           0   ,    0   , dT*dT/2,  0 ,
           dT  ,    0   ,    0   ,  0 ,
           0   ,    dT  ,    0   ,  0 ,
           0   ,    0   ,    dT  ,  0 ,
           0   ,    0   ,    0   ,  0 ,
           0   ,    0   ,    0   ,  0 ,
           0   ,    0   ,    0   ,  0 ,
           0   ,    0   ,    0   ,  0 };
  
  F = { 1, 0, 0, dT,  0,  0, 0, 0, 0, 0,
        0, 1, 0,  0, dT,  0, 0, 0, 0, 0,
        0, 0, 1,  0,  0, dT, 0, 0, 0, 0,
        0, 0, 0,  1,  0,  0, 0, 0, 0, 0,
        0, 0, 0,  0,  1,  0, 0, 0, 0, 0,
        0, 0, 0,  0,  0,  1, 0, 0, 0, 0,
        0, 0, 0,  0,  0,  0, 1, 0, 0, 0,
        0, 0, 0,  0,  0,  0, 0, 1, 0, 0,
        0, 0, 0,  0,  0,  0, 0, 0, 1, 0,
        0, 0, 0,  0,  0,  0, 0, 0, 0, 1};
  
  //quaternion angular rates: https://mariogc.com/post/angular-velocity-quaternions/ 
  BLA::Matrix<4> q0 = {x(6),x(7),x(8),x(9)};
  BLA::Matrix<4,4> omega = {  0 , psi , -phi, tht,
                            -psi,  0  ,  tht, phi,
                             phi, -tht,   0 , psi,
                            -tht, -phi, -psi,  0 };
  BLA::Matrix<4> qd = omega * q0 * 0.5f * dT;

  x_prior = (F*x + G*uXl);
  x_prior(6) += qd(0); x_prior(7) += qd(1); x_prior(8) += qd(2); x_prior(9) += qd(3);
  // Serial << x_prior;
  // Serial.print('\n');
  P_prior = F*P*~F; 
}

void quatNormalize() {
  float normFact = sqrt(x(6)*x(6) + x(7)*x(7) + x(8)*x(8) + x(9)*x(9));
  if (normFact > 1.) {
    x(6) /= normFact;
    x(7) /= normFact;
    x(8) /= normFact;
    x(9) /= normFact;
  }
}

// void writeToSD() {
//   file.write((uint8_t *)&data,sizeof(data)/sizeof(uint8_t));
// }
BLA::Matrix<3> axisAngles() {
  quaternion q = {x(6),x(7),x(8),x(9)};
  float psi = radToDeg(atan2(2*(q.r*q.k+q.i*q.j),1-2*(q.j*q.j+q.k*q.k)));
  float theta = radToDeg(asin(2*(q.r*q.j-q.i*q.k)));
  float phi = radToDeg(atan2(2*(q.r*q.i+q.j*q.k),1-2*(q.i*q.i+q.j*q.j)));

  return {psi,theta,phi};
}
void KalmanFilter() {
  //extrapolate state, extrapolate uncert, compute gain, update estimate, update uncert
  quatNormalize();
  kalExtrapolate();
  HmgCalc(quaternion {x_prior(6),x_prior(7),x_prior(8),x_prior(9)});  //use x_n,n-1 to 
  kalmanGain();
  kalUpdate();
}

void KalmanInit() {   //initialize kalman matrices and orientation
  Rmg = { stddev_mg,     0    ,     0     ,
              0    , stddev_mg,     0     ,
              0    ,     0    , stddev_mg };
  Rbm = { stddev_ps };

  Hbm = { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  //orientation init
  IMU.readAccel();
  aX = IMU.calcAccel(IMU.ax);
  aY = IMU.calcAccel(IMU.ay);
  aZ = IMU.calcAccel(IMU.az);
  IMU.readMag();
  mag = magCal_soft * BLA::Matrix<3,1> {(float)IMU.mx-magCal_hard(0),(float)IMU.my-magCal_hard(1),(float)IMU.mz-magCal_hard(2)};
  mX = IMU.calcMag(mag(0));
  mY = IMU.calcMag(mag(1));
  mZ = IMU.calcMag(mag(2));
  mgBase = {mX,mY,mZ};

  while(!baroAvail) {
    baroData();
  }
  altInit = altCalc();

  BLA::Matrix<3> gravVect = {aX, aY, aZ};
  gravVect /= sqrt(aX*aX+aY*aY+aZ*aZ);      //normalize gravity vector
  float psi = M_PI_2-(acos(gravVect(1)));
  float phi = M_PI_2-(acos(gravVect(2)));
  float theta = 0.;
  quaternion q;
  q.r = sin(phi/2.)*cos(theta/2.)*cos(psi/2.)-cos(phi/2.)*sin(theta/2.)*sin(psi/2.);    //convert initial orientation into state quaternion
  q.i = cos(phi/2.)*cos(theta/2.)*cos(psi/2.)+sin(phi/2.)*sin(theta/2.)*sin(psi/2.);
  q.j = cos(phi/2.)*sin(theta/2.)*cos(psi/2.)+sin(phi/2.)*cos(theta/2.)*sin(psi/2.);
  q.k = cos(phi/2.)*cos(theta/2.)*sin(psi/2.)-sin(phi/2.)*sin(theta/2.)*cos(psi/2.);

  x = {0., 0., 0., 0., 0., 0., q.r, q.i, q.j, q.k};   //initial state estimate; position and velocity are zero, and orientation calculated above
  //Serial.print(x(6),5); Serial.print(" "); Serial.print(x(7),5); Serial.print(" "); Serial.print(x(8),5); Serial.print(" "); Serial.println(x(9),5);
  P.Fill(0.);   //initial covariance error is zero, initial estimate is as close to true state as is reasonable
}
void sdInit() {     //initialize SD card and filename
  SD.begin(CS_SD, SPI_FULL_SPEED);
  for (uint8_t i = 0; i < 100; i++) {
    fileName[10] = i/10 + '0';
    fileName[11] = i%10 + '0';
    if (SD.exists(fileName)) continue;
    file.open(fileName);
    break;
  }
}
void imuInit() {    //initialize 9DoF IMU settings
  IMU.settings.gyro.enabled = true;
  IMU.settings.gyro.scale = 2000; //2000dps
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

  IMU.begin(0x6B,0x1E,Wire);
}
void barometerInit() {
  barometer.connect();
  barometer.ReadProm();

  delay(2000); //make sure enough time has elapsed that all sensors will have an available measurement

  while(!baroAvail) {

    baroData(); //get an initial baro measurement

  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 384;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

// void magCal() {   //for use with MotionCal software
//   Serial.print("Raw:"); //Serial.print("0,0,0,0,0,0,");
//   Serial.print((IMU.ax)/8); Serial.print(",");
//   Serial.print((IMU.ay)/8); Serial.print(",");
//   Serial.print((IMU.az)/8); Serial.print(",");
//   Serial.print((IMU.gx-140)/8); Serial.print(",");
//   Serial.print((IMU.gy-95)/8); Serial.print(",");
//   Serial.print((IMU.gz-70)/8); Serial.print(",");
//   Serial.print((IMU.mx)/8); Serial.print(",");
//   Serial.print((IMU.my)/8); Serial.print(",");
//   Serial.print((IMU.mz)/8); Serial.println("");
// }