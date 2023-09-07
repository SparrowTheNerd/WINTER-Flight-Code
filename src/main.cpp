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
#define stddev_mg   1.0     //1 Gauss from LSM9DS1 datasheet, will improve post calibration
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
uint32_t Time, timeBaro;
bool magAvail;  //is there mag data available?
bool baroAvail; //is there baro data available?
char fileName[] = "FLIGHTDATA00.bin";
float dT;
float angX, angY, angZ;
float time, gX, gY, gZ, aX, aY, aZ, mX, mY, mZ, prs, tmp;
// struct sensData {   
//   float time, gX, gY, gZ, aX, aY, aZ, mX, mY, mZ, prs, tmp;
// } data;

struct quaternion {
  float i, j, k;    //imaginary values
  float r;          //real value
};

BLA::Matrix<3,1> magCal_hard = {-19.03*8,28.14*8,-66.63*8}; //hard and soft iron calibrations
BLA::Matrix<3,3> magCal_soft = { 0.961, 0.026,-0.024,
                                 0.026, 1.010,-0.002,
                                -0.024,-0.002, 1.032};
BLA::Matrix<3,1> mag;

void setup() {
  pinMode(BZR,OUTPUT);
  analogWriteResolution(16);  //set pwm resolution to 16 bit (0-65535)
  analogWriteFrequency(1000); //set pwm frequency to 1KHz

  SerialUSB.begin(); //start serial port
  while(!SerialUSB);

  Wire.begin(uint32_t(PB9_ALT0),uint32_t(PB8_ALT0));

  imuInit();    //set IMU settings
  barometerInit();
  
  IMU_HighG.begin();    //highG needs to be looked at, values are weird
  Wire.setClock(400000);

  KalmanInit();

  Time = micros();
  timeBaro = micros();
}

void loop() {
  dT = (float)(micros()-Time)/1000000.;
  Time = micros();
  getSensorData();

  //KALMAN HERE

  magAvail = false;
  baroAvail = false;    //reset sensor availability for next loop
}

float altCalc() {
  return (288.15/-0.0065)*(pow(prs/101325.,0.1902632)-1);
}

int baroStep = 0;
unsigned long d1, d2;
void baroData() {     //rewritten function from MS5xxx lib so that baro low pollrate data can be gathered around IMU data (uses IMU proc time as delay)
  unsigned long value=0;
  unsigned long c=0;
  switch (baroStep) {
  case 0:
    if ((micros() - timeBaro) >= 25000) {   //40hz sample rate
      barometer.send_cmd(MS5xxx_CMD_ADC_CONV+MS5xxx_CMD_ADC_D2+MS5xxx_CMD_ADC_1024);
      baroStep = 1;
      timeBaro = micros();
    }
    break;
  case 1:
    barometer.send_cmd(MS5xxx_CMD_ADC_READ); // read out values
    Wire.requestFrom(0x76, 3);
    c = Wire.read();
    value = (c<<16);
    c = Wire.read();
    value += (c<<8);
    c = Wire.read();
    value += c;
    Wire.endTransmission(true);
    d2 = value;
    baroStep = 2;
    break;
  case 2:
    barometer.send_cmd(MS5xxx_CMD_ADC_CONV+MS5xxx_CMD_ADC_D1+MS5xxx_CMD_ADC_1024);
    baroStep = 3;
    break;
  case 3:
    barometer.send_cmd(MS5xxx_CMD_ADC_READ); // read out values
    Wire.requestFrom(0x76, 3);
    c = Wire.read();
    value = (c<<16);
    c = Wire.read();
    value += (c<<8);
    c = Wire.read();
    value += c;
    Wire.endTransmission(true);
    d1 = value;
    baroStep = 4;
    break;
  case 4:
    barometer.Readout(d1,d2);
    prs = barometer.GetPres();
    tmp = barometer.GetTemp();
    baroStep = 0;
    zBm = altCalc();
    baroAvail = true;
    break;
  }
}
void getSensorData() {
  baroData();
  if (IMU.gyroAvailable()) {
    IMU.readGyro();
    gX = IMU.calcGyro(IMU.gx) - gXOfst;
    gY = IMU.calcGyro(IMU.gy) - gYOfst;
    gZ = IMU.calcGyro(IMU.gz) - gZOfst;
  }
  if (IMU.accelAvailable()) {
    IMU.readAccel();
    aX = IMU.calcAccel(IMU.ax);
    aY = IMU.calcAccel(IMU.ay);
    aZ = IMU.calcAccel(IMU.az);
  }
  if (IMU.magAvailable()) {   //using mag calibration, eq ref https://www.digikey.com/en/maker/projects/how-to-calibrate-a-magnetometer/50f6bc8f36454a03b664dca30cf33a8b
    IMU.readMag();
    mag = magCal_soft * BLA::Matrix<3,1> {(float)IMU.mx-magCal_hard(0),(float)IMU.my-magCal_hard(1),(float)IMU.mz-magCal_hard(2)};
    mX = IMU.calcMag(mag(0));
    mY = IMU.calcMag(mag(1));
    mZ = IMU.calcMag(mag(2));
    magAvail = true;
  }
  uXl = localToGlobal(BLA::Matrix<4> {aX,aY,aZ,1.}, quaternion {x_prior(6),x_prior(7),x_prior(8),x_prior(9)}, x_prior(0),x_prior(1),x_prior(2)); //accel meas to global frame
  uGy = localToGlobal(BLA::Matrix<4> {gX,gY,gZ,1.}, quaternion {x_prior(6),x_prior(7),x_prior(8),x_prior(9)}, x_prior(0),x_prior(1),x_prior(2)); //gyros meas to global frame
}

BLA::Matrix<4> localToGlobal(BLA::Matrix<4> meas, quaternion q, float Tx, float Ty, float Tz) {     //use a-priori state to transform meas from rkt to global
  BLA::Matrix<4,4> transform = {  1-2*(q.i*q.i+q.k*q.k), 2*(q.i*q.j-q.k*q.r), 2*(q.i*q.k+q.j*q.r), Tx,
                                  2*(q.i*q.j+q.k*q.r), 1-2*(q.i*q.i+q.k*q.k), 2*(q.j*q.k-q.i*q.r), Ty,
                                  2*(q.i*q.k-q.j*q.r), 2*(q.j*q.k+q.i*q.r), 1-2*(q.i*q.i+q.j*q.j), Tz,
                                              0      ,           0        ,         0            ,  1};
  return transform*meas;
}

void HmgCalc(quaternion q) {  //calculate values of the mag obs matrix given state vector quats
  Hmg = { 0, 0, 0, 0, 0, 0, 0, 0, 0, atan2(2*(q.r*q.k+q.i*q.j),1-2*(q.j*q.j+q.k*q.k)),
          0, 0, 0, 0, 0, 0, 0, 0, 0,           asin(2*(q.r*q.j-q.i*q.k)),
          0, 0, 0, 0, 0, 0, 0, 0, 0, atan2(2*(q.r*q.i+q.j*q.k),1-2*(q.i*q.i+q.j*q.j))};
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
    x += Kmg*(zMg - Hmg*x_prior);
  }
  if(baroAvail) {
    x += Kbm*(zBm - Hbm*x_prior);
  }
  
  BLA::Matrix<10,10> I; I.Fill(0.);
  for(int i=0; i<10; i++) {   //identity matrix
    I(i,i) = 1.;
  }
  P = (I-Kbm*Hbm)*P_prior + (I-Kmg*Hmg)*P_prior;
}

void kalExtrapolate() {   //extrapolation / prediction function
  float psi = uGy(0);
  float tht = uGy(1);
  float phi = uGy(2);
  G = { dT*dT/2,    0   ,    0   ,         0       ,
           0   , dT*dT/2,    0   ,         0       ,
           0   ,    0   , dT*dT/2,         0       ,
           dT  ,    0   ,    0   ,         0       ,
           0   ,    dT  ,    0   ,         0       ,
           0   ,    0   ,    dT  ,         0       ,
           0   ,    0   ,    0   , dT*(sin(phi/2)*cos(tht/2)*cos(psi/2)-cos(phi/2)*sin(tht/2)*sin(psi/2)),
           0   ,    0   ,    0   , dT*(cos(phi/2)*cos(tht/2)*cos(psi/2)+sin(phi/2)*sin(tht/2)*sin(psi/2)),
           0   ,    0   ,    0   , dT*(cos(phi/2)*sin(tht/2)*cos(psi/2)+sin(phi/2)*cos(tht/2)*sin(psi/2)),
           0   ,    0   ,    0   , dT*(cos(phi/2)*cos(tht/2)*sin(psi/2)-sin(phi/2)*sin(tht/2)*cos(psi/2))};
  
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
  
  x_prior = (F*x + G*uXl);

  P_prior = F*P*~F;
}

// void writeToSD() {
//   file.write((uint8_t *)&data,sizeof(data)/sizeof(uint8_t));
// }

void KalmanFilter() {
  //extrapolate state, extrapolate uncert, compute gain, update estimate, update uncert
  kalExtrapolate();
  HmgCalc(quaternion {x(6),x(7),x(8),x(9)});
  kalmanGain();
  kalUpdate();
}

void KalmanInit() {   //initialize kalman matrices and orientation
  Rmg = { stddev_mg,     0    ,     0     ,
              0    , stddev_mg,     0     ,
              0    ,     0    , stddev_mg };
  Rbm = { stddev_ps };

  Hbm = { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};

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

  BLA::Matrix<3> gravVect = {aX, aY, aZ};
  gravVect /= sqrt(aX*aX+aY*aY+aZ*aZ);      //normalize gravity vector
  float psi = 90.-acosf(gravVect(1));
  float phi = 90.-acosf(gravVect(2));
  float theta = 0.;
  quaternion q;
  q.r = sin(phi/2)*cos(theta/2)*cos(psi/2)-cos(phi/2)*sin(theta/2)*sin(psi/2);    //convert initial orientation into state quaternion
  q.i = cos(phi/2)*cos(theta/2)*cos(psi/2)+sin(phi/2)*sin(theta/2)*sin(psi/2);
  q.j = cos(phi/2)*sin(theta/2)*cos(psi/2)+sin(phi/2)*cos(theta/2)*sin(psi/2);
  q.k = cos(phi/2)*cos(theta/2)*sin(psi/2)-sin(phi/2)*sin(theta/2)*cos(psi/2);

  //TODO: TRANSFORM MAG TO GLOBAL FRAME (there must be a way to do it without a transformation matrix, just think about it! You got this king)

  x = {0., 0., 0., 0., 0., 0., q.r, q.i, q.j, q.k};   //initial state estimate; position and velocity are zero, and orientation calculated above
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
  IMU.settings.gyro.scale = 500; //500dps
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
  delay(100); //make sure enough time has elapsed that all sensors will have an available measurement
  for(int i = 0; i<4; i++) {
    baroData(); //get an initial baro measurement
    delay(100);
  }
}
void SystemClock_Config(void) {   //set up STM32 PLL config. Code generated by STM32CubeMX software
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

void magCal() {   //for use with MotionCal software
  Serial.print("Raw:"); //Serial.print("0,0,0,0,0,0,");
  Serial.print((IMU.ax)/8); Serial.print(",");
  Serial.print((IMU.ay)/8); Serial.print(",");
  Serial.print((IMU.az)/8); Serial.print(",");
  Serial.print((IMU.gx-140)/8); Serial.print(",");
  Serial.print((IMU.gy-95)/8); Serial.print(",");
  Serial.print((IMU.gz-70)/8); Serial.print(",");
  Serial.print((IMU.mx)/8); Serial.print(",");
  Serial.print((IMU.my)/8); Serial.print(",");
  Serial.print((IMU.mz)/8); Serial.println("");
}