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
#define stddev_ps   400.    //400Pa (around 10ish feet I think) from MS5607 datasheet
//model stddev (~1/inertia)
#define m_pos       0.1
#define m_vel       0.1
#define m_ang       0.5
#define m_rol       0.5

BLA::Matrix<3> obsXl; // obs vector for accel
BLA::Matrix<3> obsGy; // obs vector for gyros
BLA::Matrix<3> obsMg; // obs vector for magns
BLA::Matrix<1> obsPs; // obs vector for press

Adafruit_ADXL375 IMU_HighG = Adafruit_ADXL375(1,&Wire);
MS5xxx barometer(&Wire);
LSM9DS1 IMU;
SdFat SD;
File file;

void imuInit();
void barometerInit();
void getSensorData();
uint32_t Time, timeBaro;
bool magAvail;  //is there mag data available?
bool baroAvail; //is there baro data available?
char fileName[] = "FLIGHTDATA00.bin";
float dT;
float angX, angY, angZ;
struct sensData {   
  float time, gX, gY, gZ, aX, aY, aZ, mX, mY, mZ, prs, tmp;
} data;

struct quaternion {
  float i, j, k;    //imaginary values
  float r;          //real value
};

BLA::Matrix<3,1> magCal_hard = {-19.03*8,28.14*8,-66.63*8}; //hard and soft iron calibrations
BLA::Matrix<3,3> magCal_soft = { 0.961, 0.026,-0.024,
                                 0.026, 1.010,-0.002,
                                -0.024,-0.002, 1.032};
BLA::Matrix<3,1> mag;
BLA::Matrix<3,1> magTemp;

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

  Time = micros();
  timeBaro = micros();

}

void loop() {
  getSensorData();
  delay(10);
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
    data.prs = barometer.GetPres();
    data.tmp = barometer.GetTemp();
    baroStep = 0;
    break;
  }
}

void getSensorData() {
  baroData();
  //time = (float)micros()/1000000.;    //elapsed time in seconds
  dT = (float)(micros()-Time)/1000000.;
  Time = micros();
  if (IMU.gyroAvailable()) {
    IMU.readGyro();
    data.gX = IMU.calcGyro(IMU.gx) - gXOfst;
    data.gY = IMU.calcGyro(IMU.gy) - gYOfst;
    data.gZ = IMU.calcGyro(IMU.gz) - gZOfst;
  }
  if (IMU.accelAvailable()) {
    IMU.readAccel();
    data.aX = IMU.calcAccel(IMU.ax);
    data.aY = IMU.calcAccel(IMU.ay);
    data.aZ = IMU.calcAccel(IMU.az);
  }
  if (IMU.magAvailable()) {   //using mag calibration, eq ref https://www.digikey.com/en/maker/projects/how-to-calibrate-a-magnetometer/50f6bc8f36454a03b664dca30cf33a8b
    IMU.readMag();
    magTemp = {(float)IMU.mx-magCal_hard(0),(float)IMU.my-magCal_hard(1),(float)IMU.mz-magCal_hard(2)};
    mag = magCal_soft * magTemp;
    data.mX = IMU.calcMag(mag(0));
    data.mY = IMU.calcMag(mag(1));
    data.mZ = IMU.calcMag(mag(2));
    magAvail = true;
  }
}

void writeToSD() {
  file.write((uint8_t *)&data,sizeof(data)/sizeof(uint8_t));
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