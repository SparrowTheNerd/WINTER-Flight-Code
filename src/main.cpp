#include <Arduino.h>
#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <MS5xxx.h>
#include <SD.h>

#define BZR     PC8
#define CS_SD   PC0

Adafruit_ADXL375 IMU_HighG = Adafruit_ADXL375(1,&Wire);
MS5xxx barometer(&Wire);
LSM9DS1 IMU;

void imuInit();
void barometerInit();
void getSensorData();
float gX, gY, gZ, aX, aY, aZ, mX, mY, mZ, prs, tmp, prevPrs;
uint32_t time, timeBaro;
bool magAvail;  //is there mag data available?
bool freshBaro; //is the barometer measurement fresh?

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

  //SD.begin(CS_SD);
  time = micros();
  timeBaro = micros(); 
}

void loop() {
  getSensorData();
  // Serial.print(gX); Serial.print(","); Serial.print(gY); Serial.print(","); Serial.print(gZ); Serial.print(" , "); 
  // Serial.print(aX); Serial.print(","); Serial.print(aY); Serial.print(","); Serial.print(aZ); Serial.print(" , ");
  // if(magAvail) {
  //   Serial.print(mX); Serial.print(","); Serial.print(mY); Serial.print(","); Serial.print(mZ); Serial.print(" , ");
  //   magAvail = false;
  // }
  // else{ Serial.print("     ,    ,     , "); }
  // if(prs != prevPrs) {
  //   Serial.println(prs);
  //   prevPrs = prs;
  // }
  // else { Serial.println(""); }
  // delayMicroseconds(1000);
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
    break;
  }
}

void getSensorData() {
  baroData();
  time = micros();
  if (IMU.gyroAvailable()) {
    
    IMU.readGyro();
    gX = IMU.calcGyro(IMU.gx);
    gY = IMU.calcGyro(IMU.gy);
    gZ = IMU.calcGyro(IMU.gz);

  }
  float dt = (micros()-time)/1000.;
  Serial.print(dt,3); Serial.println(",");
  if (IMU.accelAvailable()) {
    IMU.readAccel();
    aX = IMU.calcAccel(IMU.ax);
    aY = IMU.calcAccel(IMU.ay);
    aZ = IMU.calcAccel(IMU.az);
  }
  if (IMU.magAvailable()) {
    IMU.readMag();
    mX = IMU.calcMag(IMU.mx);
    mY = IMU.calcMag(IMU.my);
    mZ = IMU.calcMag(IMU.mz);
    magAvail = true;
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