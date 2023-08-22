#include <Arduino.h>
#include <Wire.h>
#include <SparkFunLSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <MS5xxx.h>
#include <SdFat.h>

#define BZR     PC8
#define CS_SD   PC0

Adafruit_ADXL375 IMU_HighG = Adafruit_ADXL375(1,&Wire);
MS5xxx barometer(&Wire);
LSM9DS1 IMU;
SdFat SD;
File file;

void imuInit();
void barometerInit();
void getSensorData();
void quatCalcs();
uint32_t Time, timeBaro;
bool magAvail;  //is there mag data available?
bool baroAvail; //is there baro data available?
char fileName[] = "FLIGHTDATA00.bin";
float dT;
int quatNormCounter = 0;
uint32_t quatTimer;
float angX, angY, angZ;

struct sensData {   
  float time, gX, gY, gZ, aX, aY, aZ, mX, mY, mZ, prs, tmp;
} data;

struct quaternion {
  float i, j, k;    //imaginary values
  float r;          //real value
} masterQuat;

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

  masterQuat = {.i=0, .j=0, .k=0, .r=1};
  quatTimer = micros();
}

void quatAngles() {
  quaternion p = masterQuat;
  if (1-2*(p.i*p.i+p.j*p.j) != 0) {
    angX = atan(2*(p.r*p.i+p.j*p.k)/(1-2*(p.i*p.i+p.j*p.j)));
  } 
  else {
    if((p.r*p.i+p.j*p.k) > 0) {
      angX = M_PI / 2.0f;
    } else {
      if((p.r*p.i+p.j*p.k) < 0) {
        angX = -1.0f * M_PI / 2.0f;
      } else {
        angX = 9999999;
        // SIGNAL ERROR DIVIDE BY ZERO!!!
      }
    }
  }
  // Convert x (roll) from radian to degrees
  angX = angX * 180.0f / M_PI;


  //Compute the Y (pitch) angle in radians
  if((2*(p.i*p.j-p.k*p.i)) <= -1) {
    angY = -1.0f * M_PI / 2.0f;
  } else {
    if((2*(p.r*p.j-p.k*p.i)) >= 1) {
      angY = M_PI / 2.0f;
    } else {
      angY = asin(2*(p.r*p.j-p.k*p.i));
    }
  }
  // Convert y (pitch) from radian to degrees
  angY = angY * 180.0f / M_PI; 


  // Compute the Z (Yaw) angle in radians
  if((1-2*(p.i*p.i+p.j*p.j)) != 0) {
     angZ = atan(2*(p.r*p.k+p.i*p.j)/(1-2*(p.j*p.j+p.k*p.k)));
   } else {
    if((p.r*p.k+p.i*p.j) > 0) {
      angZ = M_PI / 2.0f;
    } else {
      if((p.r*p.k+p.i*p.j) < 0) {
        angZ = -1.0f * M_PI / 2.0f;
      } else {
        angZ = 9999999;
        // SIGNAL ERROR DIVIDE BY ZERO!!!
      }
    }
  }
  // Convert z (Yaw) from radian to degrees
  angZ = angZ * 180.0f / M_PI;
}

void loop() {
  getSensorData();
  quatCalcs();
  if(micros()-quatTimer >= (0.1*1000000.)) {    //output angles every 0.1s to check
    quatAngles();
    Serial.print(angX); Serial.print(","); Serial.print(angY); Serial.print(","); Serial.println(angZ);
  }
}

void quaternionMultiply(quaternion t) {
  // combine t with the masterQuat to get an integrated rotation
  quaternion p = masterQuat;
  masterQuat.r = (p.r * t.r) + (-p.i * t.i) + (-p.j * t.j) + (-p.k * t.k);
  masterQuat.i = (p.r * t.i) + (p.i * t.r) + (p.j * t.k) + (-p.k * t.j);
  masterQuat.j = (p.r * t.j) + (-p.i * t.k) + (p.j * t.r) + (p.k * t.i);
  masterQuat.k = (p.r * t.k) + (p.i * t.j) + (-p.j * t.i) + (p.k * t.r);

}
void quatCalcs() {
  quaternion q;
  q.r = 1;
  q.i = data.gX * dT / 2.;
  q.j = data.gY * dT / 2.;
  q.k = data.gZ * dT / 2.;
  quaternionMultiply(q);
  if (quatNormCounter%50==0) {
    float quatSize = (masterQuat.r*masterQuat.r) + (masterQuat.i*masterQuat.i) + (masterQuat.j*masterQuat.j) + (masterQuat.k*masterQuat.k);
    if (quatSize > 1.) {
      float normFactor = 1.0 / sqrtf(quatSize);
      masterQuat.r *= normFactor;
      masterQuat.i *= normFactor;
      masterQuat.j *= normFactor;
      masterQuat.k *= normFactor;
    }
  }
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
  data.time = (float)micros()/1000000.;    //elapsed time in seconds
  dT = (float)(micros()-Time)/1000000.;
  Time = micros();
  if (IMU.gyroAvailable()) {
    IMU.readGyro();
    data.gX = IMU.calcGyro(IMU.gx);
    data.gY = IMU.calcGyro(IMU.gy);
    data.gZ = IMU.calcGyro(IMU.gz);
  }
  if (IMU.accelAvailable()) {
    IMU.readAccel();
    data.aX = IMU.calcAccel(IMU.ax);
    data.aY = IMU.calcAccel(IMU.ay);
    data.aZ = IMU.calcAccel(IMU.az);
  }
  if (IMU.magAvailable()) {
    IMU.readMag();
    data.mX = IMU.calcMag(IMU.mx);
    data.mY = IMU.calcMag(IMU.my);
    data.mZ = IMU.calcMag(IMU.mz);
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