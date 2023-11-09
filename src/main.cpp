#include <Arduino.h>
#include <SdFat.h>
#include "Kalman/KalmanFilter.h"
#include "Sensors/Sensors.h"

#define BZR     PC8
#define CS_SD   PC0
#define gXOfst 1.301940492
#define gYOfst 0.859327296
#define gZOfst 0.528033635
//measurement stddevs
#define stddev_xl   (float)0.180   //180 milli G's from LSM9DS1 datasheet
#define stddev_gy   (float)2.0     //2 deg/s, roughly measured
#define stddev_mg   (float)0.01     //1 Gauss from LSM9DS1 datasheet, will improve post calibration
#define stddev_ps   (float)3.05     //400Pa, around 10ft (3.05m) from MS5607 datasheet

SdFat SD;
File file;

uint32_t Time;
bool magAvail;  //is there mag data available?
bool baroAvail = false; //is there baro data available?
char fileName[] = "FLIGHTDATA00.bin";
float dT;
struct sensData {   
  float time, gX, gY, gZ, aX, aY, aZ, mX, mY, mZ, prs, tmp;
} data;


Matrix<3,1> magCal_hard = {-31.84f*8.f,51.43f*8.f,42.51f*8.f}; //hard and soft iron calibrations
Matrix<3,3> magCal_soft = { 0.985f, 0.033f,-0.010f,
                            0.033f, 1.005f, 0.017f,
                           -0.010f, 0.008f, 1.011f};

float degToRad(float deg) {
  return deg*DEG_TO_RAD;
}
float radToDeg(float rad) {
  return rad*RAD_TO_DEG;
}

Sensors sens = Sensors(magCal_hard,magCal_soft);
KalmanFilter kalman = KalmanFilter(sens);
void magCal();
void setup() {
  pinMode(BZR,OUTPUT);
  analogWriteResolution(16);  //set pwm resolution to 16 bit (0-65535)
  analogWriteFrequency(1000); //set pwm frequency to 1KHz

  SerialUSB.begin(); //start serial port
  while(!SerialUSB);

  Wire.begin(uint32_t(PB9_ALT0),uint32_t(PB8_ALT0));

  sens.init();    //initialize sensors
  sens.timeBaro = micros();
  kalman.init(stddev_mg, stddev_ps);
  Serial << kalman.x;
  Time = micros();
  delay(1);
}

int counter = 0;
void loop() {
  dT = (float)(micros()-Time)/1000000.;
  Time = micros();
  sens.getData();
  kalman.filter(dT);
  //magCal();
  delay(10);
  sens.magAvail = false; sens.baroAvail = false;  //reset sensor availability before next loop
  // Serial << kalman.x;
  // Serial.println(" ");
}

// void writeToSD() {
//   file.write((uint8_t *)&data,sizeof(data)/sizeof(uint8_t));
// }

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
//   Serial.print((int)((sens.aX)/8)); Serial.print(",");
//   Serial.print((int)((sens.aY)/8)); Serial.print(",");
//   Serial.print((int)((sens.aZ)/8)); Serial.print(",");
//   Serial.print((int)((sens.gX)/8-20)); Serial.print(",");
//   Serial.print((int)((sens.gY)/8-18)); Serial.print(",");
//   Serial.print((int)((sens.gZ)/8-8)); Serial.print(",");
//   Serial.print((int)((sens.mX)/8)); Serial.print(",");
//   Serial.print((int)((sens.mY)/8)); Serial.print(",");
//   Serial.print((int)((sens.mZ)/8)); Serial.println("");
// }