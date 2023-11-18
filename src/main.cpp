#include <Arduino.h>
#include <SdFat.h>
#include "Kalman/KalmanFilter.h"
#include "Sensors/Sensors.h"
#include <BasicLinearAlgebra.h>
using namespace BLA;

Sensors sens;
KalmanFilter kalman = KalmanFilter(sens);

uint32_t prevTime;

void setup() {
  SerialUSB.begin(); //start serial port
  while(!SerialUSB);

  Wire.begin(uint32_t(PB9_ALT0),uint32_t(PB8_ALT0));
  sens.init();
  kalman.init();
  prevTime = micros();
  delay(5);
}

void loop() {
  float dT = (float)(micros()-prevTime)/1000000.f;
  prevTime = micros();
  kalman.filter(dT);
  Serial << kalman.x;
  Serial.println();
  if(!sens.IMU.gyroAvailable()) {
    delayMicroseconds(100);
  }
  //delay(5);
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