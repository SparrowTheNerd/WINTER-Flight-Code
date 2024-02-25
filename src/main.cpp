#include <Arduino.h>
#include <SdFat.h>
#include "Kalman/KalmanFilter.h"
#include "Sensors/Sensors.h"
#include "Radio/Radio.h"
#include <ArduinoEigenDense.h>
#define EIGEN_NO_DEBUG 1

#define HAS_RADIO

using namespace Eigen;

#define latitude 42.262119f //worcester
//#define latitude 38.804661  //virginia
void magCal();
void print_matrix(const Eigen::MatrixXf &X);

Vector3f magCal_hard = {2.24222e3f, 1.0018e3f, 0.9265e3f}; //hard and soft iron calibrations from matlab
Matrix<float,3,3> magCal_soft   {{ 0.9846f, 0.0401f,-0.0099f},
                                 { 0.0401f, 0.9890f,-0.0099f},
                                 {-0.0099f,-0.0099f, 1.0288}};  //worcester
Vector3f accelHard {0.408420f, -0.078680f, 0.130302f};
Matrix3f accelSoft {{1.011042f, -0.007126f, 0.000493f},
                    {-0.007126f, 1.015386f, -0.008700f},
                    {0.000493f, -0.008700f, 1.001993f}};
Sensors sens = Sensors(magCal_hard,magCal_soft, accelHard, accelSoft);
KalmanFilter kalman = KalmanFilter(sens, latitude);
Radio radio;

uint32_t prevTime;

void setup() {
  SerialUSB.begin(); //start serial port
  while(!SerialUSB);

  Wire.begin(uint32_t(PB9_ALT0),uint32_t(PB8_ALT0));
  sens.init();
  kalman.init();
  radio.init();

  prevTime = micros();
}

void loop() {
  while(!sens.IMU.gyroAvailable()) {   //wait for data
    delayMicroseconds(100);
  }
  sens.getData();
  float dT = (float)(micros()-prevTime)/1e6f;
  sens.dt = dT;
  prevTime = micros();
  
  kalman.filter(dT);
  #ifdef HAS_RADIO
    radio.tx(kalman.x);
  #endif
  // print_matrix(kalman.x);
  sens.magAvail = false;
  sens.baroAvail = false;
}

void print_matrix(const Eigen::MatrixXf &X)  
{
  int nrow = X.rows();
  int ncol = X.cols();
  Serial.print("[");
  for (int i=0; i<nrow; i++) {   
    Serial.print("[");
    for (int j=0; j<ncol; j++) {
      Serial.print(X(i,j), 5);   // print 6 decimal places
      if(j<ncol-1) Serial.print(",");
    }
    Serial.print("]");
    if(i<nrow-1) Serial.print(",");
  }
  Serial.println("]");
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

void magCal() {   //for use with MotionCal software
  Serial.print("Raw:"); //Serial.print("0,0,0,0,0,0,");
  Serial.print((sens.IMU.ax)/8); Serial.print(",");
  Serial.print((sens.IMU.ay)/8); Serial.print(",");
  Serial.print((sens.IMU.az)/8); Serial.print(",");
  Serial.print((sens.IMU.gx-140)/8); Serial.print(",");
  Serial.print((sens.IMU.gy-95)/8); Serial.print(",");
  Serial.print((sens.IMU.gz-70)/8); Serial.print(",");
  Serial.print((sens.IMU.mx)/8); Serial.print(",");
  Serial.print((sens.IMU.my)/8); Serial.print(",");
  Serial.print((sens.IMU.mz)/8); Serial.println("");
}
