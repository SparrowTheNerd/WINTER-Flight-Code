#include <Arduino.h>
#include <SdFat.h>
#include "Kalman/KalmanFilter.h"
#include "Sensors/Sensors.h"
//#include <BasicLinearAlgebra.h>
#include <ArduinoEigenDense.h>
//using namespace BLA;
#define EIGEN_NO_DEBUG 1
using namespace Eigen;

#define latitude 42.262119f //worcester
//#define latitude 38.804661  //virginia
void magCal();
void print_matrix(const Eigen::MatrixXf &X);

//Matrix<3,1> magCal_hard = {-0.f, 0.f, 0.f}; //hard and soft iron calibrations in WPI
// Vector<float,3> magCal_hard {{ -4061.74f, 1434.475f, -8685.29f}};
// Matrix<float,3,3> magCal_soft {{ 1.015f, 0.008f,-0.004f},
//                                { 0.033f, 0.990f,-0.002f},
//                                {-0.004f,-0.002f, 0.996f}};   //virginia
Vector3f magCal_hard = {2.24222e3f, 1.0018e3f, 0.9265e3f}; //hard and soft iron calibrations from matlab
Matrix<float,3,3> magCal_soft   {{ 0.9846f, 0.0401f,-0.0099f},
                                 { 0.0401f, 0.9890f,-0.0099f},
                                 {-0.0099f,-0.0099f, 1.0288}};  //worcester
Vector3f accelBias {-0.1587, 0.0346, -0.5615};
Sensors sens = Sensors(magCal_hard,magCal_soft, accelBias);
KalmanFilter kalman = KalmanFilter(sens, latitude);

uint32_t prevTime;

void setup() {
  SerialUSB.begin(); //start serial port
  while(!SerialUSB);

  Wire.begin(uint32_t(PB9_ALT0),uint32_t(PB8_ALT0));
  sens.init();
  kalman.init();
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
  print_matrix(kalman.x);
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