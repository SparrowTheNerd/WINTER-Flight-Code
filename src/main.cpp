#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>

#define CS PC2
#define INT PC12

RHHardwareSPI spi;
RH_RF95 rf95(CS,INT,spi);

void pinModeAF(int ulPin, uint32_t Alternate);
uint32_t start,end;

struct packet {
  float float1,float2,float3;
} dataPacket;

void setup() 
{

  pinMode(PC0, OUTPUT); pinMode(PC1, OUTPUT);
  digitalWrite(PC0, HIGH); digitalWrite(PC1, HIGH); //pull other chip selects high
  pinModeAF(PB4,GPIO_AF5_SPI1); SPI.setMISO(PB4);
  pinModeAF(PB5,GPIO_AF5_SPI1); SPI.setMOSI(PB5);
  pinModeAF(PB3,GPIO_AF5_SPI1); SPI.setSCLK(PB3);
  spi.begin();

  SerialUSB.begin(); //start serial port
  while(!SerialUSB);
  if (!rf95.init())
    Serial.println("init failed");  
  else {Serial.println("init success");}
  rf95.setFrequency(915.0); //set frequency to 915MHz
  rf95.setTxPower(20,false); //set the transmit power to 20dBm using PA_BOOST
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // You can change the modulation parameters with eg
  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

float randomFloat() { return (float)(rand()) / (float)(rand()); } //random float generator

void loop() {
  delay(100);
  if(rf95.mode() != rf95.RHModeTx) {
    dataPacket = (packet){randomFloat(),randomFloat(),randomFloat()};
    char radiopacket[sizeof(dataPacket)];
    memcpy(radiopacket,&dataPacket,sizeof(dataPacket));
    Serial.print("Sending ");
    Serial.print(dataPacket.float1,5); Serial.print("  "); Serial.print(dataPacket.float2,5); Serial.print("  "); Serial.println(dataPacket.float3,5);
    // Send a message!
    start = micros();
    rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
    rf95.waitPacketSent();
    end = micros()-start;
    Serial.print("dT: "); Serial.println((float)end/1000000.f,6);
  }
  else delay(50);
}

void pinModeAF(int ulPin, uint32_t Alternate)
{
   int pn = digitalPinToPinName(ulPin);

   if (STM_PIN(pn) < 8) {
      LL_GPIO_SetAFPin_0_7( get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), Alternate);
   } else {
      LL_GPIO_SetAFPin_8_15(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), Alternate);
   }

   LL_GPIO_SetPinMode(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), LL_GPIO_MODE_ALTERNATE);
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