#include "Radio.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RH_RF95.h>
using namespace Eigen;

#define CS PC2
#define INT PC12

RHHardwareSPI spi;
RH_RF95 rf95(CS, INT, spi);

void Radio::init() {
  pinMode(PC0, OUTPUT); pinMode(PC1, OUTPUT);
  digitalWrite(PC0, HIGH); digitalWrite(PC1, HIGH); //pull other chip selects high
  pinModeAF(PB4,GPIO_AF5_SPI1); SPI.setMISO(PB4);
  pinModeAF(PB5,GPIO_AF5_SPI1); SPI.setMOSI(PB5);
  pinModeAF(PB3,GPIO_AF5_SPI1); SPI.setSCLK(PB3);
  spi.begin();

  rf95.init();
  rf95.setFrequency(915.0); //set frequency to 915MHz
  rf95.setTxPower(20,false); //set the transmit power to 20dBm using PA_BOOST
  rf95.setModemConfig(RH_RF95::Bw125Cr45Sf128);
}

void Radio::tx(Vector<float,10> x) {
  if(rf95.mode() != rf95.RHModeTx) {
    dataPacket = (packet){x(0),x(1),x(2),x(3),x(4),x(5),x(6),x(7),x(8),x(9)};
    char radiopacket[sizeof(dataPacket)];
    memcpy(radiopacket,&dataPacket,sizeof(dataPacket));
    rf95.send((uint8_t *)radiopacket, strlen(radiopacket));
  }
}

void Radio::pinModeAF(int ulPin, uint32_t Alternate) {
   int pn = digitalPinToPinName(ulPin);

   if (STM_PIN(pn) < 8) {
      LL_GPIO_SetAFPin_0_7( get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), Alternate);
   } else {
      LL_GPIO_SetAFPin_8_15(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), Alternate);
   }

   LL_GPIO_SetPinMode(get_GPIO_Port(STM_PORT(pn)), STM_LL_GPIO_PIN(pn), LL_GPIO_MODE_ALTERNATE);
}