#ifndef RADIO_H
#define RADIO_H
#include <ArduinoEigenDense.h>
using namespace Eigen;

class Radio {
public:
    void init();
    void tx(Vector<float,10>);

private:
    // Private members
    int cs;
    int interrupt;
    struct packet {
        float w,i,j,k,vX,vY,vZ,X,Y,Z;
    } dataPacket;
    void pinModeAF(int ulPin, uint32_t Alternate);
};

#endif // RADIO_H
