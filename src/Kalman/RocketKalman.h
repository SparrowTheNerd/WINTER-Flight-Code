#pragma once

#include <Arduino.h>

enum filterState {
    startup, looping, filtering
};

class RocketKalman {

public:

    RocketKalman(int randomnum);

    int init();

private:

    filterState state = startup;

    int currentRandomNum = 0;


};