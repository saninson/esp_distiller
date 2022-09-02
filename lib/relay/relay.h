#pragma once

#ifndef relay_h
#define relay_h
#include <Arduino.h>

#define RL_ON   true
#define RL_OFF  false
typedef bool RelayState;

class Relay {
private:
    uint8_t _pin;
    RelayState _state;
public:
    Relay(uint8_t pin, RelayState init_state=RL_OFF);
    ~Relay(){};
    RelayState getState();
    void on();
    void off();
    void togle();
};
#endif