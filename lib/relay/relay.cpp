#include "relay.h"

Relay::Relay(uint8_t pin, RelayState init_state){
    _pin = pin;
    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, init_state);
    _state = init_state;
}

RelayState Relay::getState(){
    return _state;
}

void Relay::on(){
    digitalWrite(_pin, RL_ON);
    _state = RL_ON;
}

void Relay::off(){
    digitalWrite(_pin, RL_OFF);
    _state = RL_OFF;
}

void Relay::togle(){
    _state = !_state;
    digitalWrite(_pin, _state);
}