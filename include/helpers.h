#ifndef helpers_h
#define helpers_h

#include <Arduino.h>

class FilterExpRunningAverage{
private:
    float _fil_val;
public:
    FilterExpRunningAverage(float init_val=0){
        _fil_val = init_val;
    }
    ~FilterExpRunningAverage(){};
    
    float filter(float new_val){
      float k = abs(new_val - _fil_val) / 100.0;
      if (k > 1) 
        k = 1.0;
      else if (k < 0.1) 
        k = 0.1; 
      _fil_val += (new_val - _fil_val) * k;
      return _fil_val;  
    }
};


enum RelayState {RL_OFF=0, RL_ON=1};

class Relay {
private:
    uint8_t _pin;
    RelayState _state;
public:
    Relay(uint8_t pin, RelayState init_state=RL_OFF){
        _pin = pin;
        pinMode(_pin, OUTPUT);
        digitalWrite(_pin, init_state);
        _state = init_state;
    }
    ~Relay(){};
    RelayState getState(){
        return _state;
    }
    void on(){
        digitalWrite(_pin, RL_ON);
        _state = RL_ON;
    }
    void off(){
        digitalWrite(_pin, RL_OFF);
        _state = RL_OFF;
    }
    void togle(){
        _state = _state==RL_ON?RL_OFF:RL_ON;
        digitalWrite(_pin, (bool)_state);
    }

};


float ntc_getC(uint16_t adc_raw, uint32_t r_div=10000, 
               uint16_t betta=3950, uint16_t r_nom=10000, uint16_t t_nom=25, float max_t=99.9){
    float ntc_r = (r_div / (1023.0 / adc_raw - 1 ));
    float t = 1 / (1/(273.15+t_nom) + log(ntc_r / r_nom)/betta) - 273.15;
    if (t > max_t)
        t = max_t;
    return t;
}

#endif
