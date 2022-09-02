#pragma once

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


float ntc_getC(uint16_t adc_raw, uint32_t r_div=10000, 
               uint16_t betta=3950, uint16_t r_nom=10000, uint16_t t_nom=25, float max_t=99.9){
    float ntc_r = (r_div / (1023.0 / adc_raw - 1 ));
    float t = 1 / (1/(273.15+t_nom) + log(ntc_r / r_nom)/betta) - 273.15;
    if (t > max_t)
        t = max_t;
    return t;
}

uint16_t proportial(uint16_t input_val, uint16_t reg_val, uint16_t reg_val_low, uint16_t reg_val_high){
    float prcnt = (100 - map(reg_val, reg_val_low, reg_val_high, 1, 100)) / 100.0; 
    return input_val * prcnt;
}

#endif
