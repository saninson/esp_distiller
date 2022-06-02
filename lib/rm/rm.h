/*
    rm.h - library to interact with RMVK power regulator from Kulikov Techonology
    Created by Nikita Rovda
    email: nikita.rovda@yandex.ru
    2022.05
*/

#pragma once

#ifndef rm_h
#define rm_h

#include <Arduino.h>

#define RM_DEBUG

const uint8_t       RM_READ_TIMEOUT     = 10;
const uint8_t       RM_MIN_V            = 40;     // min output V
const uint8_t       RM_MAX_V            = 220;    // max output V
const uint16_t      RM_LOOP_INT         = 500;    // loop interval
const uint16_t      RM_FORCE_CHECK_INT  = 5000;
#define rmSerial                        Serial2

#ifdef RM_DEBUG
#ifndef debugSerial
#define debugSerial                     Serial
#endif
#endif


enum RMState {RM_OFF=0, RM_ON=1};
uint16_t f_P(uint16_t U, float R);

class RMVK{
    private:
        char _buf[15];                          // read/write buffer
        uint16_t _vi {0};                       // input voltage
        uint16_t _vo {0};                       // output voltage 
        uint16_t _dst_v {0};                    // output set point
        uint16_t _p {0};                        // calculated output power
        RMState _state {RM_OFF};                // current state (on/off)
        bool _do_check_state {true};            // force check state flag
        bool _do_check_vo {true};               // force check output voltage flag
        float _load_R {16.13};                  // load resistance default 16,13 Ohm for 3kW heater
        uint32_t _last_get_state {0};           // last _getState exec millis
        uint32_t _last_get_vo {0};              // last _getVo exec millis
        uint16_t _maxP;                         // max power calculated by load resistance
        uint16_t _readBytes(uint8_t count);
        uint16_t _write(const char *buf);
        uint16_t _getVi();
        uint16_t _getVo();
        uint16_t _setVo(uint16_t new_v);
        RMState _setState(RMState state);
        RMState _getState();
    public:
        ~RMVK();
        RMVK();
        void begin(float load_R);
        void loop();
        uint16_t getVi();
        uint16_t getVo();
        uint16_t getP();
        uint16_t getMaxP();
        bool setVo(uint16_t dst_v);
        void switch_on();
        void switch_off();
        RMState getState();
};

#endif
