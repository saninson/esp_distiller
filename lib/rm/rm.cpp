/*
    rm.cpp - library to interact with RMVK power regulator from Kulikov Techonology
    Created by Nikita Rovda
    email: nikita.rovda@yandex.ru
    2022.05
*/

#include "rm.h"

uint16_t f_P(uint16_t U, float R){
    return round(U * U / R);
}

uint16_t f_V(uint16_t P, float R){
    return round(sqrt(P * R));
}

void debug(const char* msg, bool nl=true){
    #ifdef RM_DEBUG
    static bool prev_nl = true;
    if(prev_nl){
        debugSerial.print(millis());
        debugSerial.print(" RM: ");
    }
    debugSerial.print(msg);
    if(nl)
        debugSerial.println();
    prev_nl = nl;
    #endif
}


RMVK::~RMVK(){
}

RMVK::RMVK(){
}

uint16_t RMVK::_readBytes(uint8_t length){
    debug("readBytes: '", false);
    uint16_t n = rmSerial.readBytes(_buf, length);
    _buf[n] = '\0';
    if(n < length){
        debug("warning: expected ", false);
        debug(String(length).c_str(), false);
        debug(" bytes, got ", false);
        debug(String(n).c_str(), false);
        debug(" bytes '", false);
    }
    debug(_buf, false);
    debug("'");
    // clear rest bytes
    while (rmSerial.available())
        rmSerial.read();
    return n;
}

uint16_t RMVK::_write(const char *buf){
    debug("write: ", false);
    debug(buf);
    uint16_t n = rmSerial.print(buf);
    rmSerial.flush();
    delay(7);
    return n;
}

// read RM input voltage
uint16_t RMVK::_getVi(){
    _write("AT+VI?\r");
    if (!_readBytes(3))
        _vi = 0;
    else
        _vi = String(_buf).toInt();
    if(_vi < _vo)
        _do_check_vo = true;
    return _vi;
}

uint16_t RMVK::_getVo(){
    _write("AT+VO?\r");
    if (!_readBytes(3)){
        _vo = 0;
        _do_check_vo = true;
    } else{
        _vo = String(_buf).toInt();
        _do_check_vo = false;
    }
    _last_get_vo = millis();
    return _vo;
}

RMState RMVK::_getState(){
    _write("AT+ON?\r");
    if (!_readBytes(2)){
        _do_check_state = true;
        return _state;
    }
    if (_buf[1] == 'N')
        _state = RM_ON;
    else if (_buf[1] == 'F')
        _state = RM_OFF;
    _do_check_state = false;
    _last_get_state = millis();
    return _state;
}

uint16_t RMVK::_setVo(uint16_t new_v){
    char cmd[11];
    sprintf(cmd, "AT+VS=%03d\r", new_v);
    _write(cmd);
    if (!_readBytes(3) || _buf[0] == 'e'){
        _do_check_vo = true;
        return _vo;
    }
    _vo = new_v;
    _do_check_vo = false;
    return _vo;
}

RMState RMVK::_setState(RMState new_state){
    char cmd[9];
    sprintf(cmd, "AT+ON=%d\r", new_state);
    _write(cmd);
    if (!_readBytes(2)){
        return _state;
        _do_check_state = true;
    }

    _state = new_state;
    if (_state == RM_OFF)
        _vo = 0;
    _do_check_state = false;
    return _state;
}

void RMVK::loop(){
    static uint32_t ms;
    if (millis() - ms < RM_LOOP_INT)
        return;

    debug("=== loop start ===");
    // update state when need or by interval
    if(_do_check_state || millis() - _last_get_state > RM_FORCE_CHECK_INT)
        _getState();

    // always check input voltage
    _getVi();

    // update output voltage when need or by interval
    if(_do_check_vo || millis() - _last_get_vo > RM_FORCE_CHECK_INT)
        _getVo();

    if(_vi < RM_MIN_V && _state != RM_OFF)
        debug("RM.Vi error: Vi < RM_MIN_V while RM_ON");

    // turn on/off by voltage set point
    if(_dst_v < RM_MIN_V && _state == RM_ON)
        _setState(RM_OFF);
    else if (_dst_v >= RM_MIN_V && _state == RM_OFF)
        _setState(RM_ON);
    
    if(_state == RM_ON){
        if(_dst_v != _vo && _dst_v <= _vi)
            _setVo(_dst_v);
        _p = f_P(_vo, _load_R);
    } else 
        _p = 0;
    debug("=== loop end ===\n");
    ms = millis();
}

void RMVK::begin(float load_R){
    rmSerial.begin(9600);
    rmSerial.flush();
    rmSerial.setTimeout(RM_READ_TIMEOUT);
    _load_R = load_R;
    _maxP = f_P(RM_MAX_V, _load_R);
    _setState(RM_OFF);  // set initial state: off
}

uint16_t RMVK::getVi(){
    // debug("getVi: ", false);
    // debug(String(_vi).c_str());
    return _vi;
}

uint16_t RMVK::getVo(){
    return _vo;
}

uint16_t RMVK::getP(){
    return _p;
}

bool RMVK::setVo(uint16_t dst_v){
    if(dst_v > RM_MAX_V)
        return false;
    _dst_v = dst_v;
    return true;
};

void RMVK::switch_off(){
    _setState(RM_OFF);
}

void RMVK::switch_on(){
    _setState(RM_ON);
}

RMState RMVK::getState(){
    return _state;
}

uint16_t RMVK::getMaxP(){
    return _maxP;
}
