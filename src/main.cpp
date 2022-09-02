#define WITH_NETWORK

#include <LiquidCrystal_I2C.h>
#include <Adafruit_MCP3008.h>
#include <helpers.h>
#include <Thermistor.h>
#include <NTC_Thermistor.h>
#include <ESP32Encoder.h>
#include <math.h>
#include <buzzer.h>
#include <rm.h>
#include <Button2.h>
#include <relay.h>
// #ifdef WITH_NETWORK
// #include "net.h"
// #endif

#ifdef WITH_NETWORK
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
const char* ssid = "FRANTSOVA";
const char* password = "218506oleg";
#endif

// MCP3008 channels
#define ADC_T_COLUMN            0       // temperature in column 
#define ADC_T_TANK              1       // temperature in tank
#define ADC_T_FRIDGE            2       // temp of dstPut for stage 1, temp of out water for stage 2
#define ADC_POT                 4       // potentiometr

// IO pins
#define ENC_A_PIN               34      // encoder CLK
#define ENC_B_PIN               35      // encoder CSW
#define ENC_BTN_PIN             32      // encoder button
#define ALC_VALVE_REL           33      // alcohol valve relay pin
#define BUZZER_PIN              25      // buzzer
//#define COLD_VALVE_REL        31      // cold water valve relay pin

// Refresh intervals
#define RI_LCD                  100     // LCD 
#define RI_ADC                  50      // MCP reading interval
#define RI_ENC                  500     // encoder process interval
#define RI_PROG                 100     // current operation mode loop interval

const uint8_t COLUMN_HOT_TEMP = 45;     // temp to consider column as hot
const uint8_t TANK_WARM_TEMP =  55;     // temp to consider tank as warm

struct {
    float tank, column, fridge;
} temp;

uint16_t dstU;                  // voltage set point
uint16_t dstP;                  // real dstPut power
float dst_percent;               
float heater_R = 16.13;         // heater resistance in Ohms 
uint16_t pot;                   // potentiometr raw value
uint32_t last_pot_ms;
float dst_t2 = 77.0;            // destination column temp for valve control
uint32_t dst_t2_touch;          // last change time for use in lcd_print

enum OperationMode{
    MODE_MANUAL = 0,            // manual power control 
    MODE_A0 = 1,                // quick warm up program
    MODE_A1 = 2,                // first stage program
    MODE_A2 = 3,                // second stage program
    MODE_OFF = 4,               // power off
    SWITCH_NEXT = 5
} operation_mode;

enum DisplayContext{
    CONT_MAIN, 
    CONT_MENU
} display_context;

LiquidCrystal_I2C lcd(0x27,16,2);
Adafruit_MCP3008 adc;
ESP32Encoder encoder;
Buzzer buzzer(BUZZER_PIN);
RMVK rm;
Relay alc_valve(ALC_VALVE_REL);
Button2 enc_btn;

// menu
// Menu menu;
// enum DistMenuItems{
//     MI_ROOT,
//         MI_SETT_,
//             MI_SETT_HR,
//             MI_SETT_MAXP,
//         MI_JOB_,
//             MI_JOB_MODE,
//             MI_JOB_ENDT1
// };

// const char CAPT_MI_SETT_[] PROGMEM = "settings";
// const char      CAPT_MI_SETT_HR[] PROGMEM = "heater R:";
// const char      CAPT_MI_SETT_MAXP[] PROGMEM = "max output P:";
// const char CAPT_MI_JOB_[] PROGMEM = "job";
// const char      CAPT_MI_JOB_MODE[] PROGMEM = "mode:";
// const char      CAPT_MI_JOB_ENDT1[] PROGMEM = "end temp:";


void lcd_print_operation_vals(){
    // 1234567890123456
    // xx.x xx.x xx x v         // t tank, t column, t fridge, mode, 
    // xxxV xxxxW xxx.x         // input voltage, actual output power, max power percentage 
    static uint32_t ms;
    if ((millis() - ms) < RI_LCD)
        return;

    // show dst_t2 for 3 sec if we change it 
    if (millis() - dst_t2_touch < 3000){
        lcd.clear();
        lcd.print("valve temp: ");
        lcd.print(dst_t2);
        lcd.print("   ");
        ms = millis();
        return;
    }
    // else show operation values
    // first line
    lcd.setCursor(0, 0);
    char row[20];
    char str_temp_tank[] = "00.0";
    char str_temp_column[] = "00.0";
    char str_temp_fridge[] = "00.0";
    dtostrf(temp.tank, 4, 1, str_temp_tank);
    dtostrf(temp.column, 4, 1, str_temp_column);
    dtostrf(temp.fridge, 2, 0, str_temp_fridge);
    char om_symb[] = "M012  ";
    sprintf(row, "%4s %4s %2s %c %c", 
        str_temp_tank, str_temp_column, str_temp_fridge, 
        om_symb[operation_mode], (alc_valve.getState()==RL_OFF) ? 'x':'o'
    );
    lcd.print(row);

    // second line
    lcd.setCursor(0,1);
    char str_dst_percent[] = "000";
    dtostrf(dst_percent, 3, 0, str_dst_percent);
    sprintf(row, "%3dV %4dW %s%% ", 
        rm.getVi(), rm.getP(), str_dst_percent);
    lcd.print(row);

    ms = millis();
}


// read and convert values on mcp3008 ADC
void adc_read(){
    static uint32_t ms;
    if ((millis() - ms) < RI_ADC)
        return;

    pot = adc.readADC(ADC_POT);
    dstU = map(pot, 0, 1023, 38, 220);
    if (dstU < RM_MIN_V)
        dstU = 0;
    dstP = f_P(dstU, heater_R);
    dst_percent = map(dstP, 0, rm.getMaxP(), 0, 1000)/10.0;
    
    static FilterExpRunningAverage filter_column, filter_tank, filter_fridge;
    temp.column = filter_column.filter(ntc_getC(adc.readADC(ADC_T_COLUMN)));
    temp.tank = filter_tank.filter(ntc_getC(adc.readADC(ADC_T_TANK)));
    temp.fridge = filter_fridge.filter(ntc_getC(adc.readADC(ADC_T_FRIDGE)));

    ms = millis();
}

void switch_operation_mode(OperationMode new_mode=SWITCH_NEXT){
    if (new_mode == SWITCH_NEXT){
        switch (operation_mode){
        case MODE_OFF:
            operation_mode = MODE_MANUAL;
            break;
        case MODE_MANUAL:
            operation_mode = MODE_A0;
            break;
        case MODE_A0:
            operation_mode = MODE_A1;
            break;
        case MODE_A1:
            operation_mode = MODE_A2;
            break;
        case MODE_A2:
            operation_mode = MODE_OFF;
            break;
        default:
            break;
        }    
    } else
        operation_mode = new_mode;
}


void btn_click(Button2& btn){
    // menu.enter();
    switch_operation_mode();
    buzzer.beep(1);
}

void btn_long_click(Button2& btn){
    // menu.exit();
    buzzer.beep(2);
}

void btn_dbl_click(Button2& btn){
    alc_valve.togle();
    buzzer.beep(3);
}


bool is_fridge_ok(){
    const uint8_t THR_WARM = 30;
    const uint8_t THR_HOT = 50;

    if(temp.fridge > THR_WARM){
    // if(temp.column >= COLUMN_HOT_TEMP && temp.fridge > THR_WARM){
        buzzer.beep(1);
        // float prcnt = (100 - map(temp.fridge, THR_WARM, THR_HOT, 1, 100)) / 100.0;
        uint16_t safeU = proportial(dstU, temp.fridge, THR_WARM, THR_HOT);
        rm.setVo(safeU);
        return false;
    }
    return true;
}

void program_manual(){
    rm.setVo(dstU);
}

// mode A0 routine - quick warm up
void program_A0(){
    static uint32_t ms;
    if(millis() - ms < RI_PROG)
        return;

    if (temp.column > COLUMN_HOT_TEMP){
        uint16_t safeU = proportial(dstU, temp.column, COLUMN_HOT_TEMP - 5, COLUMN_HOT_TEMP + 5);
        rm.setVo(safeU);
        buzzer.beep(1);
        return;
    }

    if(temp.tank < TANK_WARM_TEMP){
        static float slowV =0;
        if(slowV == 0)
            slowV = RM_MIN_V;
        if(slowV < dstU){
            slowV += 0.3;       // increase output voltage 3 volts per second
            rm.setVo(slowV);
        } else
            rm.setVo(dstU);
    } else {
        rm.setVo(dstU);
    }
    ms = millis();
}

// mode A1 routine
void program_A1(){
    static uint32_t ms;
    if(millis() - ms < RI_PROG)
        return;

    if(!is_fridge_ok())
        return;

    rm.setVo(dstU);
    ms = millis();
}

// mode A2 routine
void program_A2(){
    static uint32_t ms;
    if(millis() - ms < RI_PROG)
        return;

    if(!is_fridge_ok())
        return;

    if(alc_valve.getState() == RL_ON){
        if(temp.column - dst_t2 > 0.2)
            alc_valve.off();
    } else {
        if(temp.column <= dst_t2 && alc_valve.getState() == RL_OFF)
            alc_valve.on();
    }
    

    rm.setVo(dstU);
    ms = millis();
}

#ifdef WITH_NETWORK
void initWiFI(){
    // WiFi & OTA
    Serial.println("Init WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        Serial.println("WiFi init failed. WiFi won't use");
    } else {
        Serial.println("WiFi connected");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());

        ArduinoOTA
            .onStart([]() {
                String type;
                if (ArduinoOTA.getCommand() == U_FLASH)
                    type = "sketch";
                else // U_SPIFFS
                    type = "filesystem";

                // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
                Serial.println("Start updating " + type);
            })
            .onEnd([]() {
                Serial.println("\nEnd");
            })
            .onProgress([](unsigned int progress, unsigned int total) {
                Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
            })
            .onError([](ota_error_t error) {
                Serial.printf("Error[%u]: ", error);
                if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                else if (error == OTA_END_ERROR) Serial.println("End Failed");
            });

        ArduinoOTA.begin();
        Serial.println("OTA ready");
    }
}

void initMqtt(){

}
#endif

void processEncoder(){
    if (encoder.getCount()==0)
        return;
    dst_t2_touch = millis();
    dst_t2 += encoder.getCount() / 10.0;
    encoder.setCount(0);
    if (dst_t2 < 20.0)
        dst_t2 = 20.0;
    else if (dst_t2 > 99.9)
        dst_t2 = 99.9;
}

void loop(){
    #ifdef WITH_NETWORK
    ArduinoOTA.handle();
    #endif

    adc_read();
    enc_btn.loop();

    switch (operation_mode){
        case MODE_OFF:
            rm.setVo(0);
            break;
        case MODE_MANUAL:
            program_manual();
            break;
        case MODE_A0:
            program_A0();
            break;
        case MODE_A1:
            program_A1();
            break;
        case MODE_A2:
            program_A2();
            break;
        default:
            break;
    }
    rm.loop();
    processEncoder();
    lcd_print_operation_vals();
    buzzer.beep();
}


void setup(){
    buzzer.beep(1);
    Serial.begin(115200);
    Serial.println("Setup start...");

    operation_mode = MODE_OFF;
    display_context = CONT_MAIN;

    // init peripheral
    adc.begin();

    lcd.init();
    lcd.clear();
    lcd.backlight();
    lcd.blink_off();

    encoder.attachSingleEdge(ENC_A_PIN,ENC_B_PIN);
    ESP32Encoder::useInternalWeakPullResistors=NONE;
    encoder.setCount(0);

    enc_btn.begin(ENC_BTN_PIN);
    enc_btn.setLongClickTime(500);
    enc_btn.setClickHandler(btn_click);
    enc_btn.setLongClickHandler(btn_long_click);
    enc_btn.setDoubleClickHandler(btn_dbl_click);

    rm.begin(heater_R);

    #ifdef WITH_NETWORK
    initWiFI();

    #endif

    buzzer.beep(3);
    Serial.println("Setup finished");
}