#define WITH_OTA
#ifdef WITH_OTA
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#endif

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

// MCP3008 channels
#define ADC_T_COLUMN    0       // temperature in column 
#define ADC_T_TANK      1       // temperature in tank
#define ADC_T_FRIDGE    2       // temp of dstPut for stage 1, temp of out water for stage 2
#define ADC_POT         4       // potentiometr

// IO pins
#define ENC_A_PIN       34      // encoder CLK
#define ENC_B_PIN       35      // encoder CSW
#define ENC_BTN_PIN     32      // encoder button
#define ALC_VALVE_REL   33      // alcohol valve relay pin
#define BUZZER_PIN      25      // buzzer
//#define COLD_VALVE_REL 31      // cold water valve relay pin

// Refresh intervals
#define RI_LCD          100     // LCD 
#define RI_ADC          50      // MCP reading interval
#define RI_ENC          500     // encoder process interval

const char* ssid = "FRANTSOVA";
const char* password = "218506oleg";

struct {
    float tank, column, fridge;
} temp;

uint16_t dstU;                  // voltage set point
uint16_t dstP;                  // real dstPut power
float dstPercent;               
float heater_R = 16.13;         // heater resistance in Ohms 
uint16_t pot;                   // potentiometr raw value
uint32_t last_pot_ms;

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
    CONT_SETTINGS
} display_context;

LiquidCrystal_I2C lcd(0x27,16,2);
Adafruit_MCP3008 adc;
ESP32Encoder encoder;
Buzzer buzzer(BUZZER_PIN);
RMVK rm;
Relay alc_valve(ALC_VALVE_REL);
Button2 enc_btn;


void lcd_print_operation_vals(){
    // 1234567890123456
    // xx.x xx.x xx x v         // t tank, t column, t fridge, mode, 
    // xxxV xxxxW xxx.x         // input voltage, actual output power, max power percentage 
    static uint32_t ms;
    if ((millis() - ms) < RI_LCD)
        return;

    char row[20];
    sprintf(row, "%4s %4s %2s %1d %s", 
        String(temp.tank, 1).c_str(), String(temp.column, 1).c_str(), 
        String(temp.fridge, 0).c_str(), operation_mode, (alc_valve.getState()==RL_OFF) ? "x":"o"
    );
    lcd.setCursor(0,0);
    lcd.print(row);
    sprintf(row, "%3dV %4dW %s%s  ", rm.getVi(), rm.getP(), String(dstPercent, 0).c_str(),
        (dstPercent - (int)dstPercent > 0.4) ? ".":" "); // put point to indicate decimals > 0.5
    lcd.setCursor(0,1);
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
    dstPercent = map(dstP, 0, rm.getMaxP(), 0, 1000)/10.0;

    //debug
    // if(dstU % 2 == 0)
    //     alc_valve.on();
    // else
    //     alc_valve.off();

    
    static FilterExpRunningAverage filter_column, filter_tank, filter_fridge;
    temp.column = filter_column.filter(ntc_getC(adc.readADC(ADC_T_COLUMN)));
    temp.tank = filter_tank.filter(ntc_getC(adc.readADC(ADC_T_TANK)));
    temp.fridge = filter_fridge.filter(ntc_getC(adc.readADC(ADC_T_FRIDGE)));

    ms = millis();
}


#ifdef WITH_OTA
void initWiFI(){
    // WiFi & OTA
    lcd.clear();
    lcd.print("Init WiFi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
        lcd.clear();
        lcd.print("WiFi init failed");
        lcd.setCursor(0,1);
        lcd.print("Skip using WiFi");
        delay(2000);
    } else {
        lcd.clear();
        lcd.print("WiFi connected");
        lcd.setCursor(0,1);
        lcd.print("IP: ");
        lcd.print(WiFi.localIP());
        delay(2000);

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
        lcd.clear();
        lcd.print("OTA ready");
        delay(1000);
        lcd.clear();
    }
}
#endif

void enc_loop(){
    static uint32_t ms;
    if (millis() - ms < 1000)
        return;
    
    static int cnt;
    if(cnt != encoder.getCount()){
        alc_valve.togle();
        cnt = encoder.getCount();
    }
    ms = millis();
}


// human inputs processor (buttons, encoders, etc)
void process_hum_inputs(){
    // enc_btn.loop();
    // if (enc_btn.longclick_detected_reported())

    // switch (display_context){
    // case CONT_MAIN:
    //     break;
    // case CONT_SETTINGS:
    // default:
    //     break;
    // }
}


void loop(){
    #ifdef WITH_OTA
    ArduinoOTA.handle();
    #endif

    rm.loop();
    lcd_print_operation_vals();
    adc_read();
    process_hum_inputs();

    if (millis() - last_pot_ms > 2000){
        rm.setVo(dstU);
    }

    buzzer.beep();
    // btn_loop();
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

    enc_btn.begin(ENC_BTN_PIN);

    rm.begin(heater_R);
    
    // for (int n=0;n<5;n++){
    //     alc_valve.on();
    //     delay(100);
    //     alc_valve.off();
    //     delay(100);
    // }

    #ifdef WITH_OTA
    initWiFI();
    #endif

    buzzer.beep(3);
    Serial.println("Setup finished");
}