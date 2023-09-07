/*-----------------------------------------------------------
--
-- ECE 499 - Electronically Controlling a Smart Bike Tensioning System Using Bluetooth
--                              Official version
--
-- 10 tension levels, 5 automatic modes: Comfort, Sport, Ludicrous, Adaptive, WTF
--
--  Written by:         Hien Van To      V00948322
--                      University of Victoria
--                      27/07/2022
--
-----------------------------------------------------------*/

#include <Arduino.h>
#include "ESP32Servo.h"
#include <NimBLEDevice.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <math.h>

#define DEBUG true                              // set to true for debug output, false for no debug output
#define DEBUG_SERIAL if(DEBUG) Serial
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define GENERATOR 14                            // input pin for generator voltage
#define MOTOR 33                                // PWM pin for servo motor
#define LED 32                                  // LED pin
#define threshold_voltage_adc_value 800         // change to 0 for testing without generator
// PWM pulse width range: 1500-2500us
#define PWM_MIN 1500                            // PWM pulse width to move servo motor to 0 degree position
#define PWM_MAX 2500                            // PWM pulse width to move servo motor to 180 degree position //2400 is only like 150 degrees
#define TENSION_MODE_NUM 10                     // how many settings of tension are available
static uint8_t tensionLevel = 1;                // current tension level, default at lowest tension setting
static double upshift_ratio = 10;               // default mode is manual so it will virtually upshift at super high rpm to protect the generator
static unsigned long delay_between_shift = 2000;// delay in ms between tension change
static unsigned long last_upshift = 0;          // to determine the delay between up shift
static unsigned long last_blink = 0;            // use to blink LED when low generator voltage       
static int ledState = 0;                        // use to blink LED when low generator voltage
static bool BLE_connection = false;             // use to indicate if BLE is connected
static int special_mode = 0;                    // 1: adaptive mode; 2: WTF mode
static int lastVoltage = 0;                     // use to calculate acceleration on adaptive move
static unsigned long lastTime_voltage = 0;      // use to calculate acceleration on adaptive move
static Servo myservo;


int tensionToServo(uint8_t tension_level){
    return int(7.6611*exp((11-tension_level)*0.2664));  //calibrated equation for a linear increment of tension from user experience
}

int readGeneratorVoltage(){
    unsigned long start = millis();
    int32_t sum = 0;                                    // esp32 adc ~ 20Ks/s, still within range for each adc < 4096
    int32_t count = 0;
    while (millis() - start < 50){                      // sample for 50ms
        sum += analogRead(GENERATOR);
        count += 1;
    }
    return int(sum/count);
}

class ServerCallbacks: 
    public NimBLEServerCallbacks {
        void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
            BLE_connection = true;  
            digitalWrite(LED, HIGH);                    // turn on BLE LED upon connect
            DEBUG_SERIAL.print("Client address: ");
            DEBUG_SERIAL.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
            /** We can use the connection handle here to ask for different connection parameters.
             *  Args: connection handle, min connection interval, max connection interval
             *  latency, supervision timeout.
             *  Units; Min/Max Intervals: 1.25 millisecond increments.
             *  Latency: number of intervals allowed to skip.
             *  Timeout: 10 millisecond increments, try for 5x interval time for best results.  
             */
            pServer->updateConnParams(desc->conn_handle, 6, 6, 0, 60);
        };
        void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
            BLE_connection = false;   
            digitalWrite(LED, LOW);                     // turn off BLE LED upon connect
        }
        void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
            DEBUG_SERIAL.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
        };
    };

class CharacteristicsCallbacks:
    public NimBLECharacteristicCallbacks {
        void onWrite(NimBLECharacteristic *pCharacteristic){
            uint8_t value = atoi(pCharacteristic->getValue().c_str());
            DEBUG_SERIAL.print("Received: " + String(value) + ". ");
            if (value >= 1 & value <= TENSION_MODE_NUM){                                    // check if received value is within range   
                special_mode = 0;                                                           // override special mode
                upshift_ratio = 10;                                                         // in manual mode
                if (readGeneratorVoltage() >= threshold_voltage_adc_value){                 // check if generator voltage is above threshold level
                    if (tensionLevel != value){                                             // check if received value is different from the current setting
                        tensionLevel = value;
                        pCharacteristic->setValue(std::string(("Tension is changed to level "+String(tensionLevel)).c_str()));
                        myservo.write(tensionToServo(tensionLevel));           // change the magnet array to new position
                        DEBUG_SERIAL.println("Tension is changed to level "+String(tensionLevel) + ".");
                    } else {
                        pCharacteristic->setValue(std::string(("Tension is already at level "+String(tensionLevel)).c_str()));
                        DEBUG_SERIAL.println("Tension is already at level "+String(tensionLevel) + ".");
                    }
                } else {
                    pCharacteristic->setValue("Generator voltage is too low.");
                    DEBUG_SERIAL.println("Generator voltage is too low.");
                }
            } else {
                switch (value){
                    case 11:
                        special_mode = 0;
                        upshift_ratio = 2.5;
                        delay_between_shift = 2000;
                        pCharacteristic->setValue("Enter automatic Comfort Mode");
                        DEBUG_SERIAL.println("Enter automatic Comfort Mode");
                    break;

                    case 12:
                        special_mode = 0;
                        upshift_ratio = 2;
                        delay_between_shift = 800;
                        pCharacteristic->setValue("Enter automatic Sport Mode");
                        DEBUG_SERIAL.println("Enter automatic Sport Mode");
                    break;

                    case 13:
                        special_mode = 0;
                        upshift_ratio = 1.5;
                        delay_between_shift = 200;
                        pCharacteristic->setValue("Enter automatic Ludicrous Mode");
                        DEBUG_SERIAL.println("Enter automatic Ludicrous Mode");
                    break;

                    case 14:
                        special_mode = 1;
                        delay_between_shift = 2000;
                        lastVoltage = readGeneratorVoltage();
                        lastTime_voltage = millis();
                        pCharacteristic->setValue("Enter Adaptive Mode");
                        DEBUG_SERIAL.println("Enter Adaptive Mode");
                    break;

                    case 15:
                        special_mode = 2;
                        delay_between_shift = random(2000, 10000);
                        pCharacteristic->setValue("Enter WTF Mode");
                        DEBUG_SERIAL.println("Enter WTF Mode");
                    break;

                    default:
                        pCharacteristic->setValue("Out of range tension level.");
                        DEBUG_SERIAL.println("Out of range tension level.");
                }               
            }
            pCharacteristic->notify();              // notify the smartphone return message has been sent
        }
    };

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);     // avoid crash on unstable power supply
    DEBUG_SERIAL.begin(115200);
    pinMode(GENERATOR, INPUT);
    pinMode(LED, OUTPUT);
    digitalWrite(LED, ledState);                    // turn off BLE LED
    // Initialize servo motor
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);                     // Allow allocation of all timers
    myservo.setPeriodHertz(50);                     // standard 50 hz servo
    myservo.attach(MOTOR, PWM_MIN, PWM_MAX);        // attaches the servo on pin 18 to the servo object    
    myservo.write(tensionToServo(1));               // start at lowest tension setting 1
  /*---------------------------------BLE----------------------------------------*/
    BLEDevice::init("Tensioning System");
    NimBLEServer *pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    NimBLEService *pService = pServer->createService(SERVICE_UUID);
    NimBLECharacteristic* pCharacteristic = pService->createCharacteristic(
                                            CHARACTERISTIC_UUID,
                                            NIMBLE_PROPERTY::WRITE |
                                            NIMBLE_PROPERTY::NOTIFY
                                        );
    pService->start();
    pCharacteristic->setCallbacks(new CharacteristicsCallbacks());
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);         // max power
    NimBLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);            // functions that help with iPhone connections issue
    NimBLEDevice::startAdvertising();
    DEBUG_SERIAL.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
    if (special_mode == 1){                                                                        // adaptive mode
        if (millis() - lastTime_voltage >= 200){
            unsigned long voltage_difference = readGeneratorVoltage() - lastVoltage;
            lastVoltage = voltage_difference + lastVoltage;
            lastTime_voltage = millis();
            if (voltage_difference > 0){
                // the higher acceleration, the greater voltage difference, the smaller upshift ratio, the shorter delay between shift
                upshift_ratio = 7.5*exp(-0.002*voltage_difference);           
                delay_between_shift = 20000*exp(-0.006*voltage_difference);  
            }           
        }
    } else  if (special_mode == 2){                                                                // WTF mode
        upshift_ratio = 10;                                                                        // stop upshift base on pedal speed
        if (millis()-last_upshift >= delay_between_shift){
            tensionLevel = random(1,10);                                                           // random tension level between 1-10
            delay_between_shift = random(2000, 10000);                                             // random shift delay between 2-10 seconds
            DEBUG_SERIAL.println("tension randomly changes to " + String(tensionLevel) + ".");
            myservo.write(tensionToServo(tensionLevel));                                           // change the magnet array to new position
            last_upshift = millis();
        }
    }
    if (readGeneratorVoltage() >= int(upshift_ratio*threshold_voltage_adc_value) & tensionLevel < 10 and millis()-last_upshift >= delay_between_shift){    // upshift if rpm is high enough and tension level is not at max
        tensionLevel += 1;
        myservo.write(tensionToServo(tensionLevel));                                               // change the magnet array to new position
        last_upshift = millis();
        DEBUG_SERIAL.println("Upshift to " + String(tensionLevel) + ".");
        
    }
    if (readGeneratorVoltage() < 1.1 *threshold_voltage_adc_value & tensionLevel > 1){             // downshift if rpm is low enough and tension level is not at min
        tensionLevel -= 1;
        myservo.write(tensionToServo(tensionLevel));                                               // change the magnet array to new position
        DEBUG_SERIAL.println("Downshift to " + String(tensionLevel) + ".");
    } 
    if (BLE_connection){                                                                           // if device is connected
        if (readGeneratorVoltage() < threshold_voltage_adc_value){                                 // too low generator voltage, blink LED
            if (millis() - last_blink >= 500){
                DEBUG_SERIAL.println("blink");
                last_blink = millis();
                ledState ^= 1;
                digitalWrite(LED, ledState);
            }
        }
        else {
            digitalWrite(LED, HIGH);
        }
    }
}
