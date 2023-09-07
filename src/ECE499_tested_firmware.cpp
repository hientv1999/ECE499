
#include <Arduino.h>

 #include "ESP32Servo.h"
#include <NimBLEDevice.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <math.h>

#define DEBUG true                  // set to true for debug output, false for no debug output
#define DEBUG_SERIAL if(DEBUG) Serial
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define GENERATOR 14                // input pin for generator voltage
#define MOTOR 33                    // PWM pin for servo motor
#define LED 32                      // LED pin
#define threshold_voltage_adc_value 800     // change to 0 for testing without generator
// PWM pulse width range: 1500-2500us
#define PWM_MIN 1500                // PWM pulse width to move servo motor to 0 degree position
#define PWM_MAX 2500                // PWM pulse width to move servo motor to 180 degree position //2400 is only like 150 degrees
#define TENSION_MODE_NUM 10         // how many settings of tension are available
static uint8_t tensionLevel = 1;    // current tension level, default at lowest tension setting
static double upshift_ratio = 10;   // default mode is manual so it will virtually upshift at super high rpm to protect the generator
static unsigned long last_blink = 0;// use to blink LED when low generator voltage       
static int ledState = 0;            // use to blink LED when low generator voltage
static bool BLE_connection = false; // use to indicate if BLE is connected
static Servo myservo;

int readGeneratorVoltage(){
    unsigned long start = millis();
    int32_t sum = 0;                // esp32 adc ~ 20Ks/s, still within range for each adc < 4096
    int32_t count = 0;
    while (millis() - start < 100){  // sample for 100ms
        sum += analogRead(GENERATOR);
        count += 1;
    }
    return int(sum/count);
}

class ServerCallbacks: 
    public NimBLEServerCallbacks {
        void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
            BLE_connection = true;  // turn on BLE LED upon connect
            digitalWrite(LED, HIGH);
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
            BLE_connection = false;   // turn off BLE LED upon connect
            digitalWrite(LED, LOW);
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
            if (value >= 1 & value <= TENSION_MODE_NUM){                              // check if received value is within range
                upshift_ratio = 10;  // in manual mode
                if (readGeneratorVoltage() >= threshold_voltage_adc_value){      // check if generator voltage is above threshold level
                    if (tensionLevel != value){                                       // check if received value is different from the current setting
                        tensionLevel = value;
                        pCharacteristic->setValue(std::string(("Tension is changed to level "+String(tensionLevel)).c_str()));
                        myservo.write(int(16.549*exp((11-tensionLevel)*0.1894)));             // change the magnet array to new position
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
                        upshift_ratio = 1.6;
                        pCharacteristic->setValue("Enter automatic Comfort Mode");
                        DEBUG_SERIAL.println("Enter automatic Comfort Mode");
                    break;

                    case 12:
                        upshift_ratio = 1.5;
                        pCharacteristic->setValue("Enter automatic Sport Mode");
                        DEBUG_SERIAL.println("Enter automatic Sport Mode");
                    break;

                    case 13:
                        upshift_ratio = 1.2;
                        pCharacteristic->setValue("Enter automatic Ludicrous Mode");
                        DEBUG_SERIAL.println("Enter automatic Ludicrous Mode");
                    break;

                    default:
                        pCharacteristic->setValue("Out of range tension level.");
                        DEBUG_SERIAL.println("Out of range tension level.");
                }               
            }
            pCharacteristic->notify();     // notify the smartphone return message has been sent
        }
    };

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    DEBUG_SERIAL.begin(115200);
    // Initialize EEPROM
    pinMode(GENERATOR, INPUT);
    pinMode(LED, OUTPUT);
    digitalWrite(LED, ledState); // turn off BLE LED
    // Initialize servo motor
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);    // Allow allocation of all timers
    myservo.setPeriodHertz(50);    // standard 50 hz servo
    myservo.attach(MOTOR, PWM_MIN, PWM_MAX); // attaches the servo on pin 18 to the servo object    
    myservo.write(118);            // start at lowest tension setting 1
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
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); // max power
    NimBLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    NimBLEDevice::startAdvertising();
    DEBUG_SERIAL.println("Characteristic defined! Now you can read it in your phone!");
}
unsigned long last_upshift = 0;
void loop() {
  
    if (readGeneratorVoltage() >= int(upshift_ratio*threshold_voltage_adc_value) & tensionLevel < 10 and millis()-last_upshift >= 2000){    // upshift if rpm is high enough and tension level is not at max
        DEBUG_SERIAL.println("Upshift to " + String(tensionLevel) + ".");
        tensionLevel += 1;
        myservo.write(int(16.549*exp((11-tensionLevel)*0.1894)));   // change the magnet array to new position
        last_upshift = millis();
        
    }
    // reduce tension if generator voltage is lower than threshold
    if (readGeneratorVoltage() < 1.10 *threshold_voltage_adc_value & tensionLevel > 1){               // downshift if rpm is low enough and tension level is not at min
        DEBUG_SERIAL.println("Downshift to " + String(tensionLevel) + ".");
        tensionLevel -= 1;
        myservo.write(int(16.549*exp((11-tensionLevel)*0.1894)));   // change the magnet array to new position
    } 
    if (BLE_connection){                                            // if device is connected
        if (readGeneratorVoltage() < threshold_voltage_adc_value){                                   // too low generator voltage, blink LED
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
