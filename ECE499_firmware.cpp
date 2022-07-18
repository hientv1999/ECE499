#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Arduino.h>
#include "ESP32Servo.h"
#include <NimBLEDevice.h>



#define DEBUG true              // set to true for debug output, false for no debug output
#define DEBUG_SERIAL if(DEBUG) Serial
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define GENERATOR 35            // input pin for generator voltage
#define MOTOR 33                // PWM pin for servo motor
#define threshold_voltage 5     // change to 0 for testing without generator
// PWM pulse width range: 500-2500us
#define PWM_MIN 500             // PWM pulse width to move servo motor to 0 degree position
#define PWM_MAX 2400            // PWM pulse width to move servo motor to 180 degree position
#define TENSION_MODE_NUM 10     // how many settings of tension are available
static uint8_t tensionLevel = 1;// current tension level
static Servo myservo;

class ServerCallbacks: 
    public NimBLEServerCallbacks {
        void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
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
                if (analogRead(GENERATOR)*3.3*112/12/4095 >= threshold_voltage){      // check if generator voltage is above threshold level
                    if (tensionLevel != value){                                       // check if received value is different from the current setting
                        tensionLevel = value;
                        pCharacteristic->setValue(std::string(("Tension is changed to level "+String(tensionLevel)).c_str()));
                        myservo.write(tensionLevel*180/TENSION_MODE_NUM);             // change the magnet array to new position
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
                pCharacteristic->setValue("Out of range tension level.");
                DEBUG_SERIAL.println("Out of range tension level.");
            }
            pCharacteristic->notify();                                                  // notify the smartphone return message has been sent
        }
    };

void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    DEBUG_SERIAL.begin(115200);
    // Initialize EEPROM
    pinMode(GENERATOR, INPUT);
    // Initialize servo motor
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);    // Allow allocation of all timers
    myservo.setPeriodHertz(50);    // standard 50 hz servo
    myservo.attach(MOTOR, PWM_MIN, PWM_MAX); // attaches the servo on pin 18 to the servo object    
    myservo.write(tensionLevel*180/TENSION_MODE_NUM);
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

void loop() {
  // no code here
}