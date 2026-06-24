#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>  
#include <Adafruit_BNO08x.h>

// PHYSICAL GPIO PINS (ESP32-C3)
constexpr uint8_t PIN_DRV_IN1     = 1;
constexpr uint8_t PIN_DRV_IN2     = 0;
constexpr uint8_t PIN_DRV_IN3     = 2;
constexpr uint8_t PIN_DRV_IN4     = 3;
constexpr uint8_t PIN_ENC_LEFT    = 5;
constexpr uint8_t PIN_ENC_RIGHT   = 6;
constexpr uint8_t PIN_I2C_SDA     = 8;   
constexpr uint8_t PIN_I2C_SCL     = 9;   
constexpr uint8_t PIN_ONBOARD_LED = 10; 

// PWM SPECS
constexpr uint32_t PWM_FREQ     = 20000; 
constexpr uint8_t  PWM_RES      = 8;     
constexpr uint8_t  STEP_SPEED   = 180; 

// KINEMATICS
constexpr float WHEEL_RADIUS_CM = 6.0;
constexpr float CM_PER_TICK     = (2.0 * PI * WHEEL_RADIUS_CM) / 16.0; 

// UART BLE UUIDs (NORDIC)
#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

// GLOBAL VARS
BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool deviceConnected = false;
bool imuReady = false;

// COUNTERS
volatile uint32_t leftTicks  = 0;
volatile uint32_t rightTicks = 0;

float currentAbsYaw = 0.0;
float yawOffset     = 0.0;

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// SIMPLE INTERRUPTS (ESP32-style)
void IRAM_ATTR leftEncoderISR() { 
  leftTicks++; 
}

void IRAM_ATTR rightEncoderISR() { 
  rightTicks++; 
}

// MOTOR DRIVING
void setMotorOutputs(int16_t speedA, int16_t speedB) {
  if (speedA >= 0) {
    ledcWrite(PIN_DRV_IN1, static_cast<uint8_t>(constrain(speedA, 0, 255)));
    ledcWrite(PIN_DRV_IN2, 0);
  } else {
    ledcWrite(PIN_DRV_IN1, 0);
    ledcWrite(PIN_DRV_IN2, static_cast<uint8_t>(constrain(abs(speedA), 0, 255)));
  }

  if (speedB >= 0) {
    ledcWrite(PIN_DRV_IN3, static_cast<uint8_t>(constrain(speedB, 0, 255)));
    ledcWrite(PIN_DRV_IN4, 0);
  } else {
    ledcWrite(PIN_DRV_IN3, 0);
    ledcWrite(PIN_DRV_IN4, static_cast<uint8_t>(constrain(abs(speedB), 0, 255)));
  }
}

void stopAll() {
  Serial.println("[MOTOR] Direct Command: ALL STOP");
  setMotorOutputs(0, 0);
}

// BLE SERVER CALLBACKS
class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* server) override { deviceConnected = true; }
    void onDisconnect(BLEServer* server) override {
        deviceConnected = false;
        stopAll();
        delay(100);
        server->getAdvertising()->start();
    }
};

// DATA PARSING CALLBACKS
class CharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        String msg = pCharacteristic->getValue();
        msg.trim();
        msg.replace("\r", ""); msg.replace("\n", ""); msg.replace("\0", "");

        if (msg.length() > 0) {
            if (msg.startsWith("SLIDER:")) {
                String data = msg.substring(7); 
                int16_t commaIndex = data.indexOf(',');
                if (commaIndex > 0) {
                    int16_t valA = data.substring(0, commaIndex).toInt();
                    int16_t valB = data.substring(commaIndex + 1).toInt();
                    setMotorOutputs(valA, valB);
                }
            } 
            else if (msg.startsWith("CMD:")) {
                String cmd = msg.substring(4);
                if (cmd == "RESET_GYRO") {
                    yawOffset = currentAbsYaw;
                    leftTicks = 0; 
                    rightTicks = 0; 
                }
                else if (cmd == "FWD") { setMotorOutputs(STEP_SPEED, STEP_SPEED); delay(400); stopAll(); }
                else if (cmd == "REV") { setMotorOutputs(-STEP_SPEED, -STEP_SPEED); delay(400); stopAll(); }
                else if (cmd == "TRN_L") { setMotorOutputs(-STEP_SPEED, STEP_SPEED); delay(280); stopAll(); }
                else if (cmd == "TRN_R") { setMotorOutputs(STEP_SPEED, -STEP_SPEED); delay(280); stopAll(); }
            }
            else if (msg.equalsIgnoreCase("STOP")) { stopAll(); }
        }
    }
};

void setup() {
  Serial.begin(115200);
  
  uint32_t timeout = millis();
  while (!Serial && (millis() - timeout < 3000)) { delay(10); }

  pinMode(PIN_ONBOARD_LED, OUTPUT);
  digitalWrite(PIN_ONBOARD_LED, LOW); 

  pinMode(PIN_ENC_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT), leftEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT), rightEncoderISR, FALLING);

  ledcAttach(PIN_DRV_IN1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_DRV_IN2, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_DRV_IN3, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_DRV_IN4, PWM_FREQ, PWM_RES);
  stopAll();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000); 

  while (!bno08x.begin_I2C(0x4B)) {
    digitalWrite(PIN_ONBOARD_LED, HIGH); delay(50);
    digitalWrite(PIN_ONBOARD_LED, LOW);  delay(200); 
  }

  bno08x.enableReport(SH2_ROTATION_VECTOR, 20000); 
  imuReady = true;
  digitalWrite(PIN_ONBOARD_LED, HIGH); 

  BLEDevice::init("C3_Motor_Testbench");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new CharacteristicCallbacks());

  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
}

void loop() {
  static uint32_t lastTxTime = 0;

  if (imuReady && bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      
      float r = sensorValue.un.rotationVector.real;
      float i = sensorValue.un.rotationVector.i;
      float j = sensorValue.un.rotationVector.j;
      float k = sensorValue.un.rotationVector.k;
      currentAbsYaw = atan2(2.0 * (r * k + i * j), 1.0 - 2.0 * (j * j + k * k)) * (180.0 / PI);
      if (currentAbsYaw < 0.0) currentAbsYaw += 360.0;

      if (deviceConnected && (millis() - lastTxTime >= 40)) { 
        lastTxTime = millis();
        
        uint32_t curLeft = leftTicks;
        uint32_t curRight = rightTicks;

        float distLeft = static_cast<float>(curLeft) * CM_PER_TICK;
        float distRight = static_cast<float>(curRight) * CM_PER_TICK;

        float softwareTaredYaw = currentAbsYaw - yawOffset;
        if (softwareTaredYaw < 0.0)  softwareTaredYaw += 360.0;
        if (softwareTaredYaw >= 360.0) softwareTaredYaw -= 360.0;

        String telemetry = "IMU:" + String(softwareTaredYaw, 1) + "," +
                           String(r, 4) + "," + String(i, 4) + "," + String(j, 4) + "," + String(k, 4) +
                           "|ENG:" + String(curLeft) + "," + String(distLeft, 1) + "," +
                                     String(curRight) + "," + String(distRight, 1);
                           
        pTxCharacteristic->setValue(telemetry.c_str());
        pTxCharacteristic->notify();
      }
    }
  }
  delay(1); 
}