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

constexpr uint32_t PWM_FREQ     = 20000; 
constexpr uint8_t  PWM_RES      = 8;     
constexpr float WHEEL_RADIUS_CM = 6.0;
constexpr float CM_PER_TICK     = (2.0 * PI * WHEEL_RADIUS_CM) / 16.0; 

#define SERVICE_UUID           "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

// STATE MACHINE ENUM
enum VCUState { STATE_IDLE, STATE_NORMAL, STATE_SPORT, STATE_FAULT };
VCUState currentState = STATE_NORMAL;
uint8_t faultReason = 0; // 0=None, 1=Over-Current, 2=Stall, 3=Orientation Timeout

BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool deviceConnected = false;
bool imuReady = false;

// ODOMETRY TRACKING
volatile uint32_t leftTicks  = 0;
volatile uint32_t rightTicks = 0;
uint32_t lastLeftTicks = 0;
uint32_t lastRightTicks = 0;

float currentAbsYaw = 0.0;
float yawOffset     = 0.0;
float robotX = 0.0;
float robotY = 0.0;

// DYNAMIC CONTROL VARIABLES
uint8_t masterSpeedLimit = 195; // Stores mapped raw PWM limit (derived from 0-100% app input)
float targetLeftPWM = 0;
float targetRightPWM = 0;
float currentLeftPWM = 0;
float currentRightPWM = 0;
float simulatedCurrent = 0.15; 
float currentFaultLimit = 1.20; 

// DIAGNOSTIC WATCHDOGS (for stall faulting)
bool autoTurnActive = false;
float autoTurnTargetYaw = 0.0;
uint32_t autoTurnStartTime = 0;
uint32_t leftStallStartTime = 0;
uint32_t rightStallStartTime = 0;
uint32_t stallLastLeftTicks = 0;
uint32_t stallLastRightTicks = 0;

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

void IRAM_ATTR leftEncoderISR()  { leftTicks++; }
void IRAM_ATTR rightEncoderISR() { rightTicks++; }

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

float getNormalizedHeading() {
  float taredYaw = currentAbsYaw - yawOffset;
  if (taredYaw < 0.0) taredYaw += 360.0;
  if (taredYaw >= 360.0) taredYaw -= 360.0;
  if (taredYaw > 180.0) taredYaw -= 360.0; 
  return taredYaw;
}

float normalizeAngleRange(float angle) {
  while (angle <= -180.0) angle += 360.0;
  while (angle > 180.0)   angle -= 360.0;
  return angle;
}

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* server) override { deviceConnected = true; }
    void onDisconnect(BLEServer* server) override {
        deviceConnected = false;
        targetLeftPWM = 0; targetRightPWM = 0;
        server->getAdvertising()->start();
    }
};

class CharacteristicCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        String msg = pCharacteristic->getValue();
        msg.trim(); msg.replace("\r", ""); msg.replace("\n", ""); msg.replace("\0", "");

        if (msg.length() > 0) {
            // 0 - 100% maps linearly to 155 - 255 PWM for startup/run torque
            if (msg.startsWith("SET_SPD:")) {
                long appPercentage = msg.substring(8).toInt();
                masterSpeedLimit = map(appPercentage, 0, 100, 155, 255);
                return;
            }
            if (msg.startsWith("SET_LMT:")) {
                currentFaultLimit = msg.substring(8).toFloat();
                return;
            }
            
            if (msg == "STATE:NORMAL") { currentState = STATE_NORMAL; }
            else if (msg == "STATE:SPORT")  { currentState = STATE_SPORT; }
            else if (msg == "CMD:FAULT")    { currentState = STATE_FAULT; faultReason = 1; } 
            else if (msg == "STATE:RESET")  { 
                currentState = STATE_NORMAL; 
                faultReason = 0;
                simulatedCurrent = 0.15; 
                targetLeftPWM = 0; targetRightPWM = 0;
                currentLeftPWM = 0; currentRightPWM = 0;
                leftStallStartTime = 0; rightStallStartTime = 0;
            }
            else if (msg == "CMD:ZERO") { 
                yawOffset = currentAbsYaw; 
                robotX = 0; robotY = 0; 
                leftTicks = 0; rightTicks = 0;
                stallLastLeftTicks = 0; stallLastRightTicks = 0;
                leftStallStartTime = 0; rightStallStartTime = 0;
            }
            
            if (currentState == STATE_FAULT) return;

            if (msg == "DRV:FWD") { targetLeftPWM = masterSpeedLimit; targetRightPWM = masterSpeedLimit; autoTurnActive = false; }
            else if (msg == "DRV:REV") { targetLeftPWM = -masterSpeedLimit; targetRightPWM = -masterSpeedLimit; autoTurnActive = false; }
            else if (msg == "TRN:LEFT") { targetLeftPWM = -masterSpeedLimit; targetRightPWM = masterSpeedLimit; autoTurnActive = false; }
            else if (msg == "TRN:RIGHT") { targetLeftPWM = masterSpeedLimit; targetRightPWM = -masterSpeedLimit; autoTurnActive = false; }
            else if (msg == "DRV:STOP") { targetLeftPWM = 0; targetRightPWM = 0; }
            
            else if (msg == "CMD:TURN_90_L") {
                autoTurnTargetYaw = normalizeAngleRange(getNormalizedHeading() + 90.0);
                autoTurnActive = true;
                autoTurnStartTime = millis();
            }
            else if (msg == "CMD:TURN_90_R") {
                autoTurnTargetYaw = normalizeAngleRange(getNormalizedHeading() - 90.0);
                autoTurnActive = true;
                autoTurnStartTime = millis();
            }
        }
    }
};

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ONBOARD_LED, OUTPUT); digitalWrite(PIN_ONBOARD_LED, LOW); 

  pinMode(PIN_ENC_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT), leftEncoderISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT), rightEncoderISR, FALLING);

  ledcAttach(PIN_DRV_IN1, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_DRV_IN2, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_DRV_IN3, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_DRV_IN4, PWM_FREQ, PWM_RES);
  setMotorOutputs(0,0);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000); 

  while (!bno08x.begin_I2C(0x4B)) {
    digitalWrite(PIN_ONBOARD_LED, HIGH); delay(50);
    digitalWrite(PIN_ONBOARD_LED, LOW);  delay(200); 
  }
  bno08x.enableReport(SH2_ROTATION_VECTOR, 20000); 
  imuReady = true;

  BLEDevice::init("VCU_Testbench");
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
  
  digitalWrite(PIN_ONBOARD_LED, HIGH); 
}

void loop() {
  uint32_t now = millis();
  static uint32_t lastPhysicsTime = 0;
  static uint32_t lastTxTime = 0;

  if (imuReady && bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float r = sensorValue.un.rotationVector.real;
      float i = sensorValue.un.rotationVector.i;
      float j = sensorValue.un.rotationVector.j;
      float k = sensorValue.un.rotationVector.k;
      currentAbsYaw = atan2(2.0 * (r * k + i * j), 1.0 - 2.0 * (j * j + k * k)) * (180.0 / PI);
      if (currentAbsYaw < 0.0) currentAbsYaw += 360.0;
    }
  }

  // 20ms CONTROL LOOP (50Hz)
  if (now - lastPhysicsTime >= 20) {
    lastPhysicsTime = now;
    float currentYaw = getNormalizedHeading();

    uint32_t lTicks = leftTicks; uint32_t rTicks = rightTicks;
    float dist = (((lTicks - lastLeftTicks) + (rTicks - lastRightTicks)) / 2.0) * CM_PER_TICK;
    lastLeftTicks = lTicks; lastRightTicks = rTicks;
    
    robotX += dist * cos(currentYaw * PI / 180.0);
    robotY += dist * sin(currentYaw * PI / 180.0);

    if (currentState == STATE_FAULT) {
        targetLeftPWM = 0; targetRightPWM = 0;
        currentLeftPWM = 0; currentRightPWM = 0;
        setMotorOutputs(0, 0);
    } else {
        // Closed-Loop Automated 90° Turning Subroutines
        if (autoTurnActive) {
            float error = normalizeAngleRange(autoTurnTargetYaw - currentYaw);
            
            if (currentState == STATE_SPORT) {
                // Sport Mode Configuration: No back-damping
                if (abs(error) < 3.0) {
                    autoTurnActive = false;
                    targetLeftPWM = 0; targetRightPWM = 0;
                } else {
                    targetLeftPWM = (error > 0) ? -masterSpeedLimit : masterSpeedLimit;
                    targetRightPWM = (error > 0) ? masterSpeedLimit : -masterSpeedLimit;
                }
            } else {
                // Normal Mode Configuration: Back-Spin Correction
                if (abs(error) < 1.0) {
                    autoTurnActive = false;
                    targetLeftPWM = 0; targetRightPWM = 0;
                } else {
                    float Kp = 3.5;
                    int16_t pSpeed = abs(error) * Kp;
                    // Lower bound clamped to 170 PWM to provide enough static torque
                    int16_t dynamicOutputSpeed = constrain(170 + pSpeed, 170, masterSpeedLimit);
                    
                    // Direct proportional sign orientation routing
                    targetLeftPWM  = (error > 0) ? -dynamicOutputSpeed : dynamicOutputSpeed;
                    targetRightPWM = (error > 0) ? dynamicOutputSpeed  : -dynamicOutputSpeed;
                }
            }

            if (now - autoTurnStartTime > 3500) { 
                currentState = STATE_FAULT;
                faultReason = 3; 
                autoTurnActive = false;
            }
        }

        // Stall Identification
        bool leftStalled = false;
        bool rightStalled = false;

        if (abs(currentLeftPWM) > 160) {
            if (lTicks == stallLastLeftTicks) {
                if (leftStallStartTime == 0) leftStallStartTime = now;
                else if (now - leftStallStartTime > 1500) leftStalled = true;
            } else { leftStallStartTime = 0; stallLastLeftTicks = lTicks; }
        } else { leftStallStartTime = 0; stallLastLeftTicks = lTicks; }

        if (abs(currentRightPWM) > 160) {
            if (rTicks == stallLastRightTicks) {
                if (rightStallStartTime == 0) rightStallStartTime = now;
                else if (now - rightStallStartTime > 1500) rightStalled = true;
            } else { rightStallStartTime = 0; stallLastRightTicks = rTicks; }
        } else { rightStallStartTime = 0; stallLastRightTicks = rTicks; }

        if (leftStalled || rightStalled) {
            currentState = STATE_FAULT;
            faultReason = 2;
        }

        // State Machine Inertial Step Response Mapping
        float alpha = (currentState == STATE_SPORT) ? 0.45 : 0.06; 
        float oldLeft = currentLeftPWM;
        float oldRight = currentRightPWM;
        
        currentLeftPWM += alpha * (targetLeftPWM - currentLeftPWM);
        currentRightPWM += alpha * (targetRightPWM - currentRightPWM);

        float deltaPWM = abs(currentLeftPWM - oldLeft) + abs(currentRightPWM - oldRight);
        simulatedCurrent = 0.15 + (deltaPWM * 0.0035) + ((abs(currentLeftPWM) + abs(currentRightPWM)) / 510.0) * 0.22;

        if (currentFaultLimit >= 0.0 && simulatedCurrent > currentFaultLimit) {
            currentState = STATE_FAULT;
            faultReason = 1;
        } else {
            setMotorOutputs(currentLeftPWM, currentRightPWM);
        }
    }
  }

  // TELEMETRY BROADCASTS
  if (deviceConnected && (now - lastTxTime >= 50)) { 
    lastTxTime = now;
    // 155-255 mapped to 0-100%
    int16_t feedbackPercentage = map(masterSpeedLimit, 155, 255, 0, 100);
    
    String telemetry = String(getNormalizedHeading(), 1) + "|" +
                       String(robotX, 1) + "|" + 
                       String(robotY, 1) + "|" + 
                       String(simulatedCurrent, 2) + "|" + 
                       String(currentState) + "|" +
                       String(currentFaultLimit, 2) + "|" +
                       String(faultReason);
    pTxCharacteristic->setValue(telemetry.c_str());
    pTxCharacteristic->notify();
  }
}