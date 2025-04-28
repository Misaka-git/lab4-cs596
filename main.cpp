#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// I2C settings
#define SDA_PIN 21
#define SCL_PIN 22
#define LSM6DSO_ADDR 0x6B

// LED pin
#define LED_PIN 2 // or 4 depending on your board

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;

int stepCount = 0;
float stepThreshold = 1.05;
unsigned long lastStepTime = 0;
const unsigned long stepCooldown = 400;

float readAxis(uint8_t lowReg) {
  Wire.beginTransmission(LSM6DSO_ADDR);
  Wire.write(lowReg);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSO_ADDR, 2);

  if (Wire.available() == 2) {
    uint8_t low = Wire.read();
    uint8_t high = Wire.read();
    int16_t raw = (high << 8) | low;
    return raw * 0.061f / 1000.0f;
  }
  return 0.0f;
}

bool initLSM6DSO() {
  Wire.beginTransmission(LSM6DSO_ADDR);
  Wire.write(0x10); // CTRL1_XL register
  Wire.write(0x60); // 416 Hz, ±2g
  return (Wire.endTransmission() == 0);
}

// Custom BLE Callback to receive commands
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    if (value == "ON") {
      digitalWrite(LED_PIN, HIGH);
      Serial.println("LED ON");
    } else if (value == "OFF") {
      digitalWrite(LED_PIN, LOW);
      Serial.println("LED OFF");
    }
  }
};

void setup() {
  Serial.begin(115200);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  delay(100);

  Wire.beginTransmission(LSM6DSO_ADDR);
  Wire.write(0x0F);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSO_ADDR, 1);

  if (Wire.available()) {
    uint8_t whoami = Wire.read();
    if (whoami != 0x6C) {
      while (1);
    }
  }

  if (!initLSM6DSO()) {
    while (1);
  }

  BLEDevice::init("ESP32_Step_Counter_LED");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Steps: 0");

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
}

void loop() {
  float ax = readAxis(0x28);
  float ay = readAxis(0x2A);
  float az = readAxis(0x2C);

  float rms = sqrt(ax * ax + ay * ay + az * az);

  unsigned long now = millis();

  if (rms > stepThreshold && (now - lastStepTime) > stepCooldown) {
    stepCount++;
    lastStepTime = now;

    Serial.println(stepCount); //  ONLY print step number

    char buffer[20];
    sprintf(buffer, "Steps: %d", stepCount);
    pCharacteristic->setValue(buffer);
    pCharacteristic->notify();
  }

  delay(20);
}
