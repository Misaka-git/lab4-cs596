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

// BLE UUIDs
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;

// Step counting variables
int stepCount = 0;
float stepThreshold = 1.05;  // adjust based on your shaking strength
unsigned long lastStepTime = 0;
const unsigned long stepCooldown = 400; // ms

// Function to read one axis acceleration (example: X=0x28, Y=0x2A, Z=0x2C)
float readAxis(uint8_t lowReg) {
  Wire.beginTransmission(LSM6DSO_ADDR);
  Wire.write(lowReg);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSO_ADDR, 2);

  if (Wire.available() == 2) {
    uint8_t low = Wire.read();
    uint8_t high = Wire.read();
    int16_t raw = (high << 8) | low;
    return raw * 0.061f / 1000.0f; // convert mg -> g
  }
  return 0.0f;
}

// Initialize LSM6DSO sensor manually
bool initLSM6DSO() {
  Wire.beginTransmission(LSM6DSO_ADDR);
  Wire.write(0x10); // CTRL1_XL register (accel settings)
  Wire.write(0x60); // 416 Hz, ±2g
  return (Wire.endTransmission() == 0);
}

void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C

  delay(100);

  // Check WHO_AM_I
  Wire.beginTransmission(LSM6DSO_ADDR);
  Wire.write(0x0F);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSO_ADDR, 1);

  if (Wire.available()) {
    uint8_t whoami = Wire.read();
    Serial.print("WHO_AM_I: 0x");
    Serial.println(whoami, HEX);
    if (whoami != 0x6C) {
      Serial.println("Wrong device! Exiting.");
      while (1);
    }
  }

  if (!initLSM6DSO()) {
    Serial.println("Failed to initialize LSM6DSO sensor!");
    while (1);
  }

  Serial.println("✅ LSM6DSO initialized manually!");

  // BLE setup
  BLEDevice::init("ESP32_Step_Counter");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Steps: 0");

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  Serial.println("✅ BLE advertising started!");
}

void loop() {
  float ax = readAxis(0x28); // X-axis
  float ay = readAxis(0x2A); // Y-axis

  float rms = sqrt(ax * ax + ay * ay); // Only X and Y

  unsigned long now = millis();

  if (rms > stepThreshold && (now - lastStepTime) > stepCooldown) {
    stepCount++;
    lastStepTime = now;

    Serial.println(stepCount); // Only print step number

    char buffer[20];
    sprintf(buffer, "Steps: %d", stepCount);
    pCharacteristic->setValue(buffer);
    pCharacteristic->notify();
  }

  delay(20);
}
