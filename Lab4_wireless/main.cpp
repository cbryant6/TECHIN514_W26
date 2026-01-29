#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <stdlib.h>
#include <math.h>   // for isnan()

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// ===== Timing =====
unsigned long previousMillis = 0;
const long interval = 100;   // how often we sample + possibly notify (ms)

// ===== TODO (filled): global variables for sensor readings and processed data =====
// HC-SR04 pins (change if needed)
static const uint8_t TRIG_PIN = 7;
static const uint8_t ECHO_PIN = 6;

// Raw + processed distance values (cm)
float rawCm = NAN;
float denoisedCm = NAN;

// ===== TODO (filled): DSP state (moving average) =====
struct MovingAverage {
  static const int N = 5;   // moving average window size
  float buf[N] = {0};
  int idx = 0;
  int count = 0;

  float update(float x) {
    buf[idx] = x;
    idx = (idx + 1) % N;
    if (count < N) count++;

    float sum = 0.0f;
    for (int i = 0; i < count; i++) sum += buf[i];
    return sum / (float)count;
  }
};

MovingAverage ma;

// ===== TODO (filled): UUIDs already set (yours) =====
#define SERVICE_UUID        "3a61d668-3a41-4ad9-bd56-c4940609ac0e"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
  }
};

// ===== TODO (filled): DSP / sensor helper functions =====

// Read HC-SR04 distance in cm
float readDistanceCm(uint8_t trigPin, uint8_t echoPin) {
  // Trigger pulse: LOW -> HIGH(10us) -> LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure echo pulse width (timeout 30ms)
  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);

  if (duration == 0) return NAN;  // timeout / no echo

  // Convert microseconds to cm
  return (duration * 0.0343f) / 2.0f;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("Starting BLE work!");

  // ===== TODO (filled): sensor setup (pinMode, etc.) =====
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // ===== TODO (filled): name your device to avoid conflicts =====
  BLEDevice::init("Coleman ESP32 HC-SR04");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("Booting...");
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.println("Advertising started. Waiting for client...");
}

void loop() {
  unsigned long currentMillis = millis();

  // ===== TODO (filled): handle sensor readings =====
  // Sample every `interval` ms (also used as notify schedule)
  if (currentMillis - previousMillis >= (unsigned long)interval) {
    previousMillis = currentMillis;

    rawCm = readDistanceCm(TRIG_PIN, ECHO_PIN);

    // Basic validity gate (HC-SR04 typical range ~2–400 cm)
    if (isnan(rawCm) || rawCm < 2.0f || rawCm > 400.0f) {
      Serial.println("Raw(cm): invalid | Denoised(cm): invalid");
      return;
    }

    // ===== TODO (filled): DSP denoise =====
    denoisedCm = ma.update(rawCm);

    // Always print raw + denoised for the screenshot requirement
    Serial.print("Raw(cm): ");
    Serial.print(rawCm, 1);
    Serial.print(" | Denoised(cm): ");
    Serial.print(denoisedCm, 1);

    // ===== BLE transmit ONLY if denoised < 30cm =====
    bool shouldSend = (deviceConnected && denoisedCm < 30.0f);

    if (shouldSend) {
      // Send denoised value as ASCII text, e.g. "24.7"
      char payload[16];
      snprintf(payload, sizeof(payload), "%.1f", denoisedCm);
      pCharacteristic->setValue((uint8_t*)payload, strlen(payload));
      pCharacteristic->notify();
      Serial.print("  --> SENT");
    }

    Serial.println();
  }

  // ===== Connection state handling (keep your original logic) =====
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                 // give BT stack time
    pServer->startAdvertising();
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }

  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // Small delay so loop isn't crazy fast (sampling is already gated by interval)
  delay(10);
}
