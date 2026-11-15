// Off-Road Inclinometer ESP-NOW Sender (ESP32-C3 + LSM6DS3)
// Transmits 25 Hz IMU packets to ESP32-S3 display receiver

#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_LSM6DS3.h>

// ============================================================================
// USER TUNING & HARDWARE CONFIGURATION (Edit here for field adjustments)
// ============================================================================
uint8_t receiverMacAddress[] = {0xAC, 0x27, 0x6E, 0xB0, 0xD3, 0x80};

#define I2C_SDA              5
#define I2C_SCL              6
#define LSM6DS3_ADDR         0x6B
#define LED_PIN              8      // Onboard blue LED on ESP32-C3 SuperMini (Active LOW)

// LSM6DS3 Hardware Accelerometer Low-Pass Filter (CTRL8_XL register 0x17)
// Strips high-frequency engine/washboard chassis vibration before transmission.
// Options at 26 Hz ODR:
//   0x80 = ODR/4   (~6.5 Hz cutoff - fastest response)
//   0xA0 = ODR/9   (~2.9 Hz cutoff - recommended off-road balance)
//   0xC0 = ODR/50  (~0.5 Hz cutoff - heavy damping for extreme rock/washboard chatter)
#define ACCEL_LPF2_CTRL8_VAL 0xA0

#define TX_RATE_CONNECTED_MS 40     // 25 Hz when S3 display is online
#define TX_RATE_STANDBY_MS   500    // 2 Hz low-power heartbeat when S3 display is off
// ============================================================================

typedef struct __attribute__((packed)) {
  sensors_event_t a;
  sensors_event_t g;
  sensors_event_t t;
} SensorPacket;

Adafruit_LSM6DS3 lsm6ds3;
SensorPacket packet;
esp_now_peer_info_t peerInfo;

static volatile bool receiverReachable = false;
static volatile uint8_t consecutiveFails = 0;

// Clears stuck I2C SDA line if ESP32-C3 resets mid-transaction while sensor stays powered
static void recoverI2C(uint8_t sdaPin, uint8_t sclPin) {
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, OUTPUT);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(10);

  for (int i = 0; i < 9; i++) {
    if (digitalRead(sdaPin) == HIGH) break;
    digitalWrite(sclPin, LOW);
    delayMicroseconds(10);
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(10);
  }

  pinMode(sdaPin, OUTPUT);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(10);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sdaPin, HIGH);
  delayMicroseconds(10);
}

// ESP32 Arduino Core 3.x unicast ACK status callback
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    receiverReachable = true;
    consecutiveFails = 0;
  } else {
    if (consecutiveFails < 255) consecutiveFails++;
    if (consecutiveFails >= 10) {
      receiverReachable = false;
    }
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); // Start OFF (Active LOW)

  setCpuFrequencyMhz(80);

  recoverI2C(I2C_SDA, I2C_SCL);
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);

  if (!lsm6ds3.begin_I2C(LSM6DS3_ADDR, &Wire)) {
    // Fast 5 Hz blink if LSM6DS3 fails to initialize
    while (1) {
      digitalWrite(LED_PIN, LOW);
      delay(100);
      digitalWrite(LED_PIN, HIGH);
      delay(100);
    }
  }

  lsm6ds3.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds3.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);
  lsm6ds3.setAccelDataRate(LSM6DS_RATE_26_HZ);
  lsm6ds3.setGyroDataRate(LSM6DS_RATE_26_HZ);

  // Enable hardware LPF2 on LSM6DS3 accelerometer to attenuate chassis vibration
  Wire.beginTransmission(LSM6DS3_ADDR);
  Wire.write(0x17); // CTRL8_XL register
  Wire.write(ACCEL_LPF2_CTRL8_VAL);
  Wire.endTransmission();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setTxPower(WIFI_POWER_5dBm);
  WiFi.setSleep(true);

  if (esp_now_init() != ESP_OK) {
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

void loop() {
  lsm6ds3.getEvent(&packet.a, &packet.g, &packet.t);
  esp_now_send(receiverMacAddress, (uint8_t *)&packet, sizeof(packet));

  if (receiverReachable) {
    digitalWrite(LED_PIN, LOW); // Solid blue LED when connected
    delay(TX_RATE_CONNECTED_MS);
  } else {
    // Brief 15ms flash every 500ms while waiting for S3 display
    digitalWrite(LED_PIN, LOW);
    delay(15);
    digitalWrite(LED_PIN, HIGH);
    delay(TX_RATE_STANDBY_MS - 15);
  }
}