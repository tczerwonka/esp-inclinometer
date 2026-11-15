//LOLIN C3 mini
//code to send LSM6DS3 data
//receiver board is AC:27:6E:B0:D3:80
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_LSM6DS3.h>

// REPLACE WITH YOUR ESP32-S3 STA MAC ADDRESS
uint8_t receiverMacAddress[] = {0xAC, 0x27, 0x6E, 0xB0, 0xD3, 0x80};

#define I2C_SDA 5
#define I2C_SCL 6
#define LSM6DS3_ADDR 0x6B

typedef struct __attribute__((packed)) {
  sensors_event_t a;
  sensors_event_t g;
  sensors_event_t t;
} SensorPacket;

Adafruit_LSM6DS3 lsm6ds3;
SensorPacket packet;
esp_now_peer_info_t peerInfo;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Lock pins in the ESP32-C3 Wire driver so Adafruit_BusIO's internal Wire.begin() keeps pins 5 & 6
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();
  Wire.setClock(100000);

  if (!lsm6ds3.begin_I2C(LSM6DS3_ADDR, &Wire)) {
    Serial.println("Failed to find LSM6DS3 chip!");
    while (1) delay(10);
  }
  Serial.println("LSM6DS3 initialized successfully!");
  lsm6ds3.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  lsm6ds3.setGyroRange(LSM6DS_GYRO_RANGE_500_DPS);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP32-S3 peer");
    return;
  }
}

void loop() {
  lsm6ds3.getEvent(&packet.a, &packet.g, &packet.t);
  esp_now_send(receiverMacAddress, (uint8_t *)&packet, sizeof(packet));
  delay(20);
}