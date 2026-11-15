#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_Sensor.h>

// Must match the sender's struct layout identically (108 bytes total)
typedef struct __attribute__((packed)) {
  sensors_event_t a;
  sensors_event_t g;
  sensors_event_t t;
} SensorPacket;

sensors_event_t a, g, t;

volatile SensorPacket rxBuffer;
volatile bool newDataAvailable = false;
volatile uint32_t packetCount = 0;
volatile int lastRssi = 0;

unsigned long lastStatPrint = 0;
uint32_t lastPacketCount = 0;

// ESP32 Arduino Core 3.x receive callback signature
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(SensorPacket)) {
    memcpy((void *)&rxBuffer, incomingData, sizeof(SensorPacket));
    lastRssi = recv_info->rx_ctrl->rssi;
    packetCount++;
    newDataAvailable = true;
  } else {
    Serial.printf("Warning: Unexpected packet size %d (expected %d)\n", len, sizeof(SensorPacket));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Allow ESP32-S3 native USB CDC serial to enumerate

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println("\n--- ESP32-S3 ESP-NOW Receiver Test ---");
  Serial.print("Copy this STA MAC to your ESP32-C3: ");
  Serial.println(WiFi.macAddress());
  Serial.printf("Expected SensorPacket size: %d bytes\n\n", sizeof(SensorPacket));

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while (1) delay(10);
  }

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (newDataAvailable) {
    noInterrupts();
    a = *(sensors_event_t *)&rxBuffer.a;
    g = *(sensors_event_t *)&rxBuffer.g;
    t = *(sensors_event_t *)&rxBuffer.t;
    int rssi = lastRssi;
    newDataAvailable = false;
    interrupts();

    // Print incoming telemetry ~5 times per second so the serial monitor stays readable
    static unsigned long lastDataPrint = 0;
    if (millis() - lastDataPrint >= 200) {
      lastDataPrint = millis();
      Serial.printf("RSSI: %d dBm | Accel [m/s^2]: X=%.2f Y=%.2f Z=%.2f | Gyro [rad/s]: X=%.2f Y=%.2f Z=%.2f | Temp: %.1f C\n",
                    rssi,
                    a.acceleration.x, a.acceleration.y, a.acceleration.z,
                    g.gyro.x, g.gyro.y, g.gyro.z,
                    t.temperature);
    }
  }

  // Print packet reception rate every 2 seconds
  if (millis() - lastStatPrint >= 2000) {
    uint32_t currentCount = packetCount;
    float hz = (currentCount - lastPacketCount) / 2.0f;
    lastPacketCount = currentCount;
    lastStatPrint = millis();

    if (hz == 0.0f) {
      Serial.println("Waiting for ESP-NOW packets from ESP32-C3...");
    } else {
      Serial.printf("--- Link Stats: %.1f Hz (Total packets: %lu) ---\n", hz, currentCount);
    }
  }
}