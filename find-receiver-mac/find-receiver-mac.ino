#include <WiFi.h>
#include <esp_mac.h>

void setup() {
  Serial.begin(115200);
  // Wait briefly for native USB serial to connect on ESP32-S3
  delay(1500); 

  WiFi.mode(WIFI_STA);
  delay(100); // Give the Wi-Fi radio time to initialize

  Serial.print("WiFi.macAddress(): ");
  Serial.println(WiFi.macAddress());

  // Alternative direct eFuse read (always works immediately)
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  Serial.printf("Direct eFuse MAC:  %02X:%02X:%02X:%02X:%02X:%02X\n",
                baseMac[0], baseMac[1], baseMac[2],
                baseMac[3], baseMac[4], baseMac[5]);
}

void loop() {}
