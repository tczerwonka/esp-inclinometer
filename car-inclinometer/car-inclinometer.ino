// Car Inclinometer using ESP32S3 and remote LSM6DS3 over ESP-NOW with GC9A01 Round Display
// An open source project by LazyDays Creation https://www.facebook.com/lazydayyt
//
// modified Nov 2026 T Czerwonka tczerwonka@gmail.com
// using remote LSM6DS3 via ESP-NOW
// this receiver MAC: AC:27:6E:B0:D3:80

#include <Arduino.h>
#include <lvgl.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_Sensor.h>

#include <LovyanGFX.hpp>
#include "LGFX_GC9A01.hpp"

extern "C" {
#include "ui.h"
#include "ui_Screen1.h"
}

#define PIN_BL 8  // Display backlight

// ---------------- ESP-NOW Payload ----------------
typedef struct __attribute__((packed)) {
  sensors_event_t a;
  sensors_event_t g;
  sensors_event_t t;
} SensorPacket;

static volatile SensorPacket rxBuffer;
static volatile bool newDataAvailable = false;

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(SensorPacket)) {
    memcpy((void *)&rxBuffer, incomingData, sizeof(SensorPacket));
    newDataAvailable = true;
  }
}

// ---------------- Display (GC9A01, config in LGFX_GC9A01.hpp) ----------------
LGFX display;

static const uint16_t screenWidth = 240;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[screenWidth * 20];
static lv_color_t buf2[screenWidth * 20];

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;

  display.startWrite();
  display.setAddrWindow(area->x1, area->y1, w, h);
  display.pushImage(area->x1, area->y1, w, h, (uint16_t *)&color_p->full);
  display.endWrite();

  lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev, lv_indev_data_t *data) {
  data->state = LV_INDEV_STATE_RELEASED;
}

// ---------------- Inclinometer UI ----------------
// Gauge: 0 at top, clockwise. Roll zero at 270 deg (left), pitch zero at 90 deg (right).
// +-40 deg of tilt spans 54 deg of gauge arc -> 1.35 gauge deg per tilt deg.
static const float MAX_TILT = 40.0f;
static const float GAUGE_DEG_PER_DEG = 1.35f;

// Car icon rotation: flip the sign if an icon tilts the wrong way
static const float ROLL_ICON_SIGN = 1.0f;
static const float PITCH_ICON_SIGN = -1.0f;

static int32_t lastRollAng = -1, lastPitchAng = -1;
static int32_t lastRollIcon = -1, lastPitchIcon = -1;
static int lastRollLbl = -1000, lastPitchLbl = -1000;

static inline int32_t wrapAngle(float deg)  // deg -> LVGL 0.1 deg units, 0..3599
{
  int32_t a = (int32_t)lroundf(deg * 10.0f) % 3600;
  if (a < 0) a += 3600;
  return a;
}

// pitch: deg, + = nose up. roll: deg, + = right side down (clockwise)
static void setInclination(float pitch, float roll) {
  // Labels (true values): pitch with sign, roll without
  int rl = (int)lroundf(fabsf(roll));
  int pl = (int)lroundf(pitch);
  if (rl != lastRollLbl) {
    lastRollLbl = rl;
    lv_label_set_text_fmt(ui_labelroll, "%d", rl);
  }
  if (pl != lastPitchLbl) {
    lastPitchLbl = pl;
    if (pl == 0) lv_label_set_text(ui_labelpitch, "0");
    else lv_label_set_text_fmt(ui_labelpitch, "%+d", pl);
  }

  // Pointers and icons capped at +-40
  if (pitch > MAX_TILT) pitch = MAX_TILT;
  if (pitch < -MAX_TILT) pitch = -MAX_TILT;
  if (roll > MAX_TILT) roll = MAX_TILT;
  if (roll < -MAX_TILT) roll = -MAX_TILT;

  // Roll pointer: clockwise roll -> clockwise rotation
  int32_t rollAng = wrapAngle(roll * GAUGE_DEG_PER_DEG);
  if (rollAng != lastRollAng) {
    lastRollAng = rollAng;
    lv_img_set_angle(ui_rollpointer, rollAng);
  }

  // Pitch pointer: +pitch moves toward the top (counter-clockwise on the right side)
  int32_t pitchAng = wrapAngle(-pitch * GAUGE_DEG_PER_DEG);
  if (pitchAng != lastPitchAng) {
    lastPitchAng = pitchAng;
    lv_img_set_angle(ui_pitchpointer, pitchAng);
  }

  // Car icons
  int32_t rollIcon = wrapAngle(roll * ROLL_ICON_SIGN);
  if (rollIcon != lastRollIcon) {
    lastRollIcon = rollIcon;
    lv_img_set_angle(ui_rollicon, rollIcon);
  }
  int32_t pitchIcon = wrapAngle(pitch * PITCH_ICON_SIGN);
  if (pitchIcon != lastPitchIcon) {
    lastPitchIcon = pitchIcon;
    lv_img_set_angle(ui_pitchicon, pitchIcon);
  }
}

// Flip these if the display moves the wrong way for your mounting
static const float ROLL_SIGN = 1.0f;
static const float PITCH_SIGN = -1.0f;

static const float ALPHA = 0.98f;  // complementary filter: gyro weight (higher = smoother, slower drift correction)

static bool sensorOk = false;
static float gyroBiasX = 0, gyroBiasY = 0;     // rad/s
static float rollOffset = 0, pitchOffset = 0;  // accel angle offsets from startup calibration
static float rollDeg = 0, pitchDeg = 0;

static void accelAngles(const sensors_event_t &a, float &roll, float &pitch) {
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  roll = atan2f(ay, az) * RAD_TO_DEG - rollOffset;
  pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG - pitchOffset;
}

// Waits for N incoming ESP-NOW packets while board is still to average bias/offsets
static void calibrateSensor() {
  const int N = 200;
  float sgx = 0, sgy = 0, sr = 0, sp = 0;
  int count = 0;
  uint32_t startMs = millis();

  while (count < N && (millis() - startMs < 6000))  // 6s timeout if sender isn't powered yet
  {
    if (newDataAvailable) {
      noInterrupts();
      sensors_event_t a = *(sensors_event_t *)&rxBuffer.a;
      sensors_event_t g = *(sensors_event_t *)&rxBuffer.g;
      newDataAvailable = false;
      interrupts();

      float r, p;
      accelAngles(a, r, p);
      sgx += g.gyro.x;
      sgy += g.gyro.y;
      sr += r;
      sp += p;
      count++;
    }
    delay(1);
  }

  if (count > 0) {
    gyroBiasX = sgx / count;
    gyroBiasY = sgy / count;
    rollOffset = sr / count;  // whatever the board reads at boot becomes "zero"
    pitchOffset = sp / count;
  }
  rollDeg = 0;
  pitchDeg = 0;
}

// Event-driven on incoming ESP-NOW packets
static void updateSensor() {
  if (!newDataAvailable) return;

  noInterrupts();
  sensors_event_t a = *(sensors_event_t *)&rxBuffer.a;
  sensors_event_t g = *(sensors_event_t *)&rxBuffer.g;
  newDataAvailable = false;
  interrupts();

  static uint32_t lastUs = 0;
  uint32_t nowUs = micros();
  if (lastUs == 0) {
    lastUs = nowUs;
    return;
  }

  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt > 0.1f) dt = 0.1f;

  float accRoll, accPitch;
  accelAngles(a, accRoll, accPitch);

  float gx = (g.gyro.x - gyroBiasX) * RAD_TO_DEG;  // roll rate
  float gy = (g.gyro.y - gyroBiasY) * RAD_TO_DEG;  // pitch rate

  rollDeg = ALPHA * (rollDeg + gx * dt) + (1.0f - ALPHA) * accRoll;
  pitchDeg = ALPHA * (pitchDeg + gy * dt) + (1.0f - ALPHA) * accPitch;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("Free heap = %u\n", ESP.getFreeHeap());
  Serial.println("starting");

  pinMode(PIN_BL, OUTPUT);
  digitalWrite(PIN_BL, HIGH);

  display.init();
  display.setRotation(0);
  display.setSwapBytes(true);

  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, screenWidth * 20);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  ui_init();
  setInclination(0, 0);

  // Initialize Wi-Fi in STA mode and start ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_8_5dBm); // Reduces 3.3V rail current spikes during 802.11 ACKs
  WiFi.disconnect();
  Serial.print("Receiver STA MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(OnDataRecv);
    // calibrateSensor(); // Uncomment if you want boot-time zeroing from the remote C3
    sensorOk = true;
  } else {
    Serial.println("ESP-NOW init failed!");
  }
}

void loop() {
  static uint32_t lastTick = 0;
  static uint32_t lastUpdate = 0;

  uint32_t now = millis();
  lv_tick_inc(now - lastTick);
  lastTick = now;

  if (sensorOk) updateSensor();

  if (now - lastUpdate >= 20) {
    lastUpdate = now;
    if (sensorOk) setInclination(pitchDeg * PITCH_SIGN, rollDeg * ROLL_SIGN);
  }

  static uint32_t lastPrint = 0;  // Serial debug
  if (sensorOk && now - lastPrint >= 100) {
    lastPrint = now;
    // Serial.printf("pitch: %.1f  roll: %.1f\n", pitchDeg, rollDeg);
  }

  lv_timer_handler();
}