// Car Inclinometer using ESP32S3 and remote LSM6DS3 over ESP-NOW with GC9A01 Round Display
// An open source project by LazyDays Creation https://www.facebook.com/lazydayyt
//
// modified Nov 2026 T Czerwonka tczerwonka@gmail.com
// Off-Road Edition: Hardware PCNT Calibration, 5s Peak Hold, G-Force Rejection,
// Link-Loss Watchdog, Caution/Danger Angle Colors, and Pitch Percent Slope
// this receiver MAC: AC:27:6E:B0:D3:80

// uses the IoTeikXgo 1.28" ESP32-S3 round knob w/touch display
//maybe this is an Elecrow CrowPanel?
//pick the ESP32S3 dev module
//Flash Size: 16MB (128Mb)
//PSRAM: OPI PSRAM (the "R8" suffix on the ESP32-S3R8 indicates 8MB of Octal SPI PSRAM)
//USB CDC On Boot: Enabled (needed to get Serial Monitor output over the native USB port)
//USB Mode: Hardware CDC and JTAG
//Partition Scheme: 16M Flash (3MB APP/9.9MB FATFS) or Huge APP (3MB No OTA/1MB SPIFFS) (recommended if you're using LVGL for graphics)

// GC9A01
//cfg.pin_sclk = 10;
//cfg.pin_mosi = 11;
//cfg.pin_miso = -1;
//cfg.pin_dc = 3; 
//cfg.pin_cs = 9;
//cfg.pin_rst = 14;
//backlight 46

// encoder
// A pin 45
// B pin 42
// switch 41

//power light 40

//fun - WS2812 OLED
//LED_PIN 48
//LED_NUM 5


#include <Arduino.h>
#include <lvgl.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <ESP32Encoder.h>
#include <Adafruit_Sensor.h>

#include <LovyanGFX.hpp>
#include "LGFX_GC9A01.hpp"

extern "C"
{
#include "ui.h"
#include "ui_Screen1.h"
}

#include <Adafruit_NeoPixel.h>

#define LED_PIN                 48       // Onboard WS2812 RGB ring
#define LED_NUM                 5        // 5 LEDs on CrowPanel 1.28" S3
#define LED_BRIGHTNESS_NORMAL   25       // Dim green during normal trail riding (0-255)
#define LED_BRIGHTNESS_ALERT    160      // Bright amber/red for caution & danger alerts

Adafruit_NeoPixel strip(LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);

// ============================================================================
// USER TUNING & THRESHOLD CONFIGURATION (Edit here for field adjustments)
// ============================================================================

// Hardware Pins
#define PIN_BL                  46       // Display backlight
#define ENC_CLK                 45       // Rotary encoder A / CLK
#define ENC_DT                  42       // Rotary encoder B / DT
#define ENC_SW                  41       // Rotary encoder push button (Active LOW)

// Mounting Axis Direction Signs (Flip between 1.0f and -1.0f if reversed)
static const float ROLL_SIGN            = 1.0f;
static const float PITCH_SIGN           = -1.0f;
static const float ROLL_ICON_SIGN       = 1.0f;
static const float PITCH_ICON_SIGN      = -1.0f;

//Gator
// Off-Road Caution & Danger Angle Thresholds (degrees)
//static const int ROLL_CAUTION_DEG       = 15;       // Amber warning starts at 20 deg roll
//static const int ROLL_DANGER_DEG        = 22;       // Red danger starts at 30 deg roll
//static const int PITCH_CAUTION_DEG      = 20;       // Amber warning starts at 25 deg pitch
//static const int PITCH_DANGER_DEG       = 28;       // Red danger starts at 35 deg pitch

//Toyota Tundra
// Off-Road Caution & Danger Angle Thresholds (degrees)
static const int ROLL_CAUTION_DEG       = 20;       // Amber warning starts at 20 deg roll
static const int ROLL_DANGER_DEG        = 25;       // Red danger starts at 30 deg roll
static const int PITCH_CAUTION_DEG      = 25;       // Amber warning starts at 25 deg pitch
static const int PITCH_DANGER_DEG       = 30;       // Red danger starts at 35 deg pitch

// Percent Slope Label Position Offset (relative to ui_labelpitch)
static const int SLOPE_LABEL_Y_OFFSET   = 17;       // Pixels below ui_labelpitch (adjust +/- if needed)

// Warning Text Colors (24-bit RGB Hex)
#define COLOR_NORMAL_HEX        0xFFFFFF  // White
#define COLOR_SLOPE_NORMAL_HEX  0xCCCCCC  // Light gray for percent slope in normal range
#define COLOR_CAUTION_HEX       0xFFAA00  // Amber
#define COLOR_DANGER_HEX        0xFF2222  // Red
#define COLOR_PEAK_NEEDLE_HEX   0xFF8800  // Orange peak-hold telltale needle

// Complementary Filter & Dynamic G-Force Rejection
static const float ALPHA                = 0.98f;    // Baseline gyro weight (0.98 = 98% gyro, 2% accel)
static const float GRAVITY_MSS          = 9.80665f; // Standard 1g in m/s^2
static const float G_REJECT_TOLERANCE   = 2.5f;     // Reject accel if |accelMag - 1g| > 2.5 m/s^2 (~0.25g)
                                                    // Widen to 3.0f on extreme rocks; tighten to 1.5f on smooth trails

// Peak-Hold & Link Watchdog Timings
static const uint32_t PEAK_HOLD_MS      = 5000;     // Hold peak needle for 5 seconds before smooth decay
static const float PEAK_DECAY_RATE      = 0.15f;    // Return glide speed after hold expires (0.05 = slow, 0.30 = fast)
static const uint32_t LINK_TIMEOUT_MS   = 500;      // Show "--" if no ESP-NOW packet received for 500ms

// Gauge Arc Limits
static const float MAX_TILT             = 40.0f;    // Maximum pointer deflection on dial (+/- 40 deg)
static const float GAUGE_DEG_PER_DEG    = 1.35f;    // Dial graphic arc scale factor
// ============================================================================

// ---------------- ESP-NOW Payload & Link Watchdog ----------------
typedef struct __attribute__((packed)) {
    sensors_event_t a;
    sensors_event_t g;
    sensors_event_t t;
} SensorPacket;

static volatile SensorPacket rxBuffer;
static volatile bool newDataAvailable = false;
static volatile uint32_t lastRxMs = 0;

void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len)
{
    if (len == sizeof(SensorPacket))
    {
        memcpy((void *)&rxBuffer, incomingData, sizeof(SensorPacket));
        lastRxMs = millis();
        newDataAvailable = true;
    }
}

// ---------------- NVS Calibration Storage ----------------
Preferences prefs;

static float gyroBiasX = 0, gyroBiasY = 0;       // rad/s
static float rollOffset = 0, pitchOffset = 0;    // accel angle offsets (deg)
static float rollDeg = 0, pitchDeg = 0;          // raw filtered sensor angles (deg)

static void loadCalibration()
{
    prefs.begin("inclinometer", true);
    rollOffset  = prefs.getFloat("rollOff", 0.0f);
    pitchOffset = prefs.getFloat("pitchOff", 0.0f);
    gyroBiasX   = prefs.getFloat("gyroX", 0.0f);
    gyroBiasY   = prefs.getFloat("gyroY", 0.0f);
    prefs.end();
    Serial.printf("Loaded Cal -> rollOff: %.2f, pitchOff: %.2f, gyroBias: [%.4f, %.4f]\n",
                  rollOffset, pitchOffset, gyroBiasX, gyroBiasY);
}

static void saveCalibration()
{
    prefs.begin("inclinometer", false);
    prefs.putFloat("rollOff", rollOffset);
    prefs.putFloat("pitchOff", pitchOffset);
    prefs.putFloat("gyroX", gyroBiasX);
    prefs.putFloat("gyroY", gyroBiasY);
    prefs.end();
    Serial.printf("Saved Cal  -> rollOff: %.2f, pitchOff: %.2f, gyroBias: [%.4f, %.4f]\n",
                  rollOffset, pitchOffset, gyroBiasX, gyroBiasY);
}

// ---------------- Rotary Encoder & Calibration Modes ----------------
enum CalMode {
    MODE_NORMAL = 0,
    MODE_TRIM_ROLL,
    MODE_TRIM_PITCH
};

static CalMode calMode = MODE_NORMAL;
static ESP32Encoder encoder;
static uint32_t lastCalActivityMs = 0;
static uint32_t statusBannerUntilMs = 0;
static bool forceDisplayUpdate = false;

static int32_t readEncoderSteps()
{
    static int64_t lastCount = 0;
    int64_t curCount = encoder.getCount();
    int32_t steps = (int32_t)((curCount - lastCount) / 2); // Change / 2 to / 4 if 1 click = 2 degrees
    if (steps != 0)
    {
        lastCount += steps * 2;
    }
    return steps;
}

// ---------------- Display (GC9A01, config in LGFX_GC9A01.hpp) ----------------
LGFX display;

static const uint16_t screenWidth = 240;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[screenWidth * 20];
static lv_color_t buf2[screenWidth * 20];

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = area->x2 - area->x1 + 1;
    uint32_t h = area->y2 - area->y1 + 1;

    display.startWrite();
    display.setAddrWindow(area->x1, area->y1, w, h);
    display.pushImage(area->x1, area->y1, w, h, (uint16_t *)&color_p->full);
    display.endWrite();

    lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev, lv_indev_data_t *data)
{
    data->state = LV_INDEV_STATE_RELEASED;
}

// ---------------- Inclinometer UI ----------------
static int32_t lastRollAng = -1, lastPitchAng = -1;
static int32_t lastRollIcon = -1, lastPitchIcon = -1;
static int lastRollLbl = -1000, lastPitchLbl = -1000, lastSlopeLbl = -1000;
static uint32_t lastRollColorHex = 0xFFFFFFFF, lastPitchColorHex = 0xFFFFFFFF;
static CalMode lastRenderedMode = MODE_NORMAL;
static bool lastBannerActive = false;
static bool wasLinkLost = false;

// Dynamic UI elements created at startup
static lv_obj_t *ui_rollpeak = nullptr;
static lv_obj_t *ui_pitchpeak = nullptr;
static lv_obj_t *ui_labelslope = nullptr;

static float peakRollDeg = 0.0f, peakPitchDeg = 0.0f;
static uint32_t peakRollMs = 0, peakPitchMs = 0;
static int32_t lastRollPeakAng = -1, lastPitchPeakAng = -1;

static inline int32_t wrapAngle(float deg)
{
    int32_t a = (int32_t)lroundf(deg * 10.0f) % 3600;
    if (a < 0) a += 3600;
    return a;
}

static lv_obj_t *createPeakPointer(lv_obj_t *srcPointer, lv_color_t tintColor)
{
    lv_obj_update_layout(srcPointer);

    lv_obj_t *peak = lv_img_create(lv_obj_get_parent(srcPointer));
    lv_img_set_src(peak, lv_img_get_src(srcPointer));

    lv_obj_set_align(peak, lv_obj_get_style_align(srcPointer, LV_PART_MAIN));
    lv_obj_set_pos(peak, lv_obj_get_style_x(srcPointer, LV_PART_MAIN), lv_obj_get_style_y(srcPointer, LV_PART_MAIN));
    lv_obj_set_size(peak, lv_obj_get_width(srcPointer), lv_obj_get_height(srcPointer));

    lv_point_t pivot;
    lv_img_get_pivot(srcPointer, &pivot);
    lv_img_set_pivot(peak, pivot.x, pivot.y);

    lv_obj_set_style_img_recolor(peak, tintColor, LV_PART_MAIN);
    lv_obj_set_style_img_recolor_opa(peak, LV_OPA_80, LV_PART_MAIN);
    lv_obj_set_style_opa(peak, LV_OPA_70, LV_PART_MAIN);
    lv_obj_move_to_index(peak, lv_obj_get_index(srcPointer));

    return peak;
}

static lv_obj_t *createSlopeLabel(lv_obj_t *pitchLabel)
{
    lv_obj_update_layout(pitchLabel);

    lv_obj_t *lbl = lv_label_create(lv_obj_get_parent(pitchLabel));
    lv_obj_set_align(lbl, lv_obj_get_style_align(pitchLabel, LV_PART_MAIN));
    lv_obj_set_pos(lbl,
                   lv_obj_get_style_x(pitchLabel, LV_PART_MAIN),
                   lv_obj_get_style_y(pitchLabel, LV_PART_MAIN) + SLOPE_LABEL_Y_OFFSET);

    // Use LVGL's built-in font so '%' renders cleanly without needing SquareLine font regeneration
    lv_obj_set_style_text_font(lbl, LV_FONT_DEFAULT, LV_PART_MAIN);
    lv_obj_set_style_text_color(lbl, lv_color_hex(COLOR_SLOPE_NORMAL_HEX), LV_PART_MAIN);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
    lv_label_set_text(lbl, "0%");

    return lbl;
}

static void updateWarningColors(int absRoll, int absPitch)
{
    uint32_t targetRollHex = COLOR_NORMAL_HEX;
    uint32_t targetPitchHex = COLOR_NORMAL_HEX;
    uint32_t targetSlopeHex = COLOR_SLOPE_NORMAL_HEX;

    if (calMode == MODE_NORMAL && millis() >= statusBannerUntilMs)
    {
        if (absRoll >= ROLL_DANGER_DEG)        targetRollHex = COLOR_DANGER_HEX;
        else if (absRoll >= ROLL_CAUTION_DEG)  targetRollHex = COLOR_CAUTION_HEX;

        if (absPitch >= PITCH_DANGER_DEG)
        {
            targetPitchHex = COLOR_DANGER_HEX;
            targetSlopeHex = COLOR_DANGER_HEX;
        }
        else if (absPitch >= PITCH_CAUTION_DEG)
        {
            targetPitchHex = COLOR_CAUTION_HEX;
            targetSlopeHex = COLOR_CAUTION_HEX;
        }
    }

    if (targetRollHex != lastRollColorHex)
    {
        lastRollColorHex = targetRollHex;
        lv_obj_set_style_text_color(ui_labelroll, lv_color_hex(targetRollHex), LV_PART_MAIN);
    }

    if (targetPitchHex != lastPitchColorHex)
    {
        lastPitchColorHex = targetPitchHex;
        lv_obj_set_style_text_color(ui_labelpitch, lv_color_hex(targetPitchHex), LV_PART_MAIN);
        if (ui_labelslope)
        {
            lv_obj_set_style_text_color(ui_labelslope, lv_color_hex(targetSlopeHex), LV_PART_MAIN);
        }
    }
}

static void setInclination(float pitch, float roll)
{
    uint32_t now = millis();
    int rl = (int)lroundf(fabsf(roll));
    int pl = (int)lroundf(pitch);

    // Percent slope = tan(pitch) * 100, clamped to +/-100% (45 deg)
    float clampedPitch = constrain(pitch, -45.0f, 45.0f);
    int slopePct = (int)lroundf(tanf(clampedPitch * DEG_TO_RAD) * 100.0f);

    bool bannerActive = (now < statusBannerUntilMs);

    if (calMode != lastRenderedMode || bannerActive != lastBannerActive)
    {
        lastRollLbl = -1000;
        lastPitchLbl = -1000;
        lastSlopeLbl = -1000;
        lastRenderedMode = calMode;
        lastBannerActive = bannerActive;
    }

    if (bannerActive)
    {
        if (lastRollLbl != -999)
        {
            lastRollLbl = -999;
            lv_label_set_text(ui_labelroll, "--");
            lv_label_set_text(ui_labelpitch, "--");
            if (ui_labelslope) lv_label_set_text(ui_labelslope, "--");
        }
        updateWarningColors(0, 0);
    }
    else
    {
        if (rl != lastRollLbl)
        {
            lastRollLbl = rl;
            if (calMode == MODE_TRIM_PITCH)
                lv_label_set_text(ui_labelroll, "--");
            else if (calMode == MODE_TRIM_ROLL)
                lv_label_set_text_fmt(ui_labelroll, "+%d+", rl);
            else
                lv_label_set_text_fmt(ui_labelroll, "%d", rl);
        }
        if (pl != lastPitchLbl)
        {
            lastPitchLbl = pl;
            if (calMode == MODE_TRIM_ROLL)
                lv_label_set_text(ui_labelpitch, "--");
            else if (calMode == MODE_TRIM_PITCH)
            {
                if (pl == 0) lv_label_set_text(ui_labelpitch, "+0+");
                else lv_label_set_text_fmt(ui_labelpitch, "%+d+", pl);
            }
            else
            {
                if (pl == 0) lv_label_set_text(ui_labelpitch, "0");
                else lv_label_set_text_fmt(ui_labelpitch, "%+d", pl);
            }
        }
        if (ui_labelslope && slopePct != lastSlopeLbl)
        {
            lastSlopeLbl = slopePct;
            if (calMode != MODE_NORMAL)
                lv_label_set_text(ui_labelslope, "--");
            else if (slopePct == 0)
                lv_label_set_text(ui_labelslope, "0%");
            else
                lv_label_set_text_fmt(ui_labelslope, "%+d%%", slopePct);
        }
        updateWarningColors(rl, abs(pl));
    }

    // Pointers and icons capped at +-MAX_TILT
    if (pitch > MAX_TILT) pitch = MAX_TILT;
    if (pitch < -MAX_TILT) pitch = -MAX_TILT;
    if (roll > MAX_TILT) roll = MAX_TILT;
    if (roll < -MAX_TILT) roll = -MAX_TILT;

    // Update 5-second peak markers with smooth decay after hold expires
    if (calMode != MODE_NORMAL)
    {
        peakRollDeg = roll;
        peakPitchDeg = pitch;
        peakRollMs = now;
        peakPitchMs = now;
    }
    else
    {
        if (fabsf(roll) >= fabsf(peakRollDeg))
        {
            peakRollDeg = roll;
            peakRollMs = now;
        }
        else if (now - peakRollMs >= PEAK_HOLD_MS)
        {
            peakRollDeg += (roll - peakRollDeg) * PEAK_DECAY_RATE;
            if (fabsf(roll - peakRollDeg) < 0.2f)
            {
                peakRollDeg = roll;
                peakRollMs = now;
            }
        }

        if (fabsf(pitch) >= fabsf(peakPitchDeg))
        {
            peakPitchDeg = pitch;
            peakPitchMs = now;
        }
        else if (now - peakPitchMs >= PEAK_HOLD_MS)
        {
            peakPitchDeg += (pitch - peakPitchDeg) * PEAK_DECAY_RATE;
            if (fabsf(pitch - peakPitchDeg) < 0.2f)
            {
                peakPitchDeg = pitch;
                peakPitchMs = now;
            }
        }
    }

    // Peak pointers
    if (ui_rollpeak)
    {
        int32_t rollPeakAng = wrapAngle(peakRollDeg * GAUGE_DEG_PER_DEG);
        if (rollPeakAng != lastRollPeakAng)
        {
            lastRollPeakAng = rollPeakAng;
            lv_img_set_angle(ui_rollpeak, rollPeakAng);
        }
    }

    if (ui_pitchpeak)
    {
        int32_t pitchPeakAng = wrapAngle(-peakPitchDeg * GAUGE_DEG_PER_DEG);
        if (pitchPeakAng != lastPitchPeakAng)
        {
            lastPitchPeakAng = pitchPeakAng;
            lv_img_set_angle(ui_pitchpeak, pitchPeakAng);
        }
    }

    // Live pointers
    int32_t rollAng = wrapAngle(roll * GAUGE_DEG_PER_DEG);
    if (rollAng != lastRollAng)
    {
        lastRollAng = rollAng;
        lv_img_set_angle(ui_rollpointer, rollAng);
    }

    int32_t pitchAng = wrapAngle(-pitch * GAUGE_DEG_PER_DEG);
    if (pitchAng != lastPitchAng)
    {
        lastPitchAng = pitchAng;
        lv_img_set_angle(ui_pitchpointer, pitchAng);
    }

    // Vehicle icons
    int32_t rollIcon = wrapAngle(roll * ROLL_ICON_SIGN);
    if (rollIcon != lastRollIcon)
    {
        lastRollIcon = rollIcon;
        lv_img_set_angle(ui_rollicon, rollIcon);
    }
    int32_t pitchIcon = wrapAngle(pitch * PITCH_ICON_SIGN);
    if (pitchIcon != lastPitchIcon)
    {
        lastPitchIcon = pitchIcon;
        lv_img_set_angle(ui_pitchicon, pitchIcon);
    }
}

static bool sensorOk = false;
static uint32_t lastSensorUs = 0;

// Computes raw (un-offset) accelerometer tilt angles in degrees
static void accelAngles(const sensors_event_t &a, float &roll, float &pitch)
{
    float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
    roll  = atan2f(ay, az) * RAD_TO_DEG;
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
}

// Averages N incoming ESP-NOW packets while vehicle is stationary on level ground
static void calibrateSensor()
{
    updateWarningColors(0, 0);
    lv_label_set_text(ui_labelroll, "---");
    lv_label_set_text(ui_labelpitch, "---");
    if (ui_labelslope) lv_label_set_text(ui_labelslope, "---");
    lv_timer_handler();

    const int N = 100; // ~4 seconds at 25 Hz
    float sgx = 0, sgy = 0, sr = 0, sp = 0;
    int count = 0;
    uint32_t startMs = millis();

    while (count < N && (millis() - startMs < 6000))
    {
        if (newDataAvailable)
        {
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

    if (count > 0)
    {
        gyroBiasX    = sgx / count;
        gyroBiasY    = sgy / count;
        rollOffset   = sr / count;
        pitchOffset  = sp / count;
        rollDeg      = rollOffset;
        pitchDeg     = pitchOffset;
        peakRollDeg  = 0.0f;
        peakPitchDeg = 0.0f;
        saveCalibration();
        statusBannerUntilMs = millis() + 1200;
        forceDisplayUpdate = true;
    }
}

static void handleEncoder()
{
    uint32_t now = millis();

    // 1. Handle rotary turns in trim modes (1.0 deg per detent, zero filter lag)
    int32_t steps = readEncoderSteps();
    if (steps != 0)
    {
        if (calMode == MODE_TRIM_ROLL)
        {
            rollOffset -= steps * 1.0f * ROLL_SIGN;
            lastCalActivityMs = now;
            forceDisplayUpdate = true;
        }
        else if (calMode == MODE_TRIM_PITCH)
        {
            pitchOffset -= steps * 1.0f * PITCH_SIGN;
            lastCalActivityMs = now;
            forceDisplayUpdate = true;
        }
    }

    // 2. Auto-save and exit trim mode after 8 seconds of inactivity
    if (calMode != MODE_NORMAL && (now - lastCalActivityMs >= 8000))
    {
        calMode = MODE_NORMAL;
        saveCalibration();
        statusBannerUntilMs = now + 1200;
        forceDisplayUpdate = true;
    }

    // 3. Handle push-button (active LOW with INPUT_PULLUP)
    static bool lastBtnState = HIGH;
    static uint32_t btnDownMs = 0;
    static bool longPressHandled = false;
    static uint32_t lastDebounceMs = 0;

    bool rawBtn = digitalRead(ENC_SW);
    if (rawBtn != lastBtnState && (now - lastDebounceMs > 30))
    {
        lastDebounceMs = now;
        lastBtnState = rawBtn;

        if (rawBtn == LOW)
        {
            btnDownMs = now;
            longPressHandled = false;
        }
        else
        {
            if (!longPressHandled)
            {
                if (calMode == MODE_NORMAL)
                {
                    calMode = MODE_TRIM_ROLL;
                    lastCalActivityMs = now;
                }
                else if (calMode == MODE_TRIM_ROLL)
                {
                    calMode = MODE_TRIM_PITCH;
                    lastCalActivityMs = now;
                }
                else
                {
                    calMode = MODE_NORMAL;
                    saveCalibration();
                    statusBannerUntilMs = now + 1200;
                }
                forceDisplayUpdate = true;
            }
        }
    }

    // Long press (>= 1.5s) triggers full auto-zero & saves to NVS
    if (lastBtnState == LOW && !longPressHandled && (now - btnDownMs >= 1500))
    {
        longPressHandled = true;
        calMode = MODE_NORMAL;
        calibrateSensor();
    }
}

static void updateSensor()
{
    if (!newDataAvailable) return;

    noInterrupts();
    sensors_event_t a = *(sensors_event_t *)&rxBuffer.a;
    sensors_event_t g = *(sensors_event_t *)&rxBuffer.g;
    newDataAvailable = false;
    interrupts();

    uint32_t nowUs = micros();
    if (lastSensorUs == 0)
    {
        lastSensorUs = nowUs;
        accelAngles(a, rollDeg, pitchDeg);
        return;
    }

    float dt = (nowUs - lastSensorUs) * 1e-6f;
    lastSensorUs = nowUs;
    if (dt > 0.1f) dt = 0.1f;

    // Dynamic G-force rejection: trust gyro 100% when external acceleration > G_REJECT_TOLERANCE
    float accelMag = sqrtf(a.acceleration.x * a.acceleration.x +
                           a.acceleration.y * a.acceleration.y +
                           a.acceleration.z * a.acceleration.z);
    float dynamicAlpha = (fabsf(accelMag - GRAVITY_MSS) > G_REJECT_TOLERANCE) ? 1.0f : ALPHA;

    float accRoll, accPitch;
    accelAngles(a, accRoll, accPitch);

    float gx = (g.gyro.x - gyroBiasX) * RAD_TO_DEG;
    float gy = (g.gyro.y - gyroBiasY) * RAD_TO_DEG;

    rollDeg  = dynamicAlpha * (rollDeg + gx * dt) + (1.0f - dynamicAlpha) * accRoll;
    pitchDeg = dynamicAlpha * (pitchDeg + gy * dt) + (1.0f - dynamicAlpha) * accPitch;
}


static void updateStatusLeds(bool linkLost, float pitch, float roll)
{
    static uint32_t lastLedMs = 0;
    uint32_t now = millis();
    if (now - lastLedMs < 30) return; // Cap updates at ~33 FPS
    lastLedMs = now;

    strip.clear();

    // 1. Link-loss watchdog: slow blue breathing pulse
    if (linkLost)
    {
        float breath = (sinf(now * 0.004f) + 1.0f) * 0.5f; // 0.0 to 1.0
        uint8_t b = (uint8_t)(10 + breath * 90);
        strip.setBrightness(255);
        for (int i = 0; i < LED_NUM; i++) strip.setPixelColor(i, strip.Color(0, 0, b));
        strip.show();
        return;
    }

    // 2. Calibration / Trim modes
    if (calMode == MODE_TRIM_ROLL)
    {
        strip.setBrightness(60);
        for (int i = 0; i < LED_NUM; i++) strip.setPixelColor(i, strip.Color(0, 200, 255)); // Cyan
        strip.show();
        return;
    }
    if (calMode == MODE_TRIM_PITCH)
    {
        strip.setBrightness(60);
        for (int i = 0; i < LED_NUM; i++) strip.setPixelColor(i, strip.Color(255, 0, 200)); // Magenta
        strip.show();
        return;
    }

    // 3. Evaluate severity ratio (0.0 = level, 1.0 = caution, >=1.5 = danger)
    float absRoll  = fabsf(roll);
    float absPitch = fabsf(pitch);

    bool isDanger  = (absRoll >= ROLL_DANGER_DEG)  || (absPitch >= PITCH_DANGER_DEG);
    bool isCaution = (absRoll >= ROLL_CAUTION_DEG) || (absPitch >= PITCH_CAUTION_DEG);

    if (isDanger)
    {
        // Rapid red strobe (~6 Hz)
        bool flashOn = ((now / 80) % 2) == 0;
        strip.setBrightness(LED_BRIGHTNESS_ALERT);
        if (flashOn)
        {
            for (int i = 0; i < LED_NUM; i++) strip.setPixelColor(i, strip.Color(255, 0, 0));
        }
    }
    else if (isCaution)
    {
        // Alternating amber chase
        strip.setBrightness(LED_BRIGHTNESS_ALERT);
        int activeLed = (now / 120) % LED_NUM;
        for (int i = 0; i < LED_NUM; i++)
        {
            if (i == activeLed || i == (activeLed + 2) % LED_NUM)
                strip.setPixelColor(i, strip.Color(255, 130, 0)); // Amber
            else
                strip.setPixelColor(i, strip.Color(40, 15, 0));   // Dim warm background
        }
    }
    else
    {
        // Normal range: subtle dim green (set LED_BRIGHTNESS_NORMAL to 0 if you prefer dark)
        strip.setBrightness(LED_BRIGHTNESS_NORMAL);
        for (int i = 0; i < LED_NUM; i++) strip.setPixelColor(i, strip.Color(0, 255, 40));
    }

    strip.show();
}

void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.printf("Free heap = %u\n", ESP.getFreeHeap());
    Serial.println("starting");

    loadCalibration();




    // Enable 3.3V power gate to LCD, backlight, and touch controller
    pinMode(1, OUTPUT);
    digitalWrite(1, HIGH);
    delay(20); // Allow power rail to settle before SPI init

    strip.begin();
    strip.setBrightness(LED_BRIGHTNESS_NORMAL);
    strip.clear();
    strip.show();


    // Configure Rotary Encoder using ESP32-S3 hardware PCNT peripheral
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachHalfQuad(ENC_DT, ENC_CLK);
    encoder.setFilter(1023);
    encoder.clearCount();

    pinMode(ENC_SW, INPUT_PULLUP);

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

    // Create 5-second peak-hold markers & pitch percent slope sub-label
    ui_rollpeak   = createPeakPointer(ui_rollpointer,  lv_color_hex(COLOR_PEAK_NEEDLE_HEX));
    ui_pitchpeak  = createPeakPointer(ui_pitchpointer, lv_color_hex(COLOR_PEAK_NEEDLE_HEX));
    ui_labelslope = createSlopeLabel(ui_labelpitch);

    setInclination(0, 0);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    Serial.print("Receiver STA MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() == ESP_OK)
    {
        esp_now_register_recv_cb(OnDataRecv);
        sensorOk = true;
    }
    else
    {
        Serial.println("ESP-NOW init failed!");
    }
}

void loop()
{
    static uint32_t lastTick = 0;
    static uint32_t lastUpdate = 0;

    uint32_t now = millis();
    lv_tick_inc(now - lastTick);
    lastTick = now;

    handleEncoder();

    // Check ESP-NOW link status
    noInterrupts();
    uint32_t rxAgeMs = now - lastRxMs;
    bool hasEverReceived = (lastRxMs != 0);
    interrupts();

    bool linkLost = (!hasEverReceived || rxAgeMs > LINK_TIMEOUT_MS);

    if (linkLost)
    {
        if (!wasLinkLost)
        {
            wasLinkLost = true;
            lastSensorUs = 0;
            updateWarningColors(0, 0);
            lv_label_set_text(ui_labelroll, "--");
            lv_label_set_text(ui_labelpitch, "--");
            if (ui_labelslope) lv_label_set_text(ui_labelslope, "--");
            lastRollLbl = -1000;
            lastPitchLbl = -1000;
            lastSlopeLbl = -1000;
        }
    }
    else
    {
        if (wasLinkLost)
        {
            wasLinkLost = false;
            forceDisplayUpdate = true;
        }

        if (sensorOk) updateSensor();

        if (forceDisplayUpdate || (now - lastUpdate >= 20))
        {
            lastUpdate = now;
            forceDisplayUpdate = false;
            if (sensorOk)
            {
                float calibratedPitch = (pitchDeg - pitchOffset) * PITCH_SIGN;
                float calibratedRoll  = (rollDeg  - rollOffset)  * ROLL_SIGN;
                setInclination(calibratedPitch, calibratedRoll);
            }
        }
    }

    float livePitch = (pitchDeg - pitchOffset) * PITCH_SIGN;
    float liveRoll  = (rollDeg  - rollOffset)  * ROLL_SIGN;
    updateStatusLeds(linkLost, livePitch, liveRoll);



    lv_timer_handler();
}
