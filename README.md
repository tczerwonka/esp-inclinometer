# Wireless Off-Road Vehicle Inclinometer

A two-unit wireless pitch and roll inclinometer built with an ESP32-C3 sensor transmitter and an ESP32-S3 round display receiver communicating over ESP-NOW. Designed for off-road utility vehicles (such as the John Deere Gator XUV835), featuring hardware vibration filtering, dynamic G-force rejection, 5-second peak-hold telltales, and non-volatile rotary encoder calibration.

---

## System Architecture

The system splits sensing and display across two microcontrollers to eliminate running I2C data lines through the vehicle firewall or dash:

1. **Sensor Node (ESP32-C3 + LSM6DS3)**
   - Samples 6-axis inertial data (accelerometer and gyroscope) over I2C at 26 Hz.
   - Applies silicon-level accelerometer low-pass filtering (`CTRL8_XL` `LPF2`) to attenuate 3-cylinder/chassis vibration prior to transmission.
   - Transmits packed `SensorPacket` structs over unicast ESP-NOW at 25 Hz while the display is active, automatically dropping to a 2 Hz low-power heartbeat when the display powers down.
   - Drives an active-low onboard status LED on GPIO 8 to indicate link state and sensor health.

2. **Display Unit (ESP32-S3 + GC9A01 240x240 Round LCD)**
   - Renders dual-axis roll and pitch gauges, vehicle orientation icons, numeric degree readouts, and pitch percent grade using LVGL v8 and LovyanGFX.
   - Runs a complementary filter (`ALPHA = 0.98`) with dynamic acceleration magnitude gating to reject false tilt readings during cornering, braking, and acceleration.
   - Tracks maximum roll and pitch excursions for 5 seconds with smooth decay back to live angles.
   - Provides field calibration and trim adjustments via a hardware PCNT rotary encoder saved to NVS flash.

---

## Hardware Requirements

### Sensor Node (Transmitter)
- ESP32-C3 development board (e.g., ESP32-C3 SuperMini)
- Adafruit LSM6DS3 6-DoF IMU breakout (configured at I2C address `0x6B`)
- 12V-to-5V automotive buck converter (wide input range recommended)

### Display Unit (Receiver)
- ESP32-S3 development board
- GC9A01 240x240 SPI round TFT display
- Mechanical rotary encoder with integrated push button

---

## Pinout Configuration

### ESP32-C3 Sender
| Function | GPIO | Notes |
| :--- | :--- | :--- |
| I2C SDA | GPIO 5 | Requires external pull-ups (onboard LSM6DS3 breakout) |
| I2C SCL | GPIO 6 | Includes 9-clock bus recovery on soft reset |
| Status LED | GPIO 8 | Active-low onboard indicator |

### ESP32-S3 Receiver
| Function | GPIO | Notes |
| :--- | :--- | :--- |
| Display Backlight | GPIO 8 | Active-high backlight control |
| Encoder CLK (A) | GPIO 1 | Hardware PCNT quadrature input |
| Encoder DT (B) | GPIO 2 | Hardware PCNT quadrature input |
| Encoder Switch | GPIO 3 | Active-low with internal pull-up |
| GC9A01 SPI Pins | Defined in `LGFX_GC9A01.hpp` | Configured for 30 MHz SPI |

---

## Software Dependencies

- Arduino ESP32 Core (v3.x)
- LVGL (v8.x) with SquareLine Studio UI export (`ui.h`, `ui_Screen1.h`)
- LovyanGFX
- Adafruit LSM6DS
- Adafruit Unified Sensor
- ESP32Encoder

---

## Installation and Setup

1. **Determine Receiver MAC Address**
   - Flash `car-inclinometer.ino` to the ESP32-S3 and open the Serial Monitor at `115200` baud to note the printed STA MAC address.

2. **Configure Sender Target MAC**
   - Open `gyro-sender.ino` and update `receiverMacAddress[]` with the ESP32-S3 STA MAC address.
   - Flash `gyro-sender.ino` to the ESP32-C3.

3. **Mounting**
   - Mount the ESP32-C3 sensor enclosure rigidly to a structural chassis member (such as under the seat frame or center console) away from tilting cargo beds or spongy foam tape.
   - Ensure the sensor axes match the `ROLL_SIGN` and `PITCH_SIGN` constants at the top of `car-inclinometer.ino`.

---

## Calibration and Operation

All calibration offsets are stored in ESP32 Non-Volatile Storage (NVS) under the `inclinometer` namespace and persist across power cycles.

- **Full Level Auto-Zero:** Park the vehicle on level ground and press and hold the rotary encoder button for **1.5 seconds**. The display will show `---` while sampling 100 packets (~4 seconds) to zero roll, pitch, and gyroscope bias.
- **Manual Roll Trim:** Short-press the encoder button once (`+X+` indicator on roll). Rotate the knob to adjust roll offset in 1-degree increments.
- **Manual Pitch Trim:** Short-press the encoder button a second time (`+X+` indicator on pitch). Rotate the knob to adjust pitch offset in 1-degree increments.
- **Save and Exit:** Short-press a third time (or leave idle for 8 seconds) to save offsets to flash and return to normal operation.

---

## Status Indicators

### Display Readouts
- **White Digits:** Normal operating range.
- **Amber Digits:** Caution angle reached (default: 15 deg roll / 20 deg pitch for UTVs).
- **Red Digits:** Danger angle reached (default: 22 deg roll / 28 deg pitch for UTVs).
- **Orange Telltale Needles:** 5-second peak roll and pitch hold markers.
- **`--` Readout:** ESP-NOW link timeout (no packet received for > 500 ms).

### Sensor Node LED (GPIO 8)
- **Solid On:** Connected to display receiver and transmitting at 25 Hz.
- **Brief Flash Every 500 ms:** Standby mode; display receiver is powered off or out of range.
- **Rapid 5 Hz Blink at Boot:** LSM6DS3 failed to initialize on I2C address `0x6B`.

---

## Configuration Reference

All user-tunable constants are grouped at the top of `car-inclinometer.ino` and `gyro-sender.ino`:

- `ACCEL_LPF2_CTRL8_VAL`: Hardware accelerometer filter cutoff (`0x80` = ODR/4, `0xA0` = ODR/9, `0xC0` = ODR/50).
- `G_REJECT_TOLERANCE`: Acceleration magnitude threshold (in m/s^2) above which accelerometer blending is suspended (`2.5f` default).
- `ROLL_CAUTION_DEG` / `ROLL_DANGER_DEG`: Roll warning color thresholds.
- `PITCH_CAUTION_DEG` / `PITCH_DANGER_DEG`: Pitch warning color thresholds.
- `PEAK_HOLD_MS` / `PEAK_DECAY_RATE`: Peak needle hold duration and return glide speed.
- `SLOPE_LABEL_Y_OFFSET`: Vertical pixel offset for the pitch percent grade sub-label (`17` default).

---

## License

Based on the open-source inclinometer UI framework by LazyDays Creation. Hardware abstraction, ESP-NOW telemetry, filtering, and calibration extensions modified November 2026.
