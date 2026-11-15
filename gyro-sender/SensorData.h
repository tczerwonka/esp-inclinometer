#pragma once
#include <Adafruit_Sensor.h>

typedef struct __attribute__((packed)) {
  sensors_event_t a;
  sensors_event_t g;
  sensors_event_t t;
} SensorPacket;