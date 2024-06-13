#ifndef SENSOR_ALIASES_H
#define SENSOR_ALIASES_H

//#include <Adafruit_Sensor.h>
#include <esp_camera.h>

// Create aliases for the conflicting types
typedef struct sensor_t AdafruitSensor_t;
typedef struct _sensor ESPCameraSensor_t;

#endif // SENSOR_ALIASES_H
