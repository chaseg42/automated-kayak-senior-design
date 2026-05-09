#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#define LED_PIN 2 // LED on GPIO 2

void handleGPIO2On();
void handleGPIO2Off();
void handleGPIO27On();
void handleGPIO27Off();
void handleRoot();

void setup();
void loop();



typedef enum {
  DISABLE = 0,
  MOVE = 1,
  ANCHOR = 2,
  FOLLOW_SHORE = 3,
  MOTOR_OVERRIDE = 4
} operatingMode;

typedef enum {
  LEFT = 0,
  RIGHT = 1,
  FORWARD = 2,
  REVERSE = 3
} direction;

// TODO degree to turn?

typedef struct {
    operatingMode mode;
    direction     directionToTurn;
    uint8_t       speed;
    uint8_t       motor1;
    uint8_t       motor2;
    uint8_t       motor3;
    uint8_t       motor4;
} UIdata;

typedef struct {
  double N;
  double E;
  double D;
} NEDVector3;

typedef struct {
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
} UTCDate;

typedef struct {
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
} UTCTime;

typedef struct {
  NEDVector3 world_position;
  NEDVector3 velocity;
  NEDVector3 rotation;
  UTCDate    utc_date;
  UTCTime    utc_time;
  uint8_t    device_id[5];
} GPSDataStruct;