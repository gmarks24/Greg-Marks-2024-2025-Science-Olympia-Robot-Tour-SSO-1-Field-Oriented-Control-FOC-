//:: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: ::
//:: Greg Marks 2024-2025 Science Oly Robot tour SSO-1 - Field Oriented Control(FOC)     ::
//:: Made for Mecanum Drive                                                              ::
//:: MCU - Arduino R4 Minima                                                             ::
//:: MC - DFRobot DRI0039                                                                ::
//:: Operation - 9V & Max 20.6Amps [Rec. 20-22awg wire]                                  ::
//:: Rec. battery - 6(six) AA 1.5V                                                       ::
//:: IMU - Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055              ::
//:: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: :: ::
//V1.2.0
//11 Oct 2024
//#include "Arduino_LED_Matrix.h" //<Only needed if using Arduino R4 WiFi>//
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

const int P1 = 3;                                                            // Motor1 power (Front Right)
const int P2 = 11;                                                           // Motor2 power (Front Left)
const int P3 = 5;                                                            // Motor3 power (Back Right)
const int P4 = 6;                                                            // Motor4 power (Back Left)
const int M1 = 4;                                                            // Motor1 Direction (Front Right)
const int M2 = 12;                                                           // Motor2 Direction (Front Left)
const int M3 = 8;                                                            // Motor3 Direction (Back Right)
const int M4 = 7;                                                            // Motor4 Direction (Back Left)
const int FOC_POWER_SET_1 = 255;                                             // Motor power set high
const int FOC_POWER_SET_2 = 0;                                               // Motor power set low
int foc_distance_25cm = 500;                                                 // Distance for traveling 25cm in FOC. Movement distances in milliseconds.

int foc_distance_50cm_left = 1000;                                                // Distance for traveling 50cm in FOC. Movement distances in milliseconds.
int foc_distance_100cm_left = 2000;                                               // Distance for traveling 100cm in FOC. Movement distances in milliseconds.
int foc_distance_150cm_left = 3000;                                               // Distance for traveling 150cm in FOC. Movement distances in milliseconds.
int foc_distance_200cm_left = 4000;                                               // Distance for traveling 200cm in FOC. Movement distances in milliseconds.

int foc_distance_50cm_right = 1000;                                                // Distance for traveling 50cm in FOC. Movement distances in milliseconds.
int foc_distance_100cm_right = 2000;                                               // Distance for traveling 100cm in FOC. Movement distances in milliseconds.
int foc_distance_150cm_right = 3000;                                               // Distance for traveling 150cm in FOC. Movement distances in milliseconds.
int foc_distance_200cm_right = 4000;                                               // Distance for traveling 200cm in FOC. Movement distances in milliseconds.

int foc_distance_50cm_forward = 1000;                                                // Distance for traveling 50cm in FOC. Movement distances in milliseconds.
int foc_distance_100cm_forward = 2000;                                               // Distance for traveling 100cm in FOC. Movement distances in milliseconds.
int foc_distance_150cm_forward = 3000;                                               // Distance for traveling 150cm in FOC. Movement distances in milliseconds.
int foc_distance_200cm_forward = 4000;                                               // Distance for traveling 200cm in FOC. Movement distances in milliseconds.

int foc_distance_50cm_backward = 1000;                                                // Distance for traveling 50cm in FOC. Movement distances in milliseconds.
int foc_distance_100cm_backward = 2000;                                               // Distance for traveling 100cm in FOC. Movement distances in milliseconds.
int foc_distance_150cm_backward = 3000;                                               // Distance for traveling 150cm in FOC. Movement distances in milliseconds.
int foc_distance_200cm_backward = 4000;                                               // Distance for traveling 200cm in FOC. Movement distances in milliseconds.

int movement_list[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };  // Up to 9600 command chains. Always end with a '0' command to end the program.  Use to set robots movement=> <https://docs.google.com/document/d/13FU5d5H6w4hezWctLNM-Rf5GTDMTaTW0ypAhlw5teiY/edit>
const int maxStages = sizeof(movement_list);                                 // Determines the size of 'movement_list'
const int cool_down_time_set = 1;                                         //milliseconds
int motor_tweak[] = { 0.99, 0.99, 0.99, 0.99};
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);


void foc_off(int FOC_POWER_SET_2) {
  for (int pin : { P1, P2, P3, P4 }) { analogWrite(pin, FOC_POWER_SET_2); }
}
void foc_forward(int FOC_POWER_SET_1) {
  int pins[] = { M1, M2, M3, M4 };
  bool states[] = { HIGH, LOW, HIGH, LOW };
  for (int i = 0; i < 4; i++) { digitalWrite(pins[i], states[i]); }
  for (int pin : { P1, P2, P3, P4 }) { analogWrite(pin, FOC_POWER_SET_1*motor_tweak[pin]); }
}
void foc_backward(int FOC_POWER_SET_1) {
  int pins[] = { M1, M2, M3, M4 };
  bool states[] = { LOW, HIGH, LOW, HIGH };
  for (int i = 0; i < 4; i++) { digitalWrite(pins[i], states[i]); }
  for (int pin : { P1, P2, P3, P4 }) { analogWrite(pin, FOC_POWER_SET_1*motor_tweak[pin]); }
}
void foc_left(int FOC_POWER_SET_1) {
  int pins[] = { M1, M2, M3, M4 };
  bool states[] = { LOW, LOW, HIGH, HIGH };
  for (int i = 0; i < 4; i++) { digitalWrite(pins[i], states[i]); }
  for (int pin : { P1, P2, P3, P4 }) { analogWrite(pin, FOC_POWER_SET_1*motor_tweak[pin]); }
}
void foc_right(int FOC_POWER_SET_1) {
  int pins[] = { M1, M2, M3, M4 };
  bool states[] = { HIGH, HIGH, LOW, LOW };
  for (int i = 0; i < 4; i++) { digitalWrite(pins[i], states[i]); }
  for (int pin : { P1, P2, P3, P4 }) { analogWrite(pin, FOC_POWER_SET_1*motor_tweak[pin]); }
}



void setup() {
  Serial.begin(115200);
  int pins[] = { P1, P2, P3, P4, M1, M2, M3, M4 };
  for (int i = 0; i < 8; i++) { pinMode(pins[i], OUTPUT);}
  while (!Serial) delay(10);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
}
void pre_executionMovement() {
  foc_forward(FOC_POWER_SET_1);
  delay(foc_distance_25cm);
  foc_off(FOC_POWER_SET_2);
}
void loop() {
  if (Serial.available() > 0) {
    
    delay(10000);
    pre_executionMovement();
    delay(1000);
    for (int stage = 0; stage <= maxStages; stage++) {
      if (movement_list[stage] == 0) { break; }
      Serial.print("Executing stage: ");
      Serial.println(stage);
      executeMovement(movement_list[stage]);
    }
  }
}
int error = 0.09;
void executeMovement(int command) {
  switch (command) {
    case 1:
      foc_left(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_50cm_left; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 2:
      foc_left(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_100cm_left; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 3:
      foc_left(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_150cm_left; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 4:
      foc_left(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_200cm_left; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 5:
      foc_right(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_50cm_right; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 6:
      foc_right(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_100cm_right; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 7:
      foc_right(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_150cm_right; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 8:
      foc_right(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_200cm_right; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 9:
      foc_forward(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_50cm_forward; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 10:
      foc_forward(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_100cm_forward; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 11:
      foc_forward(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_150cm_forward; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 12:
      foc_backward(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_50cm_backward; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 13:
      foc_backward(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_100cm_backward; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 14:
      foc_backward(FOC_POWER_SET_1);
      for (int orientation_z_update = 0; orientation_z_update <= foc_distance_150cm_backward; orientation_z_update++){
        sensors_event_t orientationData;
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
          motor_tweak[0] = 0.99; //or motor_tweak[?] = motor_tweak[?] + or - 1;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.99;
        } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
          motor_tweak[0] = 0.99;
          motor_tweak[1] = 0.90;
          motor_tweak[2] = 0.99;
          motor_tweak[3] = 0.90;
        } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
          motor_tweak[0] = 0.90;
          motor_tweak[1] = 0.99;
          motor_tweak[2] = 0.90;
          motor_tweak[3] = 0.99;
        }
        delay(1);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    default: break;  // Handle invalid commands
  }
}
