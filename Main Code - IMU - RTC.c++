/*
MIT License

Copyright (c) 2024 gmarks24

Permission is hereby granted, free of charge, to any person obtaining a copy of 
this software and associated documentation files (the "Software"), to deal in 
the Software without restriction, including without limitation the rights to use, 
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
Software, and to permit persons to whom the Software is furnished to do so, subject
to the following conditions:

The above copyright notice and this permission notice shall be included in all 
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE 
USE OR OTHER DEALINGS IN THE SOFTWARE.
–––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––
Greg Marks 2024-2025 Science Oly Robot tour SSO-1 - Field Oriented Control(FOC)     
Made for Mecanum Drive                                                             
MCU - Arduino R4 Minima                                                            
MC - DFRobot DRI0039                                                                
Operation - 9V & Max 20.6Amps [Rec. 20-22awg wire]                                  
Rec. battery - 6(six) AA 1.5V                                                       
IMU - Adafruit 9-DOF Absolute Orientation IMU Fusion Breakout - BNO055

V2.1.0
12 Novermber 2024
*/
//#include "Arduino_LED_Matrix.h" /* Only needed if using Arduino R4 WiFi */
#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include "RTC.h"
#include <EEPROM.h>
const int P1 = 3;                                                    
const int P2 = 11;                                                
const int P3 = 5;                                                       
const int P4 = 6;                                                  
const int M1 = 4;                                                     
const int M2 = 12;                                                 
const int M3 = 8;                                                     
const int M4 = 7;                                                    
const int FOC_POWER_SET_1 = 255;                               
const int FOC_POWER_SET_2 = 0;                                      
int foc_distance_25cm = 500;                                         
int foc_distance_50cm_left = 1000;                              
int foc_distance_100cm_left = 2000;                              
int foc_distance_150cm_left = 3000;                                 
int foc_distance_200cm_left = 4000;                               
int foc_distance_50cm_right = 1000;                                     
int foc_distance_100cm_right = 2000;                                
int foc_distance_150cm_right = 3000;                           
int foc_distance_200cm_right = 4000;                             
int foc_distance_50cm_forward = 1000;                                
int foc_distance_100cm_forward = 2000;                          
int foc_distance_150cm_forward = 3000;                  
int foc_distance_200cm_forward = 4000;                         
int foc_distance_50cm_backward = 1000;   
int foc_distance_100cm_backward = 2000;                               
int foc_distance_150cm_backward = 3000;                                 
int foc_distance_200cm_backward = 4000;                                 
/*––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––*/
int motor_tune[] = { 0.0, 0.0, 0.0, 0.0}; 
int movement_list[] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 };  /*Always end with a '0' command to end the program.  Use to set robots movement=> <https://docs.google.com/document/d/13FU5d5H6w4hezWctLNM-Rf5GTDMTaTW0ypAhlw5teiY/edit>
/*––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––––*/
const int maxStages = sizeof(movement_list);                            
const int cool_down_time_set = 100;                                       /*milliseconds*/
int motor_tweak[] = { 0.99, 0.99, 0.99, 0.99 }; /* <<< DO NOT TWEAK THIS AT ALL!!! */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
void setup() {
  Serial.begin(115200);
  MOTORsetup();
  BNOsetup();
  RTCsetup();
  /*EEPROMsetup();*/
  delay(1000);
}
void MOTORsetup() {
  int pins[] = { P1, P2, P3, P4, M1, M2, M3, M4 };
  for (int i = 0; i < 8; i++) { pinMode(pins[i], OUTPUT); }
  Serial.println("Motor setup complete");
}
void EEPROMsetup() {
  while (!Serial.available()) {}
  for (int i = 0; i < EEPROM.length(); i++) {
    EEPROM.write(i, 0xFF);  // Write 0xFF to every address
  }
  //byte value = EEPROM.read(10);
  //EEPROM.write(10, 42);
  //int size = EEPROM.length();
}
void EEPROMdatalog() {}
void BNOsetup() {
  while (!Serial.available()) {}
  Serial.println("Orientation Sensor Test");
  Serial.println("");
  Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
  if (!bno.begin()) {
    Serial.print("No BNO055 detected, Check your wiring or I2C ADDR!");
    while (1);
  }
  Serial.println("BNO setup complete");
}
void RTCsetup() {
  RTC.begin();
  while (!Serial.available()) {}
  Serial.println("Formating RTC");
  while (Serial.available() == 0) {}
  int day = 1;
  int month = 1;
  int year = 2024;
  int hour = 1;
  int minute = 1;
  int second = 1;
  RTCTime startTime(day, (Month)month, year, hour, minute, second, DayOfWeek::MONDAY, SaveLight::SAVING_TIME_INACTIVE);
  RTC.setTime(startTime);
  Serial.println("RTC setup complete");
}
void pre_executionMovement() {
  foc_forward(FOC_POWER_SET_1);
  delay(foc_distance_25cm);
  foc_off(FOC_POWER_SET_2);
}
void loop() {
  if (Serial.available() > 0) {
    delay(1000);
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
void foc_off(int FOC_POWER_SET_2) {
  for (int pin : { P1, P2, P3, P4 }) { analogWrite(pin, FOC_POWER_SET_2); }
}
void foc_forward(int FOC_POWER_SET_1) {
  int pins[] = { M1, M2, M3, M4 };
  bool states[] = { HIGH, LOW, HIGH, LOW };
  for (int i = 0; i < 4; i++) { digitalWrite(pins[i], states[i]); }
  for (int pin : { P1, P2, P3, P4 }) { analogWrite(pin, FOC_POWER_SET_1 * motor_tweak[pin]); }
}
void foc_backward(int FOC_POWER_SET_1) {
  int pins[] = { M1, M2, M3, M4 };
  bool states[] = { LOW, HIGH, LOW, HIGH };
  for (int i = 0; i < 4; i++) { digitalWrite(pins[i], states[i]); }
  for (int pin : { P1, P2, P3, P4 }) { analogWrite(pin, FOC_POWER_SET_1 * motor_tweak[pin]); }
}
void foc_left(int FOC_POWER_SET_1) {
  int pins[] = { M1, M2, M3, M4 };
  bool states[] = { LOW, LOW, HIGH, HIGH };
  for (int i = 0; i < 4; i++) { digitalWrite(pins[i], states[i]); }
  for (int pin : { P1, P2, P3, P4 }) { analogWrite(pin, FOC_POWER_SET_1 * motor_tweak[pin]); }
}
void foc_right(int FOC_POWER_SET_1) {
  int pins[] = { M1, M2, M3, M4 };
  bool states[] = { HIGH, HIGH, LOW, LOW };
  for (int i = 0; i < 4; i++) { digitalWrite(pins[i], states[i]); }
  for (int pin : { P1, P2, P3, P4 }) { analogWrite(pin, FOC_POWER_SET_1 * motor_tweak[pin]); }
}
void left_orientation() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
    motor_tweak[0] = 0.99 - motor_tune[0];  //or motor_tweak[?] = motor_tweak[?] + or - 1;
    motor_tweak[1] = 0.99 - motor_tune[1];
    motor_tweak[2] = 0.99 - motor_tune[2];
    motor_tweak[3] = 0.99 - motor_tune[3];
  } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
    motor_tweak[0] = 0.99 - motor_tune[0];
    motor_tweak[1] = 0.90 - motor_tune[1];
    motor_tweak[2] = 0.90 - motor_tune[2];
    motor_tweak[3] = 0.99 - motor_tune[3];
  } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
    motor_tweak[0] = 0.90 - motor_tune[0];
    motor_tweak[1] = 0.99 - motor_tune[1];
    motor_tweak[2] = 0.99 - motor_tune[2];
    motor_tweak[3] = 0.90 - motor_tune[3];
  }
}

void right_orientation() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
    motor_tweak[0] = 0.99 - motor_tune[0];  //or motor_tweak[?] = motor_tweak[?] + or - 1;
    motor_tweak[1] = 0.99 - motor_tune[1];
    motor_tweak[2] = 0.99 - motor_tune[2];
    motor_tweak[3] = 0.99 - motor_tune[3];
  } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
    motor_tweak[0] = 0.99 - motor_tune[0];
    motor_tweak[1] = 0.90 - motor_tune[1];
    motor_tweak[2] = 0.90 - motor_tune[2];
    motor_tweak[3] = 0.99 - motor_tune[3];
  } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
    motor_tweak[0] = 0.90 - motor_tune[0];
    motor_tweak[1] = 0.99 - motor_tune[1];
    motor_tweak[2] = 0.99 - motor_tune[2];
    motor_tweak[3] = 0.90 - motor_tune[3];
  }
}
void forward_orientation() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
    motor_tweak[0] = 0.99 - motor_tune[0];  //or motor_tweak[?] = motor_tweak[?] + or - 1;
    motor_tweak[1] = 0.99 - motor_tune[1];
    motor_tweak[2] = 0.99 - motor_tune[2];
    motor_tweak[3] = 0.99 - motor_tune[3];
  } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
    motor_tweak[0] = 0.90 - motor_tune[0];
    motor_tweak[1] = 0.99 - motor_tune[1];
    motor_tweak[2] = 0.90 - motor_tune[2];
    motor_tweak[3] = 0.99 - motor_tune[3];
  } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
    motor_tweak[0] = 0.99 - motor_tune[0];
    motor_tweak[1] = 0.90 - motor_tune[1];
    motor_tweak[2] = 0.99 - motor_tune[2];
    motor_tweak[3] = 0.90 - motor_tune[3];
  }
}
void backward_orientation() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  if (orientationData.orientation.z > 359 || orientationData.orientation.z < 1) {
    motor_tweak[0] = 0.99 - motor_tune[0];  //or motor_tweak[?] = motor_tweak[?] + or - 1;
    motor_tweak[1] = 0.99 - motor_tune[1];
    motor_tweak[2] = 0.99 - motor_tune[2];
    motor_tweak[3] = 0.99 - motor_tune[3];
  } else if (orientationData.orientation.z <= 359 && orientationData.orientation.z >= 180) {
    motor_tweak[0] = 0.99 - motor_tune[0];
    motor_tweak[1] = 0.90 - motor_tune[1];
    motor_tweak[2] = 0.99 - motor_tune[2];
    motor_tweak[3] = 0.90 - motor_tune[3];
  } else if (orientationData.orientation.z >= 1 && orientationData.orientation.z <= 180) {
    motor_tweak[0] = 0.90 - motor_tune[0];
    motor_tweak[1] = 0.99 - motor_tune[1];
    motor_tweak[2] = 0.90 - motor_tune[2];
    motor_tweak[3] = 0.99 - motor_tune[3];
  }
}
int error = 0.09;
void executeMovement(int command) {
  RTCTime currentTime;
  unsigned long startTime;
  unsigned long targetTime;
  RTC.getTime(currentTime);
  startTime = currentTime.getUnixTime();
  switch (command) {
    case 1:
      targetTime = foc_distance_50cm_left;
      foc_left(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        left_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 2:
      targetTime = foc_distance_100cm_left;
      foc_left(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        left_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 3:
      targetTime = foc_distance_150cm_left;
      foc_left(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        left_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 4:
      targetTime = foc_distance_200cm_left;
      foc_left(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        left_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 5:
      targetTime = foc_distance_50cm_right;
      foc_right(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        right_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 6:
      targetTime = foc_distance_100cm_right;
      foc_right(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        right_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 7:
      targetTime = foc_distance_150cm_right;
      foc_right(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        right_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 8:
      targetTime = foc_distance_200cm_right;
      foc_right(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        right_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 9:
      targetTime = foc_distance_50cm_forward;
      foc_forward(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        forward_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 10:
      targetTime = foc_distance_100cm_forward;
      foc_forward(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        forward_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 11:
      targetTime = foc_distance_150cm_forward;
      foc_forward(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        forward_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 12:
      targetTime = foc_distance_50cm_backward;
      foc_backward(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        backward_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 13:
      targetTime = foc_distance_150cm_backward;
      foc_backward(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        backward_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    case 14:
      targetTime = foc_distance_150cm_backward;
      foc_backward(FOC_POWER_SET_1);
      while ((currentTime.getUnixTime() - startTime) < targetTime) {
        backward_orientation();
        RTC.getTime(currentTime);
      }
      foc_off(FOC_POWER_SET_2);
      delay(cool_down_time_set);
      break;
    default: break;  // Handle invalid commands
  }
}

