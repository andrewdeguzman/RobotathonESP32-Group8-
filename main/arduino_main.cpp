/****************************************************************************
Copyright 2021 Ricardo Quesada

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
****************************************************************************/

#include "sdkconfig.h"
#ifndef CONFIG_BLUEPAD32_PLATFORM_ARDUINO
#error "Must only be compiled when using Bluepad32 Arduino platform"
#endif  // !CONFIG_BLUEPAD32_PLATFORM_ARDUINO

#include <Arduino.h>
#include <Bluepad32.h>
#include <Wire.h>
#include <Arduino_APDS9960.h>
#include <ESP32Servo.h>
#include <ESP32SharpIR.h>
#include <QTRSensors.h>

#define MOTOR1_PIN1 27
#define MOTOR1_PIN2 26
#define ENABLE1_PIN 14
#define MOTOR2_PIN1 25
#define MOTOR2_PIN2 33
#define ENABLE2_PIN 32
#define APDS9960_INT 0
#define I2C_SDA     21
#define I2C_SCL     22
#define I2C_FREQ    100000

Servo myservo;
int pos = 0;

QTRSensors qtr;
uint16_t sensors[2];

TwoWire I2C_0 = TwoWire(0);
APDS9960 sensor = APDS9960(I2C_0, APDS9960_INT);

const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
int dutyCycle = 170;

GamepadPtr myGamepads[BP32_MAX_GAMEPADS];

ESP32SharpIR sharpIR(ESP32SharpIR::GP2Y0A41SK0F, 34);

// This callback gets called any time a new gamepad is connected.
void onConnectedGamepad(GamepadPtr gp) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == nullptr) {
            myGamepads[i] = gp;
            foundEmptySlot = true;
            break;
        }
    }
}

void onDisconnectedGamepad(GamepadPtr gp) {
    bool foundGamepad = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myGamepads[i] == gp) {
            myGamepads[i] = nullptr;
            foundGamepad = true;
            break;
        }
    }
}

// Arduino setup function. Runs in CPU 1
void setup() {
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    BP32.forgetBluetoothKeys();

    ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

    // TODO: Write your setup code here
	myservo.attach(13);
    qtr.setTypeAnalog(); // or setTypeAnalog()
    qtr.setSensorPins((const uint8_t[]) {32, 33}, 2); // pin numbers go in the curly brackets {}, and number of pins goes after

    //sets up I2C protocol
    I2C_0.begin(I2C_SDA, I2C_SCL, I2C_FREQ);

    //sets up color sensor
    sensor.setInterruptPin(APDS9960_INT);
    sensor.begin();

    IRSensorName.setFilterRate(1.0f);

    // calibration sequence
    for (uint8_t i = 0; i < 250; i++) { 
        Serial.println("calibrating");
        qtr.calibrate(); 
        delay(20);
    }

    pinMode(MOTOR1_PIN1, OUTPUT);
    pinMode(MOTOR1_PIN2, OUTPUT);
    pinMode(ENABLE1_PIN, OUTPUT);
    pinMode(MOTOR2_PIN1, OUTPUT);
    pinMode(MOTOR2_PIN2, OUTPUT);
    pinMode(ENABLE2_PIN, OUTPUT);

    Serial.begin(115200);
}

void moveNoob(Controller* controller) {
    if(controller && controller -> isConnected()) {
        if(controller -> axisRY() > 0) {
            digitalWrite(MOTOR1_PIN1, LOW);
            digitalWrite(MOTOR1_PIN2, HIGH);
            digitalWrite(MOTOR2_PIN1, LOW);
            digitalWrite(MOTOR2_PIN2, HIGH);
        }
        if(controller -> axisRY() == 0) {
            digitalWrite(MOTOR1_PIN1, LOW);
            digitalWrite(MOTOR1_PIN2, LOW);
            digitalWrite(MOTOR2_PIN1, LOW);
            digitalWrite(MOTOR2_PIN2, LOW);
        }
        if(controller -> axisY() > 0) {
            digitalWrite(MOTOR1_PIN1, LOW);
            digitalWrite(MOTOR1_PIN2, HIGH);
            digitalWrite(MOTOR2_PIN1, LOW);
            digitalWrite(MOTOR2_PIN2, HIGH);
        }
        if(controller -> axisY() == 0) {
            digitalWrite(MOTOR1_PIN1, LOW);
            digitalWrite(MOTOR1_PIN2, LOW);
            digitalWrite(MOTOR2_PIN1, LOW);
            digitalWrite(MOTOR2_PIN2, LOW);
        }
    }
}

void racism() {
    int r, g, b, a;
    // Wait until color is read from the sensor 
    while (!sensor.colorAvailable()) { delay(5); }
    sensor.readColor(r, g, b, a);

     String racist;
    if (r > g && r > b) {
        racist = "Red";
    } else if (g > r && g > b) {
        racist = "Green";
    } else if (b > r && b > g) {
        racist = "Blue";
    }

    Serial.print("Color: ");
    Serial.println(racist);

    //LED
}

//monkey see monkey follow line
void monkey() {
    qtr.readLineBlack(sensors); // Get calibrated sensor values returned in the sensors array
    int leftSensor = sensors[0];
    int rightSensor = sensors[1];
    int threshold = 500; // fix later

    if (leftSensor < threshold && rightSensor < threshold) {
        // Move forward
        digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR1_PIN2, HIGH);
        digitalWrite(MOTOR2_PIN1, LOW);
        digitalWrite(MOTOR2_PIN2, HIGH);
    } else if (leftSensor < threshold && rightSensor >= threshold) {
        // Turn right
        digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR1_PIN2, HIGH);
        digitalWrite(MOTOR2_PIN1, HIGH);
        digitalWrite(MOTOR2_PIN2, LOW);
    } else if (leftSensor >= threshold && rightSensor < threshold) {
        // Turn left
        digitalWrite(MOTOR1_PIN1, HIGH);
        digitalWrite(MOTOR1_PIN2, LOW);
        digitalWrite(MOTOR2_PIN1, LOW);
        digitalWrite(MOTOR2_PIN2, HIGH);
    } else {
        // Stop
        digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR1_PIN2, LOW);
        digitalWrite(MOTOR2_PIN1, LOW);
        digitalWrite(MOTOR2_PIN2, LOW);
    }

    Serial.print(sensors[0]);
    Serial.print(" ");
    Serial.println(sensors[1]);
    
    delay(250);
}
void covid() {
    float distance = IRSensorName.getDistanceFloat();
    Serial.print("Distance: ");
    Serial.println(distance);

    float desiredDistance = 20.0; // self-explanatory
    float tolerance = 5.0; // cm maybe

    if (distance < desiredDistance - tolerance) {
        // too close turn right
        digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR1_PIN2, HIGH);
        digitalWrite(MOTOR2_PIN1, HIGH);
        digitalWrite(MOTOR2_PIN2, LOW);
    } else if (distance > desiredDistance + tolerance) {
        // too far turn right
        digitalWrite(MOTOR1_PIN1, HIGH);
        digitalWrite(MOTOR1_PIN2, LOW);
        digitalWrite(MOTOR2_PIN1, LOW);
        digitalWrite(MOTOR2_PIN2, HIGH);
    } else {
        // go straight
        digitalWrite(MOTOR1_PIN1, LOW);
        digitalWrite(MOTOR1_PIN2, HIGH);
        digitalWrite(MOTOR2_PIN1, LOW);
        digitalWrite(MOTOR2_PIN2, HIGH);
    }

    delay(100);
}

// Arduino loop function. Runs in CPU 1
void loop() {
    BP32.update();

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        GamepadPtr myGamepad = myGamepads[i];
        if (myGamepad && myGamepad->isConnected()) {
            //use if-else with buttons to make the calls
            //eg. press button to trigger line follow function then hit another button to break out
            //joystick controll
            moveNoob(myGamepad);
        }
    }

    delay(250);

    vTaskDelay(1);
}
