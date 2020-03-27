/***************************************************************************
  Copyright 2020 LE CLUB SANDWICH STUDIO SAS

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

  http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
***************************************************************************/

/***************************************************************************
  !!!                        DISCLAMER                                 !!!
  ALARMS ARE NOT JET WORKING PROPERLY DUE TO TWO REASONS :
  - BME280 library is blocking the loop if the sensor fails
  - I didn't untangle all the program flow...

  BUT :
  It beeps when you exceed inspirationAlarmLevel and plateauAlarmlevel
  This provides a minimum of over pressure protection...
***************************************************************************/

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BME280.h>

Servo inValve;
Servo outValve;
Servo pressureValve;
Adafruit_BME280 bmePatient;
Adafruit_BME280 bmeAmbient;

// Servo Calibration, Alarm Levels and flags

/***************************************************************************
  // with a "standard" servo this value "should" be 180, please check with the
  // output valve that it's really 180° !!!
  // IF THIS IS NOT THE CASE :
  // - plug only the output Valve Servo
  // - !!! DON'T MOUNT ANY SERVO IN THE VALVES !!! You could damage the valves !!!
  // - enable the maintaince Switch so that the servo goes to 0°
  // - mark the 0° position somehow (marker, tape etc...)
  // - disable maintaince
  // - see if the servo makes exactly 160°
  // - IF NOT tweak the servoCal value to match 160°
  // - once the servo does exactly 180° save the sketch so that you keep it!
  //
  // HERE SOME STANDARD VALUES I FOUND FOR THE SERVOS I HAVE :
  // 9g standard Servos :         60
  // RD3115MG Robotic Servos :    40-50
***************************************************************************/
int servoCal = 60;
int inValveLatency = 30;    // close the inValve slightly before opening outValve
int outValveLatency = 200;  // close the outValve slightly before opening outValve

// treshold values are arbitary set after consulting wikipedia. minimum 600hpa, maximum 1200hpa
float minimumAtmosphericPressure = 600.;
float maximumAtmosphericPressure = 1200.;
// Ring Alarm if differential Pressure goes above that
float inspirationAlarmLevel = 40.;
float plateauAlarmLevel = 30.;
bool inspirationAlarm = 0;
bool plateauAlarm = 0;

// Potentiometer and Servo pins
int inValvePin = 2;     // Pin for input Valve
int outValvePin = 3;    // Pin for Output Valve
int Led = 5;            // future neopixel Led for user feedback
int Buzzer = 6;         // Alarm Buzzer
int Maintenance = 7;    // sets all valves to 0° for maintaince
int TarePressure = 8;   // possible feature to tare pressure sensors when system not under pressure
int Cycles = A0;
int Ratio = A1;
int Peak = A2;
int Inspiratory = A3;
int Expiratory = A6;

// different timers in our breathing cycle
unsigned long cycle = 0;
unsigned long inspirationT = 0;
unsigned long plateauT = 0;
unsigned long expirationT = 0;

// running timers to get asycronous loop
unsigned long cycleZero = 0;
unsigned long bmpZero = 0;

//running cycle flags
bool inspiration = 0;
bool plateau = 0;
bool expiration = 0;

// Servo positions and others
int ivPos = 0;
int ovPos = servoCal;
int pvPos = 0;
int plateauPos = 0;
int baselinePos = 0;
float bmpP = 0.;
float bmeP = 0.;
float differentialP = 0.;
// pressure sensor sample Frequency = 20ms runs smooth on teensy3.2
unsigned long bmpSample = 20;
bool pressureSensorFailure = 0;

// Read all potentiometers and adjust values
void readPot() {
  // calculate total breath cycle lenght
  cycle = map(analogRead(Cycles), 0, 1023, 6000, 2000);
  // calculate the time of the inspiration cycle including plateau
  plateauT = map(analogRead(Ratio), 0, 1023, (cycle / 2), (cycle / 4));
  // add peak if necessairy
  int tempI = analogRead(Peak);
  // impementation of "dead Zone" on the low end of the peak potentiometer 
  if (tempI <= 100) {
    inspirationT = 0;
  } else {
    // inspiration Peak can not go higher than 1/10th of inspiration!
    inspirationT = map(tempI, 100, 1023, 50, (plateauT / 10));
  }

  // set plateau support pressure
  plateauPos = map(analogRead(Inspiratory), 0, 1023, 0, servoCal);
  // set baseline pressure, can only be opend until a certain point
  baselinePos = map(analogRead(Expiratory), 0, 1023, 0, (servoCal / 2));
}

void updateSensors() {
  // reset timer first for regular intervals!
  bmpZero = millis();
  // read Sensors, get differential
  bmePatient.takeForcedMeasurement();
  bmpP = (float)bmePatient.readPressure() / 100.;
  bmeAmbient.takeForcedMeasurement();
  bmeP = (float)bmeAmbient.readPressure() / 100.;
  differentialP = bmpP - bmeP;

  // check for sensor failiure, this alarm doesn't care if another alarm is active.
  // this is due to the fact that if one sensor fails the other alarms won't work anymore...
  if ((bmpP <= minimumAtmosphericPressure) || (bmpP >= maximumAtmosphericPressure) || (bmeP <= minimumAtmosphericPressure) || (bmeP >= maximumAtmosphericPressure)) {
    pressureSensorFailure = 1;
    digitalWrite(Buzzer, HIGH);
  }
  // check for realistic pressures if pressureSensorFailure is active
  if (pressureSensorFailure == 1) {
    if ((bmpP > minimumAtmosphericPressure) && (bmpP < maximumAtmosphericPressure) && (bmeP > minimumAtmosphericPressure) && (bmeP < maximumAtmosphericPressure)) {
      pressureSensorFailure = 0;
      digitalWrite(Buzzer, LOW);
    }
  }

  // this is a more userfriendly graph
  Serial.print(differentialP);
  Serial.print("\t");
  Serial.print(ivPos / 4);
  Serial.print("\t");
  Serial.print(ovPos / 4);
  Serial.print("\t");
  Serial.println(pvPos / 4);

  // this is more like a debug graph
  /*Serial.print(bmpP);
    Serial.print("\t");
    Serial.print(bmeP);
    Serial.println("\t");
    // do some funny math to see valve positions on the graphs on Serial Plotter
    Serial.print((float)(101000 + (ivPos * 10)) / 100.);
    Serial.print("\t");
    Serial.print((float)(101000 + (ovPos * 10)) / 100.);
    Serial.print('\t');
    Serial.print((float)(101000 + (pvPos * 10)) / 100.);
    Serial.print('\t');
    Serial.print(101000 + (baselinePos * 10));
    Serial.print('\t');
    Serial.println(101000 + (plateauPos * 10));
  */
}

void setup() {
  pinMode(Maintenance, INPUT_PULLUP);
  pinMode(Buzzer, OUTPUT);
  digitalWrite(Buzzer, LOW);
  Serial.begin(115200);
  bool P1 = bmePatient.begin(0x77);
  if (!P1) {
    pressureSensorFailure = 1;
    //Serial.println("Could not find a valid BMP180 sensor, check wiring, address, sensor ID!");
  }
  // configure bme280 : mode, tempSampling, pressSampling, humSampling, filter, duRation
  bmePatient.setSampling(1, 1, 3, 1, 1, 10);
  bool P2 = bmeAmbient.begin(0x76);
  if (!P2) {
    pressureSensorFailure = 1;
    //Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
  }
  bmeAmbient.setSampling(1, 1, 3, 1, 1, 10);
  // if the two pressure sensors are started for good light the led in green, attach servos and wait a bit
  inValve.attach(inValvePin);
  outValve.attach(outValvePin);
  inValve.write(ivPos);
  outValve.write(ovPos);
  readPot();
}

void loop() {
  if (digitalRead(Maintenance) == HIGH) {
    // read potentiometers and update values
    readPot();

    // read the bmp180 in regular intervals
    if (bmpSample < (millis() - bmpZero)) {
      // sample time before reading the sensor for a regular interval
      updateSensors();
    }

    // here comes the breathing cycle
    // start cycle
    if ((inspiration == 0) && (plateau == 0) && (expiration == 0)) {
      cycleZero = millis();
      inspiration = 1;
      //Serial.println("Start cycle");
    }

    // inspiration
    if ((inspiration == 1) && (plateau == 0) && (expiration == 0)) {
      // if inValve is not jet open on start
      if (ivPos != servoCal) { // && (inspirationT <= (millis() - cycleZero))) {
        ivPos = servoCal;
        inValve.write(ivPos);
        // Serial.print("Open Input Valve\t");
        // Serial.println((int)millis() - cycleZero);
      }

      // check for plateau Alarm if no other Alarm is active
      if (!pressureSensorFailure && !plateauAlarm) {
        if (differentialP > inspirationAlarmLevel) {
          digitalWrite(Buzzer, HIGH);
          inspirationAlarm = 1;
        } else {
          digitalWrite(Buzzer, LOW);
          inspirationAlarm = 0;
        }
      }

      // once inspiration is finished close input valve and set flags
      if (inspirationT <= (millis() - (cycleZero - inValveLatency))) {
        ivPos = plateauPos;
        inValve.write(ivPos);
        // Serial.print("Close Input Valve\t");
        // Serial.println((int)millis() - cycleZero);
      }
      if (inspirationT <= (millis() - cycleZero)) {
        plateau = 1;
      }
    }

    // plateau
    if ((inspiration == 1) && (plateau == 1) && (expiration == 0)) {
      if (plateauT <= (millis() - cycleZero)) {
        ivPos = 0;
        inValve.write(baselinePos);
        ovPos = servoCal;
        outValve.write(ovPos);
        expiration = 1;
        // Serial.print("Open Output Valve\t");
        // Serial.println((int)millis() - cycleZero);

        // keep track of any control changes on plateau pressure
      } else if (plateauPos != ivPos) {
        ivPos = plateauPos;
        inValve.write(ivPos);
      }

      // check for plateau Alarm if no other Alarm is active
      if (!pressureSensorFailure && !inspirationAlarm) {
        // activate Buzzer if there is to much pressure during plateau
        if (differentialP > plateauAlarmLevel) {
          digitalWrite(Buzzer, HIGH);
          plateauAlarm = 1;
        } else {
          digitalWrite(Buzzer, LOW);
          plateauAlarm = 0;
        }
      }
    }

    // cycle finished
    if ((inspiration == 1) && (plateau == 1) && (expiration == 1)) {
      //close outputValve before the end of the cycle
      if (cycle <= (millis() - (cycleZero - outValveLatency))) {
        //close Output valve
        if (ovPos != 0) {
          ovPos = 0;
          outValve.write(ovPos);
          //Serial.println("Close Output Valve");
        }
      }
      if (cycle <= (millis() - cycleZero)) {
        inspiration = 0;
        plateau = 0;
        expiration = 0;
        // Serial.print("Cycle Finished\t\t");
        // Serial.println((int)millis() - cycleZero);
        // Serial.println();

        // same here, keep track of any control changes on baseline pressure
      } else if (baselinePos != ivPos) {
        ivPos = baselinePos;
        inValve.write(ivPos);
      }
    }
  } else {
    inValve.write(0);
    outValve.write(0);
    pressureValve.write(0);
  }
  // please uncomment following line for serial.debug
  // delay(200);
}
