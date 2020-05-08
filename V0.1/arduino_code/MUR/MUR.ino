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
  It beeps when you exceed peakAlarmLevel and plateauAlarmlevel
  This provides a minimum of over pressure protection...
***************************************************************************/

#include <Servo.h>
#include <Wire.h>
#include "SparkFunBME280.h"

Servo inValve;
Servo outValve;
BME280 bmePatient;
BME280 bmeAmbient;

// Servo Calibration, Alarm Levels and flags

/***************************************************************************
  //  THERE IS NO "STANDARD SERVO OR AIRSOURCE" IN THIS PROJECT FOR INSTANCE
  //  DUE TO THE DIFFICULTY TO GET RELIABLY PARTS DURING LOCKDOWN
  //
  //  !!! WE ARE USING FUTABA S3003 SERVOS IN THIS PROTOTYPE with V2.1 VALVES !!!
  //
  //  Please follow the steps carefully to calibrate your device to
  //  your Servos and Airsource. Wrong calibration can be harmful or even
  //  kill a person! If you run into problems :
  //  Common sense, physics and anatomy knowledge may help!
  //
  //
  //  1. Mount and calibrate Valves :
  //
  //  - !!! DON'T MOUNT ANY SERVO IN THE VALVES !!! You could damage the valves
  //  - Plug the servos and power up the device
  //  - enable the maintaince switch so that the servo goes to 0° (closed valven position)
  //  - close the Valve by Hand and check for air tight
  //  - reapeatly check for easy-moving and air tight
  //  - Mount the servos into the valves
  //  - disable Maintaince Switch and check for each valve if they are air tight
  //  - your system usually has leaks due to 3d printing
  //  tolerance. make shure they are as small as possible!
  //
  //
  //  2. Calibrate absolute Pressure from Airsource and Servo movement :
  //
  //  - Connect all "plumbing" to the device.
  //  - Close the patient side hole with something air tight!
  //  - Open overpressure Valve (V3) on the filterbox
  //  - engage maintaince and calibration Switches
  //  - Power up the device, WITHOUT AIRSOURCE!
  //  - inValve should open, outValve is closed
  //  - power up or connect your Airsource
  //  - Open Arduino Serial Plotter
  //  - adjust the overpressure valves until reach the desired pressure
  //  (usually i close almost completly the little valve and I progressifly close
  //  the valve on the filter box...)
  //  - switch back maintaince and calibration Switches to normal position
  //  - control pressure levels over several cycles and adjust if necessary
  //  - play with the controls, be shure the pressure is ok, check peak, plateau
  //  and expiratory individually
  //  - if the valves open to wide, lower the servoCal value (less servo wear),
  //  - if the pressure is to low rise the servoCal Value
  //  (carefully! your servo will wear more)
  //
  //  BE SURE THAT YOUR AIRSOURCE IS STABLE OVER TIME, WE OBSERVED THAT
  //  TURBINE AIRSOURCES LOSE MORE POWER ONCE HOT THAN MEMBRANE PUMPS
  //
  //  !!! ATTENTION THIS A MEDICAL DEVICE, YOUR JOB AS MAKER IS ONLY TO MAKE
  //  AND CALIBRATE THE DEVICE, NOT TO OPERATE THE DEVICE !!!
***************************************************************************/

//  Please adapt these three values to your Servos and Airsource. We use Futaba S3003 Servos.
int servoCal = 90;          // maximum movement of servo
int inValveLatency = 30;    // close the inValve slightly before opening outValve
int outValveLatency = 400;  // close the outValve slightly before opening outValve

// treshold values are arbitary set after consulting wikipedia. minimum 600hpa, maximum 1200hpa
float minimumAtmosphericPressure = 600.;
float maximumAtmosphericPressure = 1200.;
// Ring Alarm if differential Pressure goes above that
float peakAlarmLevel = 40.;
float plateauAlarmLevel = 30.;
float maximumPressure = 1.;
bool peakAlarm = 0;
bool plateauAlarm = 0;

// Potentiometer and Servo pins
int inValvePin = 2;     // Pin for input Valve
int outValvePin = 3;    // Pin for Output Valve
int Led = 5;            // future neopixel Led for user feedback
int Buzzer = 6;         // Alarm Buzzer
int Maintenance = 7;    // sets all valves to 0° for maintaince
int PressureCal = 8;  // closes outputValve and opens Input Valve for maximum pressure calibration
int Cycles = A0;
int Ratio = A1;
int Peak = A6;
int Inspiratory = A7;
int Expiratory = A8;

// different timers in our breathing cycle
unsigned long cycle = 0;
unsigned long peakT = 0;
unsigned long plateauT = 0;
unsigned long expirationT = 0;

// running timers to get asycronous loop
unsigned long cycleZero = 0;
unsigned long bmpZero = 0;

//running cycle flags
bool peak = 0;
bool plateau = 0;
bool expiration = 0;
bool pressureMaxCal = 0;
bool pressureCal = 0;

// Servo positions and others
int ivPos = 0;
int ovPos = servoCal;
int pvPos = 0;
int plateauPos = 0;
int baselinePos = 0;
float bmeP = 0.;
float bmeA = 0.;
float differentialP = 0.;
float sensorTare = 0.;
// pressure sensor sample Frequency = 20ms runs smooth on teensy3.2
unsigned long pressureSample = 20;
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
  // the "inValveLatency * 3" is just some rule of thumb, depends heavily on the Servo used
  if (tempI <= (inValveLatency * 3)) {
    peakT = 0;
  } else {
    // inspiration Peak can not go higher than 1/10th of inspiration!
    peakT = map(tempI, 100, 1023, 50, (plateauT / 10));
  }

  // set plateau support pressure
  plateauPos = map(analogRead(Inspiratory), 0, 1023, 0, (servoCal));// / 3));
  // set baseline pressure, can only be opend until a certain point
  baselinePos = map(analogRead(Expiratory), 0, 1023, 0, (servoCal / 4));
}

void updateSensors() {
  // reset timer first for regular intervals!
  bmpZero = millis();

  // read Sensors, get differential
  //bmePatient.takeForcedMeasurement();
  bmeP = bmePatient.readFloatPressure() / 100.;//(float)bmePatient.readPressure() / 100.;

  //bmeAmbient.takeForcedMeasurement();
  bmeA = bmeAmbient.readFloatPressure() / 100.;//(float)bmeAmbient.readPressure() / 100.;

  differentialP = bmeP - bmeA;
  if ((differentialP < -30.) || (differentialP > 150.)) {
    differentialP = 0.;
    pressureSensorFailure = 1;
    digitalWrite(Buzzer, HIGH);
  }
  if (!pressureCal) {
    differentialP += sensorTare;
  }

  // check for sensor failiure, this alarm doesn't care if another alarm is active.
  // this is due to the fact that if one sensor fails the other alarms won't work anymore...
  if ((bmeP <= minimumAtmosphericPressure) || (bmeP >= maximumAtmosphericPressure) || (bmeA <= minimumAtmosphericPressure) || (bmeA >= maximumAtmosphericPressure)) {
    pressureSensorFailure = 1;
    digitalWrite(Buzzer, HIGH);
  }
  // check for realistic pressures if pressureSensorFailure is active
  if (pressureSensorFailure == 1) {
    // check if sensor was reconnected
    initBME();
    if ((bmeP > minimumAtmosphericPressure) && (bmeP < maximumAtmosphericPressure) && (bmeA > minimumAtmosphericPressure) && (bmeA < maximumAtmosphericPressure)) {
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
  if (pressureMaxCal) {
    Serial.print("\t");
    Serial.print(maximumPressure);
  }
  if (PressureCal) {
    Serial.print("\t");
    Serial.print(sensorTare);
  }
  Serial.println();

  // this is more like a debug graph
  /*Serial.print(bmpP);
    Serial.print("\t");
    Serial.print(bmeP);
    Serial.println("\t");
    // do some funny math to see valve positions on the graphs on Serial Plotter
    // made at about 60m above sea level.
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

void initBME() {
  bmePatient.setI2CAddress(0x77);
  if (!bmePatient.isMeasuring()) {
    Serial.println("Patient side Sensor HS !");
    if (bmePatient.beginI2C() == false) {
      pressureSensorFailure = 1;
      Serial.println("Could not find a valid Patient BME280 sensor, check wiring, address, sensor ID!");
    }
    bmePatient.setFilter(1); //0 to 4 is valid. Filter coefficient. See 3.4.4
    bmePatient.setStandbyTime(0); //0 to 7 valid. Time between readings. See table 27.
    bmePatient.setTempOverSample(0); //0 to 16 are valid. 0 disables temp sensing. See table 24.
    bmePatient.setPressureOverSample(4); //0 to 16 are valid. 0 disables pressure sensing. See table 23.
    bmePatient.setHumidityOverSample(0); //0 to 16 are valid. 0 disables humidity sensing. See table 19.
    bmePatient.setMode(MODE_NORMAL);
  }

  bmeAmbient.setI2CAddress(0x76);
  if (!bmeAmbient.isMeasuring()) {
    Serial.println("Ambient side Sensor HS !");
    if (bmeAmbient.beginI2C() == false) {
      pressureSensorFailure = 1;
      Serial.println("Could not find a valid Ambient BME280 sensor, check wiring, address, sensor ID!");
    }
    bmeAmbient.setFilter(1); //0 to 4 is valid. Filter coefficient. See 3.4.4
    bmeAmbient.setStandbyTime(0); //0 to 7 valid. Time between readings. See table 27.
    bmeAmbient.setTempOverSample(0); //0 to 16 are valid. 0 disables temp sensing. See table 24.
    bmeAmbient.setPressureOverSample(4); //0 to 16 are valid. 0 disables pressure sensing. See table 23.
    bmeAmbient.setHumidityOverSample(0); //0 to 16 are valid. 0 disables humidity sensing. See table 19.
    bmeAmbient.setMode(MODE_NORMAL);
  }
  // whait a bit to ensure sensor startup
  delay(20);
}

void setup() {
  pinMode(Maintenance, INPUT_PULLUP);
  pinMode(PressureCal, INPUT_PULLUP);
  pinMode(Buzzer, OUTPUT);
  digitalWrite(Buzzer, LOW);
  Serial.begin(115200);
  //while(!Serial) delay(100);
  delay(1000);
  Wire.begin();
  Wire.setClock(400000);
  initBME();

  inValve.attach(inValvePin);
  outValve.attach(outValvePin);
  inValve.write(ivPos);
  outValve.write(ovPos);
  readPot();
}

void loop() {
  // read the bmp180 in regular intervals
  if (pressureSample < (millis() - bmpZero)) {
    // sample time before reading the sensor for a regular interval
    updateSensors();
    // read potentiometers and update values
    readPot();
  }
  if (!digitalRead(Maintenance) && digitalRead(PressureCal)){
    ivPos = 0;
    inValve.write(ivPos);
    ovPos = 0;
    outValve.write(ovPos);
  } else if (!digitalRead(Maintenance) && !digitalRead(PressureCal)){
    maximumPressure = 0.5;
    unsigned long timer = millis();
    pressureMaxCal = 1;
    //loop here to get maximum pressure
    while (!digitalRead(Maintenance) && !digitalRead(PressureCal)){
      if (pressureSample < (millis() - bmpZero)) {
        // sample time before reading the sensor for a regular interval
        updateSensors();
        if(differentialP > (maximumPressure * 0.98)){
          maximumPressure = (0.5 * differentialP + ((1 - 0.5) * maximumPressure));
        }
      }
      //toggle valve positions regularlyto evaluate maximum pressure avaliable
      if (millis() >= (timer + 1500)){
        if (ivPos == 0){
          timer = millis();
          ivPos = servoCal;
          inValve.write(ivPos);
          ovPos = 0;
          outValve.write(ovPos);
        } else {
          timer = millis();
          ivPos = 0;
          inValve.write(ivPos);
          ovPos = servoCal;
          outValve.write(ovPos);

        }
      }
    }
    pressureMaxCal = 0;
    // save maximumPressure here
  } else if (digitalRead(Maintenance) && !digitalRead(PressureCal)) {
    // close input valve and open outputvalve, reset sensorTare
    ivPos = 0;
    inValve.write(ivPos);
    ovPos = servoCal;
    outValve.write(ovPos);
    sensorTare = 0.;
    pressureCal = 1;
    float tempTare = 0.;
    // wait a bit to depressurize cirquit
    delay(2000);
    // get differential between the two sensors
    while (digitalRead(Maintenance) && !digitalRead(PressureCal)) {
      if (pressureSample < (millis() - bmpZero)) {
        // sample time before reading the sensor for a regular interval
        updateSensors();
        tempTare = (0.5 * differentialP + ((1 - 0.5) * tempTare));
      }
    }
    pressureCal = 0;
    sensorTare = -tempTare;

  } else if (digitalRead(Maintenance) && digitalRead(PressureCal)) {

    // here comes the breathing cycle
    // start cycle
    if ((peak == 0) && (plateau == 0) && (expiration == 0)) {
      cycleZero = millis();
      peak = 1;
      //Serial.println("Start cycle");
    }

    // peak and transition to plateau
    if ((peak == 1) && (plateau == 0) && (expiration == 0)) {
      // if inValve is not jet open on start
      if (ivPos != servoCal) {
        ivPos = servoCal;
        inValve.write(ivPos);
        // Serial.print("Open Input Valve\t");
        // Serial.println((int)millis() - cycleZero);
      }

      // check for plateau Alarm if no other Alarm is active
      if (!pressureSensorFailure && !plateauAlarm) {
        if (differentialP > peakAlarmLevel) {
          digitalWrite(Buzzer, HIGH);
          peakAlarm = 1;
        } else {
          digitalWrite(Buzzer, LOW);
          peakAlarm = 0;
        }
      }

      // once inspiration is finished close input valve and set flags
      if (peakT <= (millis() - (cycleZero - inValveLatency))) {
        ivPos = plateauPos;
        inValve.write(ivPos);
        // Serial.print("Close Input Valve\t");
        // Serial.println((int)millis() - cycleZero);
      }
      if (peakT <= (millis() - cycleZero)) {
        plateau = 1;
      }
    }

    // plateau and transition to expiration
    if ((peak == 1) && (plateau == 1) && (expiration == 0)) {
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
      if (!pressureSensorFailure && !peakAlarm) {
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

    // end of cycle
    if ((peak == 1) && (plateau == 1) && (expiration == 1)) {
      //close outputValve slightly before the end of the cycle
      if (cycle <= (millis() - (cycleZero - outValveLatency))) {
        //close Output valve
        if (ovPos != 0) {
          ovPos = 0;
          outValve.write(ovPos);
          //Serial.println("Close Output Valve");
        }
      }
      if (cycle <= (millis() - cycleZero)) {
        peak = 0;
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
  }/* else {
    if (!digitalRead(Maintenance) && digitalRead(pressureCal)) {
      ivPos = 0;
      inValve.write(ivPos);
      ovPos = 0;
      outValve.write(ovPos);
    } else if (!digitalRead(Maintenance) && !digitalRead(pressureCal)) {
      ivPos = servoCal;
      inValve.write(ivPos);
      ovPos = 0;
      outValve.write(ovPos);

      ivPos = servoCal;
      inValve.write(ivPos);
    } else {
      ivPos = 0;
      inValve.write(ivPos);

    }
    ovPos = 0;
    outValve.write(ovPos);
  }*/

  // please uncomment following line for serial debug or you will rapidly overflow Arduino
  //delay(200);
}
