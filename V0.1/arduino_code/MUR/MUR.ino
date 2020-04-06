/***************************************************************************
  Copyright 2020 LE CLUB SANDWICH STUDIO SAS

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.
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
#include <Adafruit_BME280.h>

Servo inValve;
Servo outValve;
Adafruit_BME280 bmePatient;
Adafruit_BME280 bmeAmbient;

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
int servoCal = 50;          // maximum movement of servo
int inValveLatency = 30;    // close the inValve slightly before opening outValve
int outValveLatency = 200;  // close the outValve slightly before opening outValve

// treshold values are arbitary set after consulting wikipedia. minimum 600hpa, maximum 1200hpa
float minimumAtmosphericPressure = 600.;
float maximumAtmosphericPressure = 1200.;
// Ring Alarm if differential Pressure goes above that
float peakAlarmLevel = 40.;
float plateauAlarmLevel = 30.;
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
  plateauPos = map(analogRead(Inspiratory), 0, 1023, 0, (servoCal / 3));
  // set baseline pressure, can only be opend until a certain point
  baselinePos = map(analogRead(Expiratory), 0, 1023, 0, (servoCal / 4));
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
  Serial.println(ovPos / 4);

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

void setup() {
  pinMode(Maintenance, INPUT_PULLUP);
  pinMode(PressureCal, INPUT_PULLUP);
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
  // read the bmp180 in regular intervals
  if (pressureSample < (millis() - bmpZero)) {
    // sample time before reading the sensor for a regular interval
    updateSensors();
  }

  if (digitalRead(Maintenance) == HIGH) {
    // read potentiometers and update values
    readPot();

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
  } else {
    if (digitalRead(PressureCal) == 0) {
      inValve.write(servoCal);
    } else {
      inValve.write(0);
    }
    outValve.write(0);
  }
  // please uncomment following line for serial debug or you will rapidly overflow Arduino
  // delay(200);
}
