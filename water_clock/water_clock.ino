/*
   -------------------------------------------------------------------------------------
   HX711_ADC
   Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales
   Olav Kallhovd sept2017
   -------------------------------------------------------------------------------------
*/

/*
   Settling time (number of samples) and data filtering can be adjusted in the config.h file
   For calibration and storing the calibration value in eeprom, see example file "Calibration.ino"

   The update() function checks for new data and starts the next conversion. In order to acheive maximum effective
   sample rate, update() should be called at least as often as the HX711 sample rate; >10Hz@10SPS, >80Hz@80SPS.
   If you have other time consuming code running (i.e. a graphical LCD), consider calling update() from an interrupt routine,
   see example file "Read_1x_load_cell_interrupt_driven.ino".

   This is an example sketch on how to use this library
*/

#include <LCD5110_Graph.h>
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#include <Wire.h>
#include <RTClib.h>

#endif

// Constants
const int ALARM_FREQ_SECONDS = 20 * 60;  // Used to set and update alarms
const float FILL_THRESHOLD = 150.0;
const float BUFFER_ZONE = 5.0;
const int SPEAKER_PIN = 9;

//Time chip constrcutor
RTC_DS3231 rtc;

//Nokia LCD Screen globals
LCD5110 myGLCD(2,3,4,6,5);
extern uint8_t BigNumbers[];
extern uint8_t SmallFont[];
extern uint8_t MediumNumbers[];
extern uint8_t TinyFont[];

//HX711 constructor:
const int HX711_dout = 12; //mcu > HX711 dout pin
const int HX711_sck = 13; //mcu > HX711 sck pin
const int calVal_eepromAdress = 0;
HX711_ADC LoadCell(HX711_dout, HX711_sck);

//save the next alarm
DateTime next_alarm;

// Init the load cell, zero out the scale on power up
// Init the LCD screen
void setup() {
  Serial.begin(57600); 
  delay(10);
  Wire.begin();

  LoadCell.begin();
  float calibrationValue; // calibration value (see example file "Calibration.ino")
  calibrationValue = 293.09; // uncomment this if you want to set the calibration value in the sketch
  unsigned long stabilizingtime = 2000; // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }

  //Initialize the real time clock
  if(!rtc.begin()) {
      Serial.println("Couldn't find RTC!");
      Serial.flush();
      while (1) delay(10);
  }

  // Clear alarms
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  // Init the alarm, match the seconds to the time span
  // Initialize the LCD screen and small font  
  myGLCD.InitLCD();
}


//Program vars 
unsigned long t = 0;
float mls_drank = 0;
float weight_reading = 0;

enum cup_state {
  filled,
  empty,
  removed
};

enum alarm_state {
  ringing,
  ready,
  off
};

cup_state state = empty;
boolean alarm = false;
boolean alarm_counting = false;
int cupdelay = 10;
int cupsdrank = 0;

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 10; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell.update()) newDataReady = true;

  // get smoothed weight reading
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      weight_reading = LoadCell.getData();
      newDataReady = 0;
      t = millis();
    }
  }

    // Decide on the states
    // Only run the alarm if the cup is filled (significantly higher than the buffer value)
    if(weight_reading > BUFFER_ZONE){
      state = filled;
      cupdelay = 10;
      if(!alarm_counting){
        alarm_counting = true;
        next_alarm = resetAlarm(&rtc, ALARM_FREQ_SECONDS);  
      }
    } else if (weight_reading < -1 * BUFFER_ZONE){
      state = removed;
      cupdelay = 10;
    } else {
      if (cupdelay < 0){
        state = empty;
      }
      else{
        cupdelay -= 1;
      }
    }
 
  // Play the alarm
  if(rtc.alarmFired(1)) {
      alarm = true;
      analogWrite(SPEAKER_PIN, 5);
  } else {
    analogWrite(SPEAKER_PIN, 0);  
  }


  if(alarm){
    if(state == empty){
      disableAlarm(&rtc);
      alarm_counting = false;
      alarm = false;
      cupsdrank += 1;
    }
  }

  //Print to the LCD
  myGLCD.clrScr();
  myGLCD.setFont(SmallFont);
  String time_left_str = "";

  // update time left string for displaying to the user
  if(alarm_counting){
    TimeSpan time_left = next_alarm - rtc.now();
    
    if(time_left.totalseconds() > -1 ){
      time_left_str = String(time_left.hours()) + ":" + String(time_left.minutes()) + ":" + String(time_left.seconds());
    } else {
      time_left_str = "Now!";
    }
    
    myGLCD.print("Drink:" + time_left_str, 0, 0);
  }
  
  myGLCD.print("MLs:" + String(cupsdrank*237), 0, 24);
  myGLCD.print("Cups:" + String(cupsdrank), 0, 34);
  myGLCD.update();
}


DateTime resetAlarm(RTC_DS3231 * rtc, int duration){
    rtc->clearAlarm(1);
    rtc->clearAlarm(2);

    DateTime next = rtc->now() + TimeSpan(duration);
    rtc->setAlarm1(next, DS3231_A1_Hour ); 
    return next;
}


void disableAlarm(RTC_DS3231 * rtc){
    rtc->clearAlarm(1);
    rtc->clearAlarm(2);
}
