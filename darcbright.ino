/* darcbright - A custom firmware for the Hexbright flashlight.
** Based on `hexbright4` from https://github.com/hexbright/hexbright-samples
** Modifications authored by Robert Quattlebaum <darco@deepdarc.com>
**
** Added Features:
**  * Smooth brightness transitions.
**  * Increases in brightness happen on the button press, decreases on button release.
**  * Smooth pulsing charging indicator.
**  * Accelerometer filtering.
**  * Mode changing by holding down button. (4 "modes": constant, momentary, blinky, knob)
**  * Overtemp condition now attempts to throttle back brightness instead of just turning off.
**
** TODO:
**  * Debounce power-on button press.
*/

#include <math.h>
#include <Wire.h>
#include "pt.h"
#include <EEPROM.h>

// Settings
// Constants
#define ACC_ADDRESS             0x4C
#define ACC_REG_XOUT            0
#define ACC_REG_YOUT            1
#define ACC_REG_ZOUT            2
#define ACC_REG_TILT            3
#define ACC_REG_INTS            6
#define ACC_REG_MODE            7
// Pin assignments
#define DPIN_RLED_SW            2
#define DPIN_GLED               5
#define DPIN_PGOOD              7
#define DPIN_PWR                8
#define DPIN_DRV_MODE           9
#define DPIN_DRV_EN             10
#define DPIN_ACC_INT            3
#define APIN_TEMP               0
#define APIN_CHARGE             3
// Interrupts
#define INT_SW                  0
#define INT_ACC                 1
// Modes
#define MODE_POWERUP            0
#define MODE_OFF                1
#define MODE_LOW                2
#define MODE_MED                3
#define MODE_HIGH               4
#define MODE_KNOBBING           5
#define MODE_KNOBBED            6
#define MODE_BLINKING           7
#define MODE_BLINKING_PREVIEW   8
#define MODE_DAZZLING           9
#define MODE_DAZZLING_PREVIEW   10

// State
//byte mode = 0;
//unsigned long btnTime = 0;
//boolean btnDown = false;
boolean overtemp_throttle;
byte overtemp_max = 255;
byte light_mode;

struct pt fade_control_pt;
struct pt power_pt;

struct pt light_pt;
struct pt light_momentary_pt;
struct pt light_blinky_pt;
struct pt light_knob_pt;



byte amount_current;
byte amount_begin;
byte amount_end;
unsigned short amount_fade_duration;
unsigned long amount_fade_start;
byte amount_flash,amount_off;

unsigned long button_pressed_time;
unsigned long button_released_time;
unsigned long button_pressed_duration;
unsigned long button_released_duration;

short readVcc() {
  short result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

void
retrieve_settings(void) {
  byte data[4];
  data[0] = EEPROM.read(0);
  data[1] = EEPROM.read(1);
  data[2] = EEPROM.read(2);
  data[3] = EEPROM.read(3);
  if(data[0] == (byte)~(data[1]^data[2]^data[3] + sizeof(data))) {
    light_mode = data[2];
    if(light_mode) {
      set_amount(data[1]);
      digitalWrite(DPIN_DRV_MODE,data[3]);
    }
    Serial.println("Settings retrieved");
  }
}

void
save_settings(void) {
  byte data[4];
  data[1] = amount_current;
  data[2] = light_mode;
  data[3] = digitalRead(DPIN_DRV_MODE);
  data[0] = ~(data[1]^data[2]^data[3] + sizeof(data));
  EEPROM.write(0,data[0]);
  EEPROM.write(1,data[1]);
  EEPROM.write(2,data[2]);
  EEPROM.write(3,data[3]);
}

void
set_amount(byte amount) {
  amount_current = amount_begin = amount_end = amount;
  amount_fade_duration = 0;
  amount_off = 0;
  analogWrite(DPIN_DRV_EN,amount_current);
  pinMode(DPIN_PWR, OUTPUT);
  digitalWrite(DPIN_PWR, amount_current==0?LOW:HIGH);
}

void
fade_to_amount(byte amount, unsigned short fade_duration) {
  if((amount == 0) && (amount_current < 64) && digitalRead(DPIN_DRV_MODE)) {
    // Automatically fall back to "low" mode on the driver when
    // we are already dim and fading to black. This helps make the transition
    // appear more smooth.
    amount_current *= 4;
    digitalWrite(DPIN_DRV_MODE,LOW);
    analogWrite(DPIN_DRV_EN,amount_current);
  }
  amount_begin = amount_current;
  amount_end = amount;
  amount_fade_duration = fade_duration;
  amount_fade_start = millis();
  amount_off = 0;
}

PT_THREAD(fade_control_pt_func(struct pt *pt))
{
  static long fade_time;

  PT_BEGIN(pt);

  do {
    PT_YIELD(pt);

    if(amount_flash) {
      analogWrite(DPIN_DRV_EN, 0);
      fade_time =  millis();
      PT_WAIT_UNTIL(pt, (millis()-fade_time) > (100));
    }

    amount_flash = 0;

    fade_time = millis() - amount_fade_start;

    if(fade_time >= amount_fade_duration) {
      if(amount_current != amount_end) {
        amount_current = amount_end;

        pinMode(DPIN_PWR, OUTPUT);
        digitalWrite(DPIN_PWR, amount_current==0?LOW:HIGH);
      }
    } else {
      amount_current = (((long)amount_end - (long)amount_begin)*fade_time)/amount_fade_duration + amount_begin;
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, amount_current==0?LOW:HIGH);
    }

    if(amount_current>overtemp_max) {
      amount_current = overtemp_max;
    }

    analogWrite(DPIN_DRV_EN, amount_off?0:amount_current);

    if(!amount_current)
      digitalWrite(DPIN_DRV_MODE, LOW);
  } while(1);

  PT_END(pt);
}

PT_THREAD(power_pt_func(struct pt *pt))
{
  const unsigned long time = millis();
  int chargeState;
  PT_BEGIN(pt);

  do {
    // Check the state of the charge controller
    chargeState = analogRead(APIN_CHARGE);

    if (chargeState < 128) {  // Low - charging
      // Smoothly pulse the green LED over a two-second interval,
      // as if it were "breathing". This is the charging indication.
      byte pulse = ((time>>2)&0xFF);
      pulse = ((pulse * pulse) >> 8);
      analogWrite(DPIN_GLED, ((time>>2)&0x0100)?0xFF-pulse:pulse);

    } else if (chargeState > 768) {  // High - fully charged.
      // Solid green LED.
      analogWrite(DPIN_GLED, 255);
      digitalWrite(DPIN_GLED, HIGH);

    } else {  // Hi-Z - Not charging, not pulged in.
      // Blink the indicator LED now and then.
      digitalWrite(DPIN_GLED, (time&0x03FF)?LOW:HIGH);
    }

#define VOLTAGE_NOMINAL        3400
#define VOLTAGE_LOW            3100

#define OVERTEMP_SHUTDOWN       (long)((long)370*VOLTAGE_NOMINAL/(long)1024)
#define OVERTEMP_THROTTLE       (long)(OVERTEMP_SHUTDOWN-200)

    // Check the temperature sensor
    {
      short voltage = readVcc();
      short temperature = analogRead(APIN_TEMP)*(long)voltage/1024;
      byte pgood = digitalRead(DPIN_PGOOD);
      static bool low_power_condition = false;

      if(temperature > OVERTEMP_SHUTDOWN) {
        if(amount_current)
          Serial.println("Overheat shutdown!");
        set_amount(0);
        digitalWrite(DPIN_DRV_MODE, LOW);
        digitalWrite(DPIN_DRV_EN, LOW);
        digitalWrite(DPIN_PWR, LOW);
      }

      if(low_power_condition || (voltage<VOLTAGE_LOW)) {
        low_power_condition = true;
        if((voltage < VOLTAGE_LOW) && (overtemp_max > 4))
          overtemp_max--;
        if((voltage > VOLTAGE_NOMINAL+50) && (overtemp_max != 255))
          overtemp_max++;
        digitalWrite(DPIN_DRV_MODE,LOW);
      } else {
        overtemp_max = 255;
      }

      if(temperature > OVERTEMP_THROTTLE) {
        overtemp_max = min(overtemp_max,(OVERTEMP_SHUTDOWN - temperature));
      }

      static unsigned long lastStatTime;
      if(time-lastStatTime > 1000) {
        lastStatTime = time;
        Serial.print("stat:");

        if (chargeState < 128) {  // Low - charging
          Serial.print(" Charging");
        } else if (chargeState > 768) {  // High - fully charged.
          Serial.print(" Fully-charged");
        } else {  // Hi-Z - Not charging, not pulged in.
          Serial.print(" Battery");
        }

        Serial.print(" drv-good=");
        Serial.print(pgood?"yes":"no");
        Serial.print(" drv-mode=");
        Serial.print(digitalRead(DPIN_DRV_MODE)?"hi":"low");
        Serial.print(" brght=");
        Serial.print(amount_current);
        Serial.print(" thrt=");
        Serial.print(overtemp_max);
        Serial.print(" Vcc=");
        Serial.print(voltage);
        Serial.print("mv Temp=");
        Serial.println(temperature);
      }
    }

    PT_YIELD(pt);
  } while(1);

  PT_END(pt);
}

PT_THREAD(light_momentary_pt_func(struct pt *pt))
{
  unsigned long time = millis();

  // If more than two minutes go by without the user
  // pressing a button, then go ahead and shut down.
  if(button_released_duration > 120000) {
    pinMode(DPIN_PWR, OUTPUT);
    digitalWrite(DPIN_PWR, LOW);
  }

  PT_BEGIN(pt);

  do {
    pinMode(DPIN_PWR, OUTPUT);
    digitalWrite(DPIN_PWR, HIGH);
    amount_off = 0;

    PT_WAIT_UNTIL(pt, !digitalRead(DPIN_RLED_SW));

    amount_off = 1;

    PT_WAIT_UNTIL(pt, digitalRead(DPIN_RLED_SW));
  } while(amount_current);

  PT_END(pt);

}

PT_THREAD(light_blinky_pt_func(struct pt *pt))
{
  unsigned long time = millis();

  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, !digitalRead(DPIN_RLED_SW) && (button_released_duration > 10));
  button_pressed_duration = 0;

  do {
    amount_off = !((time&(255))<64);
    PT_YIELD(pt);
  } while(amount_current && (button_pressed_duration<10));

  amount_off = 0;
  PT_WAIT_UNTIL(pt,!digitalRead(DPIN_RLED_SW));

  PT_END(pt);
}

PT_THREAD(light_knob_pt_func(struct pt *pt))
{
  unsigned long time = millis();
  static unsigned long lastTime;
  static float lastKnobAngle, knob;

#define PT_WAIT_FOR_PERIOD(pt,x) \
    lastTime =  time; \
    PT_WAIT_UNTIL(pt, (time-lastTime) > (x));

  PT_BEGIN(pt);

  // If we aren't running the driver in 'high' mode,
  // then go ahead and set it to high mode. Also adjust
  // the amount so that it is as close as possible in brightness.
  if(!digitalRead(DPIN_DRV_MODE)) {
    set_amount(amount_current/4);
    digitalWrite(DPIN_DRV_MODE, HIGH);
  }

  // Set the initial knob value based on our current light bightness level.
  knob = sqrt((float)amount_current/255.0)*255.0;

  // Wait for the user to let go of the button.
  PT_WAIT_UNTIL(pt,!digitalRead(DPIN_RLED_SW));

  // Wait for a brief moment for any vibrations to stabalize.
  PT_WAIT_FOR_PERIOD(pt,50);

  // We read this three times to make sure our filtering has stabalized.
  lastKnobAngle = readAccelAngleXZ();
  lastKnobAngle = readAccelAngleXZ();
  lastKnobAngle = readAccelAngleXZ();

  do {
    PT_WAIT_FOR_PERIOD(pt,50);
    float angle = readAccelAngleXZ();
    float change = angle - lastKnobAngle;
    lastKnobAngle = angle;

    // Don't bother updating our brightness reading if our angle isn't good.
    char acc[3];
    readAccel(acc);
    if(acc[0]*acc[0] + acc[2]*acc[2] >= 10*10) {
      if (change >  PI) change -= 2.0*PI;
      if (change < -PI) change += 2.0*PI;
      knob += -change * 40.0;
      if (knob < 0)   knob = 0;
      if (knob > 255) knob = 255;
    }

    // Make apparent brightness changes linear by squaring the
    // value and dividing back down into range.  This gives us
    // a gamma correction of 2.0, which is close enough.
    byte bright = (long)(knob * knob) >> 8;

    // Avoid ever appearing off in this mode!
    if (bright < 4) bright = 4;

    fade_to_amount(bright,50);
  } while(amount_current && (button_pressed_duration<10));

  PT_WAIT_UNTIL(pt,!digitalRead(DPIN_RLED_SW));

  PT_END(pt);
}

PT_THREAD(light_pt_func(struct pt *pt))
{
  unsigned long time = millis();
  static unsigned long lastTime;
  static unsigned long debounceTime;
  static byte level;

  PT_BEGIN(pt);

  // Estimate what the current brighness level is closest to.
  if(digitalRead(DPIN_DRV_MODE)) {
    if(amount_current <= 8) {
      level = 0;
    } else if(amount_current <= 32) {
      level = 1;
    } else if(amount_current <= 128) {
      level = 2;
    } else {
      level = 3;
    }
  } else {
    if(amount_current <= 32) {
      level = 0;
    } else if(amount_current <= 128) {
      level = 1;
    } else {
      level = 2;
    }
  }

  button_released_time = time;
  button_released_duration = 0;

  Serial.println("Starting light thread.");
  do {
    PT_WAIT_UNTIL(pt, digitalRead(DPIN_RLED_SW) && (button_pressed_duration > 20));

    if(!amount_current || (button_released_duration<600)) {
      level++;
      level &= 3;
      Serial.print("intensity=");
      Serial.println(level);
    } else {
      Serial.println("turning off");
      level = 0;
    }

    switch(level) {
    case 0:
      //pinMode(DPIN_PWR, OUTPUT);
      //digitalWrite(DPIN_PWR, LOW);
      //digitalWrite(DPIN_DRV_MODE, LOW);
      //analogWrite(DPIN_DRV_EN, 0);

      //fade_to_amount(0,500);
      break;
    case 1:
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);

      digitalWrite(DPIN_DRV_MODE, LOW);
      fade_to_amount(64,250);
      break;
    case 2:
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);

      digitalWrite(DPIN_DRV_MODE, LOW);
      fade_to_amount(255,500);
      break;
    case 3:
      pinMode(DPIN_PWR, OUTPUT);
      digitalWrite(DPIN_PWR, HIGH);

      if(!digitalRead(DPIN_DRV_MODE)) {
        set_amount(amount_current/4);
        digitalWrite(DPIN_DRV_MODE, HIGH);
      }
      fade_to_amount(255,500);
      break;
    }

    PT_WAIT_UNTIL(pt, !digitalRead(DPIN_RLED_SW) && (button_released_duration > 20));

    if(!level)
      fade_to_amount(0,500);
  } while(1);

  PT_END(pt);
}

void
setup(void)
{
  // We just powered on!  That means either we got plugged 
  // into USB, or (more likely) the user is pressing the 
  // power button.  We need to pull up the enable pin of 
  // the regulator very soon so we don't lose power.

  // We don't pull the pin high here quite yet because
  // we will do that when we transition out of MODE_OFF,
  // somewhere in the main loop. Delaying this as long
  // as possible acts like a debounce for accidental button
  // taps.
  pinMode(DPIN_PWR,      INPUT);
  digitalWrite(DPIN_PWR, LOW);

  // Initialize GPIO
  pinMode(DPIN_RLED_SW,  INPUT);
  pinMode(DPIN_GLED,     OUTPUT);
  pinMode(DPIN_DRV_MODE, OUTPUT);
  pinMode(DPIN_DRV_EN,   OUTPUT);
  pinMode(DPIN_ACC_INT,  INPUT);
  pinMode(DPIN_PGOOD,    INPUT);
  digitalWrite(DPIN_DRV_MODE, LOW);
  digitalWrite(DPIN_DRV_EN,   LOW);
  digitalWrite(DPIN_ACC_INT,  HIGH);
  
  // Initialize serial busses
  Serial.begin(9600);
  Wire.begin();

  // Configure accelerometer
  byte config[] = {
    ACC_REG_INTS,  // First register (see next line)
    0xE4,  // Interrupts: shakes, taps
    0x00,  // Mode: not enabled yet
    0x00,  // Sample rate: 120 Hz
    0x0F,  // Tap threshold
    0x10   // Tap debounce samples
  };
  Wire.beginTransmission(ACC_ADDRESS);
  Wire.write(config, sizeof(config));
  Wire.endTransmission();

  // Enable accelerometer
  byte enable[] = {ACC_REG_MODE, 0x01};  // Mode: active!
  Wire.beginTransmission(ACC_ADDRESS);
  Wire.write(enable, sizeof(enable));
  Wire.endTransmission();
  
//  btnTime = millis();
//  btnDown = digitalRead(DPIN_RLED_SW);

  // We are initially in the "off" state. We will transition
  // to something else in the main loop.
  //mode = MODE_OFF;

  Serial.println("Powered up!");

  button_released_time = millis();
  button_pressed_time = millis();

  int chargeState = analogRead(APIN_CHARGE);

  // Don't bother loading the settings if we are connected to USB.
  // Only load the settings when we are running from the battery.
  if ((chargeState >= 128) && (chargeState <= 768)) {
    retrieve_settings();
  }
}

void
loop(void)
{
  static unsigned long lastTime, lastTempTime, lastAccTime;
  static float lastKnobAngle, knob;
  static byte blink;
  unsigned long time = millis();

  if(digitalRead(DPIN_RLED_SW)) {
    button_released_time = time;
    button_pressed_duration = time - button_pressed_time;
  } else {
    button_pressed_time = time;
    button_released_duration = time - button_released_time;
    button_pressed_duration = 0;
  }


  // Check the serial port
  if(Serial.available()) {
    char c = Serial.read();
    switch(c) {
    case 's':
      {
        int temperature = analogRead(APIN_TEMP);
        Serial.print("Temperature = ");
        Serial.println(temperature);

        char accel[3];
        readAccel(accel);
        Serial.print("Acceleration = ");
        Serial.print(accel[0], DEC);
        Serial.print(", ");
        Serial.print(accel[1], DEC);
        Serial.print(", ");
        Serial.println(accel[2], DEC);
      
        byte pgood = digitalRead(DPIN_PGOOD);
        Serial.print("LED driver power good = ");
        Serial.println(pgood?"Yes":"No");
      }
      break;
    }
  }

  // Check if the accelerometer wants to interrupt
//  byte tapped = 0, shaked = 0;
//  if (!digitalRead(DPIN_ACC_INT)) {
//    Wire.beginTransmission(ACC_ADDRESS);
//    Wire.write(ACC_REG_TILT);
//    Wire.endTransmission(false);       // End, but do not stop!
//    Wire.requestFrom(ACC_ADDRESS, 1);  // This one stops.
//    byte tilt = Wire.read();
//
//    if (time-lastAccTime > 500) {
//      lastAccTime = time;
//
//      tapped = !!(tilt & 0x20);
//      shaked = !!(tilt & 0x80);
//
//      if (tapped) Serial.println("Tap!");
//      if (shaked) Serial.println("Shake!");
//    }
//  }

  power_pt_func(&power_pt);
  fade_control_pt_func(&fade_control_pt);

#define NUMBER_OF_MODES    (4)

  static byte last_mode;
  static byte save_mode;
  if((button_pressed_duration > 2048+1024+512*NUMBER_OF_MODES+1024)) {
    if(save_mode == 1) {
      amount_off = 1;
      save_settings();
      amount_off = 0;
      save_mode = 2;
      //fade_to_amount(0,500);
    }
  } else if((button_pressed_duration > 2048+1024+512*NUMBER_OF_MODES)) {
    amount_off = ((button_pressed_duration/64) & 1);
    if(save_mode == 0) {
      save_mode = 1;
      last_mode = 0;
    }
  } else if((button_pressed_duration > 2048)) {
    byte selected_mode = (button_pressed_duration-2048)/512 + 1;
    amount_off = 0;
    if(selected_mode-1>=light_mode) {
      selected_mode++;
    }
    if(selected_mode > NUMBER_OF_MODES)
      selected_mode = NUMBER_OF_MODES;
    if(selected_mode-1==light_mode)
      selected_mode--;
    if(selected_mode != last_mode) {
      amount_flash = 1;
      last_mode = selected_mode;
      Serial.print("Mode selected: ");
      Serial.println(selected_mode-1);
    }
  } else {
    char thread_status;
    if(save_mode) {
      save_mode = 0;
      amount_off = 0;
      button_pressed_duration = 0;
      button_released_duration = 0;
    }
    if(last_mode) {
      light_mode = last_mode-1;
      last_mode = 0;
      Serial.print("Mode change: ");
      Serial.println(light_mode);
      PT_INIT(&light_pt);
      PT_INIT(&light_momentary_pt);
      PT_INIT(&light_blinky_pt);
      PT_INIT(&light_knob_pt);
    }
    switch(light_mode) {
      default:
      case 0: thread_status = light_pt_func(&light_pt); break;
      case 1: thread_status = light_momentary_pt_func(&light_momentary_pt); break;
      case 2: thread_status = light_blinky_pt_func(&light_blinky_pt); break;
      case 3: thread_status = light_knob_pt_func(&light_knob_pt); break;
    }
    if(!PT_SCHEDULE(thread_status)) {
      light_mode = 0;
      fade_to_amount(0,500);
    }
  }

  return;
}

void readAccel(char *acc)
{
  while (1)
  {
    Wire.beginTransmission(ACC_ADDRESS);
    Wire.write(ACC_REG_XOUT);
    Wire.endTransmission(false);       // End, but do not stop!
    Wire.requestFrom(ACC_ADDRESS, 3);  // This one stops.

    for (int i = 0; i < 3; i++)
    {
      if (!Wire.available())
        continue;
      acc[i] = Wire.read();
      if (acc[i] & 0x40)  // Indicates failed read; redo!
        continue;
      if (acc[i] & 0x20)  // Sign-extend
        acc[i] |= 0xC0;
    }
    break;
  }
}

/* Returns the median value of the given three parameters */
char median_char(char a, char b, char c) {
    if(a<c) {
        if(b<a) {
            return a;
        } else if(c<b) {
            return c;
        }
    } else {
        if(a<b) {
            return a;
        } else if(b<c) {
            return c;
        }
    }
    return b;
}


float readAccelAngleXZ()
{
  static char acc[3][3];
  static char i;
  readAccel(acc[i++]);
  if(i==3)
    i=0;
  return atan2(median_char(acc[0][0],acc[1][0],acc[2][0]),median_char(acc[0][2],acc[1][2],acc[2][2]));
}

