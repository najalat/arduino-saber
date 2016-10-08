
#include <SD.h>
#include <LowPower.h>
#include "SaberConfig.h"
#include "pins_arduino.h"

#define ACCELEROMETER_ACTIVE
#define DEBUG

#define LED_PIN_COUNT 4
const uint8_t LED_PINS[LED_PIN_COUNT] { 9, 6, 5, 10 };

#define SW_PIN 2
#define X_PIN A3
#define Y_PIN A2
#define Z_PIN A1
#define GND_PIN 14
#define SD_CS_PIN 4
#define SPEAKER_PIN 3

#define SWING_SOUND 0
#define CLASH_SOUND 1
#define BLASTER_SOUND 2
#define SOUND_TYPE_COUNT 3

// Ligtsaber states
#define SABER_OFF 0
#define SABER_ON 1
#define SETTINGS_VOLUME 2
#define SETTINGS_FONT 3
#define SETTINGS_COLOR 4

#define TMAX 240

struct {
  uint8_t volume;
  uint8_t font;
  uint8_t color;
} settings;

uint8_t font_count = 0;
uint8_t sound_counts[SOUND_TYPE_COUNT];
char filename[25];

uint8_t saber_state = SABER_OFF;

const uint8_t LED_OFF[CHANNELS] = { 0 };

// At transition = TMAX, led pins are set to default.
// At transition = 0, led pins are set to led_alt.
uint8_t transition = 0;
const uint8_t *led_def;
const uint8_t *led_alt = LED_OFF;

// time in millis of the upcoming button hold event
unsigned long hold_time = 0;

// no swing event can occur if millis() < swing_time
unsigned long swing_time = 0;

// no clash OR swing event can occur if millis() < clash_time
unsigned long clash_time = 0;

#define SD_FOUND_BIT 0
#define BUTTON_STATE_BIT 1
#define HOLD_WAITING_BIT 2
#define NEXT_BUTTON_STATE_BIT 3
byte bitmap = 0;

/*** AUDIO ***/

#define BUFFER_DONE 0
#define BUFFER_FILE 1
#define BUFFER_IDLE 2
#define BUFFER_RAMP 3

#ifdef DEBUG
// use a smaller buffer in debug mode to save memory
const int BUFFER_BITS = 8;
#else
const int BUFFER_BITS = 9;
#endif
const int BUFFER_SIZE = 1 << BUFFER_BITS;
const int BUFFER_EXTEND = BUFFER_SIZE / 2;
const int BUFFER_MASK = BUFFER_SIZE - 1;

File sFile;
File lFile;
byte lFirstSample;
byte buffer[BUFFER_SIZE];
volatile uint16_t buffer_front = 0;
volatile uint16_t buffer_back = 0;
volatile uint8_t buffer_state = BUFFER_DONE;
volatile bool buffering = false;

#define buffer_add(c) (buffer_back = (buffer_back + c) & BUFFER_MASK)

ISR(TIMER1_CAPT_vect) {
  if (buffer_front != buffer_back) {
    OCR2B = buffer[buffer_front] >> (2 - settings.volume);
    buffer_front = (buffer_front + 1) & BUFFER_MASK;
  } else if (!buffer_state) {
    #ifdef DEBUG
    Serial.println("audio off");
    #endif
    TIMSK1 &= ~_BV(ICIE1);
    return;
  }

  if (buffering || !buffer_state ||
      bitRead(buffer_front - 1, BUFFER_BITS - 1) 
        == bitRead(buffer_back, BUFFER_BITS - 1))
    return;

  // Note that we are still buffering, but allow nested interrupts.
  buffering = true;
  interrupts();

  // Fill half of the buffer. This assumes sFile and lFile are both
  // open if necessary, and lFile is long enough to fill the buffer.
  uint16_t target = (buffer_back + BUFFER_EXTEND) & BUFFER_MASK;
  switch (buffer_state) {
    case BUFFER_FILE:
      buffer_add(sFile.read(buffer + buffer_back, BUFFER_EXTEND));
      if (buffer_back == target) {
        buffering = false;
        return;
      }
      sFile.close();
      break;
    case BUFFER_IDLE:
      buffer_add(lFile.read(buffer + buffer_back, BUFFER_EXTEND));
      if (buffer_back == target) {
        buffering = false;
        return;
      }
      break;
  }
  // Ramp down to either zero or to the first loop sample.
  byte sample = buffer[(buffer_back - 1) & BUFFER_MASK];
  byte rampTarget = saber_state == SABER_ON ? lFirstSample : 0;
  byte dir = rampTarget > sample ? 1 : -1;
  while (buffer_back != target && sample != rampTarget) {
    sample += dir;
    buffer[buffer_back] = sample;
    buffer_add(1);
  }
  if (buffer_back != target) {
    if (saber_state == SABER_ON) {
      // Fill in remaining space with the loop file.
      lFile.seek(0);
      buffer_add(lFile.read(
        buffer + buffer_back, (target-buffer_back) & BUFFER_MASK));
      buffer_state = BUFFER_IDLE;
    } else {
      // Fill in remaining space with zeros.
      do {
        buffer[buffer_back] = 0;
        buffer_add(1);
      } while (buffer_back != target);
      buffer_state = BUFFER_DONE;
    }
  } else buffer_state = BUFFER_RAMP;
  buffering = false;
}

void beep() {
  tone(SPEAKER_PIN, 110, 300);
}

const char *sound_prefix(uint8_t type) {
  switch (type) {
    case SWING_SOUND: return "swing";
    case CLASH_SOUND: return "clash";
    case BLASTER_SOUND: return "blast";
  }
}

inline void set_filename(const char *f) {
  sprintf(filename, "/font%d/%s.pcm", settings.font, f);
}

void play_current() {
  #ifdef DEBUG
  Serial.println(filename);
  #endif

  buffering = true;
  if (sFile) sFile.close();
  sFile = SD.open(filename);
  buffer_state = BUFFER_FILE;
  buffering = false;
  TIMSK1 |= _BV(ICIE1);
}

void play_sound(const char *file) {
  if (settings.volume) {
    set_filename(file);
    play_current();
  }
}

// Choose one of the available sounds at random and play it.
void play_one(uint8_t type) {
  int count = sound_counts[type];
  if (settings.volume && count) {
    sprintf(filename, "/font%d/%s%d.pcm", settings.font,
      sound_prefix(type), rand() % count + 1);
    play_current();
  }
}

void play_menu(const char *file) {
  sprintf(filename, "/menu/%s.pcm", file);
  play_current();
}

void save_settings() {
  if (buffer_state) {
    buffer_state = BUFFER_RAMP;
    sFile.close();
  }
  SD.remove("settings");
  File f = SD.open("settings", FILE_WRITE);
  if (f) {
    f.write((byte*)&settings, sizeof(settings));
    f.close();
  }
  #ifdef DEBUG
  else Serial.println("failed to save settings");
  #endif
}

bool starts_with(const char *prefix, const char *str) {
  while (*prefix) {
    if (*prefix != tolower(*str))
      return false;
    ++prefix;
    ++str;
  }
  return true;
}

bool check_sd() {
  font_count = 0;
  if (SD.begin(SD_CS_PIN)) {
    File root = SD.open("/");
    root.rewindDirectory();
    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;
      if (starts_with("font", entry.name()))
        ++font_count;
      entry.close();
    }
    root.close();
    #ifdef DEBUG
    Serial.print("font count: "); Serial.println(font_count);
    #endif
  }
  return font_count > 0;
}

void count_sound_file(const char *file) {
  for (int i = 0; i < SOUND_TYPE_COUNT; ++i)
    if (starts_with(sound_prefix(i), file)) {
      ++sound_counts[i];
      return;
    }
}

void set_font() {
  for (int i = 0; i < SOUND_TYPE_COUNT; ++i)
    sound_counts[i] = 0;
  sprintf(filename, "/font%d", settings.font);
  File dir = SD.open(filename);
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) break;
    count_sound_file(entry.name());
    entry.close();
  }
  dir.close();
  if (lFile) lFile.close();
  set_filename("idle");
  lFile = SD.open(filename);
  lFirstSample = lFile.read();
  #ifdef DEBUG
  Serial.print("loaded font "); Serial.println(settings.font);
  for (int i = 0; i < SOUND_TYPE_COUNT; ++i) {
    Serial.print(" "); Serial.print(sound_prefix(i));
    Serial.print(":"); Serial.print(sound_counts[i]);
  }
  Serial.println();
  Serial.flush();
  #endif
}

void set_timers() {
  TCCR2A = _BV(WGM21) | _BV(WGM20) | _BV(COM2B1);
  TCCR2B = _BV(CS20);

  noInterrupts();
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = F_CPU / SAMPLE_RATE;
  interrupts();
}

void on_switch() {
  bitWrite(bitmap, NEXT_BUTTON_STATE_BIT, digitalRead(SW_PIN));
}

void setup() {
  for (int i = 0; i < LED_PIN_COUNT; ++i) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }
  pinMode(GND_PIN, OUTPUT);
  digitalWrite(GND_PIN, LOW);
  pinMode(SW_PIN, INPUT);
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(Z_PIN, INPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(SW_PIN), on_switch, CHANGE);
  
  #ifdef DEBUG
  Serial.begin(9600);
  Serial.println("sup");
  #endif

  // default settings
  settings.volume = 1;
  settings.font = 1;
  settings.color = 0;

  if (check_sd()) {
    bitSet(bitmap, SD_FOUND_BIT);
    
    File f = SD.open("settings");
    if (f) {
      f.read((byte*)&settings, sizeof(settings));
      f.close();
      if (settings.font > font_count) {
        // some fonts may have been deleted, reset to 1
        settings.font = 1;
      }
      #ifdef DEBUG
      Serial.print("settings: ");
      Serial.print(settings.volume);
      Serial.print(",");
      Serial.print(settings.font);
      Serial.print(",");
      Serial.println(settings.color);
      #endif
    } else {
      #ifdef DEBUG
      Serial.println("saving default settings");
      #endif
      save_settings();
    }
    set_font();
    set_timers();
    set_filename("boot");
    play_current();
  } else {
    bitClear(bitmap, SD_FOUND_BIT);
    settings.volume = 0;
    beep();
  }
  led_def = LED_DEFAULT[settings.color];

  #ifdef COLOR_TEST
  saber_state = SETTINGS_COLOR;
  #endif
}

void write_pwm(uint8_t pin, uint32_t val) {
  if (val == 0) digitalWrite(pin, LOW);
  else if (val == 100) digitalWrite(pin, HIGH);
  else switch (digitalPinToTimer(pin)) {
    case TIMER0A:
      TCCR0A |= _BV(COM0A1);
      OCR0A = val * 255 / 100;
      break;
    case TIMER0B:
      TCCR0A |= _BV(COM0B1);
      OCR0B = val * 255 / 100;
      break;
    case TIMER1A:
      TCCR1A |= _BV(COM1A1);
      OCR1A = val * ICR1 / 100;
      break;
    case TIMER1B:
      TCCR1A |= _BV(COM1B1);
      OCR1B = val * ICR1 / 100;
      break;
  }
}

void set_transition(uint16_t t) {
  transition = t;
  uint8_t brightness[CHANNELS];
  for (int i = 0; i < CHANNELS; ++i)
    brightness[i] = led_def[i]*t/TMAX + led_alt[i]*(TMAX-t)/TMAX;
  noInterrupts(); // disable interrupts while setting LEDs
  for (int i = 0; i < CHANNELS; ++i)
    write_pwm(LED_PINS[i], brightness[i]);
  interrupts();
}

inline void play_menu_volume() {
  play_menu("sndon");
}

void onButtonPush(unsigned long ms) {
  #ifdef DEBUG
  Serial.println("push");
  #endif
  switch (saber_state) {
    case SABER_OFF:
      saber_state = SABER_ON;
      led_alt = LED_OFF;
      set_transition(0);
      play_sound("on");
      clash_time = ms + MIN_START_TIME;
      break;
    case SABER_ON:
      led_alt = LED_BLASTER[settings.color];
      set_transition(0);
      play_one(BLASTER_SOUND);
      clash_time = ms + MIN_CLASH_TIME;
      break;
    case SETTINGS_VOLUME:
      settings.volume = (settings.volume + 1) % 3;
      play_menu_volume();
      break;
    case SETTINGS_FONT:
      settings.font = settings.font % font_count + 1;
      play_sound("boot");
      break;
    case SETTINGS_COLOR:
      led_alt = led_def;
      settings.color = (settings.color + 1) % COLORS;
      led_def = LED_DEFAULT[settings.color];
      set_transition(0);
      break;
  }
}

void onButtonHold() {
  #ifdef DEBUG
  Serial.println("hold");
  #endif
  switch (saber_state) {
    case SABER_ON:
      saber_state = SABER_OFF;
      led_alt = LED_OFF;
      set_transition(TMAX);
      play_sound("off");
      break;
    case SABER_OFF:
      if (bitRead(bitmap, SD_FOUND_BIT)) {
        saber_state = SETTINGS_VOLUME;
        play_menu_volume();
        break;
      }
      // no sd card, don't bother setting volume
      // fall through
    case SETTINGS_VOLUME:
      if (settings.volume) {
        saber_state = SETTINGS_FONT;
        play_sound("boot");
        break;
      }
      // muted, don't bother setting sound font
      // fall through
    case SETTINGS_FONT:
      if (COLORS > 1) {
        saber_state = SETTINGS_COLOR;
        set_transition(TMAX);
        break;
      }
      // only one color, don't bother changing
      // fall through
    case SETTINGS_COLOR:
      saber_state = SABER_OFF;
      led_alt = LED_OFF;
      set_transition(0);
      if (bitRead(bitmap, SD_FOUND_BIT)) {
        save_settings();
        set_font();
        play_menu("done");
      } else {
        // no sd card, can't save
        beep();
      }
      break;
  }
}

#define CLASH_EVENT 1
#define SWING_EVENT 2

double readAcceleration(uint8_t pin, double zeroRange) {
  double v = analogRead(pin) * BOARD_VOLTAGE / 1023.0;
  return max(0, abs(v - 1.5) - zeroRange) / 0.3;
}

uint8_t accelerationEvent(unsigned long ms) {
  #ifdef ACCELEROMETER_ACTIVE
  if (ms >= clash_time) {
    // Calculate acceleration in units of g^2
    double x = readAcceleration(X_PIN, 0.15);
    double y = readAcceleration(Y_PIN, 0.15);
    double z = readAcceleration(Z_PIN, 0.3);
    double a = x*x + y*y + z*z;
    if (a > CLASH_ACC*CLASH_ACC) {
      #ifdef DEBUG
      Serial.print("clash: ");
      Serial.print(x); Serial.print(", ");
      Serial.print(y); Serial.print(", ");
      Serial.print(z); Serial.print(" = ");
      Serial.println(a);
      #endif
      clash_time = ms + MIN_CLASH_TIME;
      return CLASH_EVENT;
    
    } else if (ms >= swing_time && a > SWING_ACC*SWING_ACC) {
      #ifdef DEBUG
      Serial.print("swing: ");
      Serial.print(x); Serial.print(", ");
      Serial.print(y); Serial.print(", ");
      Serial.print(z); Serial.print(" = ");
      Serial.println(a);
      #endif
      swing_time = ms + MIN_SWING_TIME;
      return SWING_EVENT;
    }
  }
  #endif
  return 0;
}

void loop() {
  unsigned long ms = millis();
  switch (saber_state) {
    case SABER_OFF:
      if (transition > 0)
        set_transition(transition - FADE_OFF);
      break;
    
    case SABER_ON:
      if (transition < TMAX)
        set_transition(transition + FADE_ON);
      switch (accelerationEvent(ms)) {
        case CLASH_EVENT:
          led_alt = LED_CLASH[settings.color];
          set_transition(0);
          play_one(CLASH_SOUND);
          break;
        
        case SWING_EVENT:
          play_one(SWING_SOUND);
          break;
      }
      break;

    case SETTINGS_COLOR:
      if (transition < TMAX)
        set_transition(transition + FADE_ON);
      if (accelerationEvent(ms) == CLASH_EVENT) {
        led_alt = LED_CLASH[settings.color];
        set_transition(0);
      }
      break;
  }

  int button_state = bitRead(bitmap, NEXT_BUTTON_STATE_BIT);
  if (button_state != bitRead(bitmap, BUTTON_STATE_BIT)) {
    if (button_state) {
      bitSet(bitmap, BUTTON_STATE_BIT);
      bitSet(bitmap, HOLD_WAITING_BIT);
      hold_time = ms + HOLD_TIME;
    } else {
      bitClear(bitmap, BUTTON_STATE_BIT);
      if (bitRead(bitmap, HOLD_WAITING_BIT)) {
        bitClear(bitmap, HOLD_WAITING_BIT);
        onButtonPush(ms);
      }
    }
    
  } else if (bitRead(bitmap, HOLD_WAITING_BIT) && ms >= hold_time) {
    bitClear(bitmap, HOLD_WAITING_BIT);
    onButtonHold();
  }

  // If the saber is turned off, the button is released,
  // and there is no sound playing, go into power saving mode.
  // This will wake up on interrupt when the button is pushed.
  if (saber_state == SABER_OFF && !button_state
      && !bitRead(TIMSK1, ICIE1)) {
    #ifdef DEBUG
    Serial.println("good night");
    Serial.flush();
    #endif
    
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    
    #ifdef DEBUG
    Serial.println("good morning");
    Serial.flush();
    #endif
  }

  while (millis() < ms + 30);
}

