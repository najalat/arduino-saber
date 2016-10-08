
#ifndef SABER_CONFIG_H
#define SABER_CONFIG_H

// active color channels, in range 0-4
#define CHANNELS 2

// number of color options
#define COLORS 2

// default color (saber on)
const uint8_t LED_DEFAULT[COLORS][CHANNELS] = {
  { 100, 0 },
  { 0, 100 },
};

// flash on clash color
const uint8_t LED_CLASH[COLORS][CHANNELS] = {
  { 0, 100 },
  { 0, 75 },
};

// flash on blaster color, if different from clash
const uint8_t LED_BLASTER[COLORS][CHANNELS] = {
  { 0, 100 },
  { 0, 85 },
};

// delay times in ms
#define HOLD_TIME 2000
#define MIN_START_TIME 1000
#define MIN_CLASH_TIME 300
#define MIN_SWING_TIME 700

// minimum acceleration for a clash/swing event in units of g
#define CLASH_ACC 5.0
#define SWING_ACC 3.0

// speed to fade to the default on/off color
#define FADE_ON 16
#define FADE_OFF 5

// sample rate of sound effects
#define SAMPLE_RATE 16000

#define BOARD_VOLTAGE 5.0

#endif
