// Trellis M4 MIDI Keypad CC
// sends 32 notes, pitch bend & a CC from accelerometer tilt over USB MIDI

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL343.h>
#include <Adafruit_NeoTrellisM4.h>
#include "Ticker.h"
#include <config.h>
#include <colors.h>

#define MIDI_CHANNEL 0 // default channel # is 0
void check_mode();

// Set the value of first note, C is a good choice. Lowest C is 0.
// 36 is a good default. 48 is a high range. Set to 24 for a bass machine.

Adafruit_ADXL343 accel = Adafruit_ADXL343(123, &Wire1);
Adafruit_NeoTrellisM4 trellis = Adafruit_NeoTrellisM4();
Config config = new_config();

int last_xbend = 0;
int last_ybend = 0;
bool pressed[32];
bool last_mode_change = false;
uint8_t mode = 0;
Ticker mode_tick(check_mode, 4000, MILLIS);

void next_mode()
{
  fill(&trellis, BLACK);
  delay(100);
  switch (mode)
  {
  case 0:
    mode = 1;
    note_animation(&trellis, TURQUOISE);
    break;
  case 1:
    mode = 0;
    config_animation(&trellis, STEEL);
    break;
  }
  delay(300);
  fill(&trellis, BLACK);
}

void check_mode()
{
  bool all_four = pressed[0] && pressed[16] && pressed[24] && pressed[32];
  if (last_mode_change && all_four)
  {
    next_mode();
  }
  last_mode_change = all_four;
}

// floating point map
float ofMap(float value, float inputMin,
            float inputMax, float outputMin,
            float outputMax, bool clamp)
{

  float outVal = ((value - inputMin) / (inputMax - inputMin) * (outputMax - outputMin) + outputMin);

  if (!clamp)
  {
    return outVal;
  }

  if (outputMax < outputMin)
  {
    if (outVal < outputMax)
    {
      return outputMax;
    }
    if (outVal > outputMin)
    {
      return outputMin;
    }
  }
  else
  {
    if (outVal > outputMax)
    {
      return outputMax;
    }
    if (outVal < outputMin)
    {
      return outputMin;
    }
  }

  return outVal;
}

void setup()
{
  Serial.begin(115200);
  //while (!Serial);
  Serial.println("MIDI keypad & pitchbend!");

  trellis.begin();
  trellis.setBrightness(80);

  // USB MIDI messages sent over the micro B USB port
  Serial.println("Enabling MIDI on USB");
  trellis.enableUSBMIDI(true);
  trellis.setUSBMIDIchannel(MIDI_CHANNEL);
  // UART MIDI messages sent over the 4-pin STEMMA connector (3.3V logic)
  Serial.println("Enabling MIDI on UART");
  trellis.enableUARTMIDI(true);
  trellis.setUARTMIDIchannel(MIDI_CHANNEL);
  if (!accel.begin())
  {
    Serial.println("No accelerometer found");
    while (1)
      ;
  }
  mode_tick.start();
}

void trellis_config_event()
{
  while (trellis.available())
  {
    keypadEvent e = trellis.read();
    uint8_t update = update_config(&config, e.bit.KEY);
    switch (update)
    {
      case 0:
        Serial.print("Invalid config: ");
        Serial.println(e.bit.KEY);
        trellis.noteOn(5, 64);
        trellis.noteOn(6, 64);
        trellis.sendMIDI();
        trellis.noteOff(5, 64);
        trellis.noteOff(6, 64);
        trellis.sendMIDI();
        fill(&trellis, RED);
        delay(300);
        fill(&trellis, BLACK);
        break;
      case 1:
        twenty_four_animation(&trellis, TURQUOISE);
        break;
      case 2:
        thirty_six_animation(&trellis, TURQUOISE);
        break;
    }
  }
}

void trellis_sound_event()
{
  while (trellis.available())
  {
    keypadEvent e = trellis.read();
    int key = e.bit.KEY;
    Serial.print("Keypad key: ");
    Serial.println(key);
    Serial.print("MIDI note: ");
    Serial.println(config.first_midi_note + key);

    if (e.bit.EVENT == KEY_JUST_PRESSED)
    {
      pressed[e.bit.KEY] = true;
      Serial.println(" pressed\n");
      trellis.setPixelColor(key, 0xFFFFFF);
      trellis.noteOn(config.first_midi_note + key, 64);
    }
    else if (e.bit.EVENT == KEY_JUST_RELEASED)
    {
      pressed[e.bit.KEY] = false;
      Serial.println(" released\n");
      trellis.setPixelColor(key, 0x0);
      trellis.noteOff(config.first_midi_note + key, 64);
    }
  }
}

void accel_y_tick(float y)
{
  float y_bend = 0;
  if (abs(y) < 2.0)
  { // 2.0 m/s^2
    // don't make any bend unless they've really started moving it
    y_bend = 8192; // 8192 means no bend
  }
  else
  {
    if (y > 0)
    {
      y_bend = ofMap(y, 2.0, 10.0, 8192, 0, true); // 2 ~ 10 m/s^2 is downward bend
    }
    else
    {
      y_bend = ofMap(y, -2.0, -10.0, 8192, 16383, true); // -2 ~ -10 m/s^2 is upward bend
    }
  }
  if (y_bend != last_ybend)
  {
    Serial.print("Y pitchbend: ");
    Serial.println(y_bend);
    trellis.pitchBend(y_bend);
    last_ybend = y_bend;
  }
}

void accel_x_tick(float x)
{
  float x_bend = 0;
  if (abs(x) < 2.0)
  { // 2.0 m/s^2
    // don't make any bend unless they've really started moving it
    x_bend = 0;
  }
  else
  {
    if (x > 0)
    {
      x_bend = ofMap(x, 2.0, 10.0, 0, 127, true); // 2 ~ 10 m/s^2 is upward bend
    }
    else
    {
      x_bend = ofMap(x, -2.0, -10.0, 0, 127, true); // -2 ~ -10 m/s^2 is downward bend
    }
  }
  if (x_bend != last_xbend)
  {
    Serial.print("X mod: ");
    Serial.println(x_bend);
    trellis.controlChange(config.cc, x_bend); //xCC is set at top of sketch. e.g., CC 1 is Mod Wheel
    last_xbend = x_bend;
  }
}

void accel_tick()
{
  sensors_event_t event;
  accel.getEvent(&event);
  accel_y_tick(event.acceleration.y);
  accel_x_tick(event.acceleration.x);
}

void mode_zero_loop()
{
  trellis_config_event();
}

void mode_one_loop()
{
  trellis_sound_event();
  accel_tick();
  trellis.sendMIDI();
}

void loop()
{
  trellis.tick();
  mode_tick.update();
  switch (mode)
  {
  case 0:
    mode_zero_loop();
    break;
  case 1:
    mode_one_loop();
    break;
  }
  delay(10);
}
