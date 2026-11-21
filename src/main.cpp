/*
  Arduino Uno Signal Generator  -  replaces the function generator in Lab 12, Part V
  --------------------------------------------------------------------------------
  The Uno has no DAC, so this uses the same trick as Part VIII of the lab:
  a fast PWM signal on D9 is smoothed by an RC low-pass filter into an analog voltage.

  Default output (exactly the Part V settings):  sine, 10 Hz, 2 Vpp, offset 1.5 V
                                                 -> the signal swings 0.5 V to 2.5 V

  WIRING
                    R1 = 1k         R2 = 1k
    Uno D9 ---------/\/\/\---+-----/\/\/\---+------------ OUTPUT (the "red lead")
                             |               |
                           C1 = 1uF        C2 = 1uF
                             |               |
    Uno GND -----------------+---------------+------------ OUTPUT GND (the "black lead")

    Then, as in the lab:  OUTPUT --> 1k resistor --> ESP32 GPIO 32
                          Uno GND --> ESP32 GND   (common ground!)

    Optional push button: D2 --- button --- GND   (toggles the output on / off)
    Built-in LED (D13) is ON when the output is ON.

  SERIAL COMMANDS (115200 baud, end each command with Enter) - optional:
    f10     frequency in Hz (0.1 - 100)        a2      amplitude in Vpp
    o1.5    DC offset in V                     ws/wq/wt/wr   sine / square / triangle / ramp
    on/off  output on / off                    v4.95   your measured Uno 5 V supply (calibration)
    ?       show the help

  SAFETY: the output is limited to 0 - 3.0 V in software, so it can never exceed the
  3.3 V limit of an ESP32 pin. Always keep the 1k series resistor in front of GPIO 32.
*/

#include <Arduino.h>

// ======================= SETTINGS (defaults = Lab 12 Part V) =======================
float freqHz   = 10.0;    // frequency (Hz)
float ampVpp   = 2.0;     // peak-to-peak amplitude (V)
float offsetV  = 1.5;     // DC offset = centre of the wave (V)
char  waveform = 's';     // s = sine, q = square, t = triangle, r = ramp (sawtooth)
float vccV     = 4.916;    // measure the Uno "5V" pin with a multimeter and enter it here
bool  outputOn = true;   // false = output OFF at power-up (press the button or type "on")

const float MAX_OUT_V = 3.0;   // never output more than this (ESP32 pins: 3.3 V max)
const float MIN_FREQ  = 0.1;
const float MAX_FREQ  = 100.0;

// ================================ PINS ================================
const uint8_t PWM_PIN    = 9;     // OC1A (Timer1) - fixed by the hardware
const uint8_t BUTTON_PIN = 2;     // optional button to GND
const uint8_t LED_PIN    = 13;    // built-in LED

// ============================ INTERNAL VALUES ============================
// Timer1: 10-bit fast PWM, 16 MHz / 1024 = 15.625 kHz  (duty = (OCR1A + 1) / 1024)
// Timer2: interrupt at 10 000 Hz updates the PWM duty to build the wave (DDS method)
const float UPDATE_RATE = 10000.0;

int16_t sineTable[256];            // one sine cycle, -32767 ... +32767

volatile uint32_t phase     = 0;   // position in the wave (32-bit accumulator)
volatile uint32_t phaseInc  = 0;   // how far the phase moves every update
volatile int16_t  ampCnt    = 0;   // half of Vpp, in PWM counts
volatile int16_t  offsetCnt = 0;   // centre of the wave, in PWM counts
volatile char     waveSel   = 's';

// Runs 10 000 times per second
ISR(TIMER2_COMPA_vect)
{
  phase += phaseInc;
  uint8_t idx = phase >> 24;                      // 0 - 255: position inside one cycle
  int16_t w;                                      // wave value -32767 ... +32767

  switch (waveSel)
  {
    case 'q':  w = (idx < 128) ? 32767 : -32767;                      break;   // square
    case 't':  {                                                               // triangle
                 uint8_t t = (idx < 128) ? idx : 255 - idx;
                 w = (int16_t)(t * 516L - 32767L);
               }                                                      break;
    case 'r':  w = (int16_t)(((uint16_t)idx << 8) - 32768L);          break;   // ramp
    default:   w = sineTable[idx];                                             // sine
  }

  int16_t duty = offsetCnt + (int16_t)(((int32_t)w * ampCnt) >> 15);
  if (duty < 0) duty = 0;
  OCR1A = duty;
}

void printSettings();
void printHelp();

// Turn the PWM output on or off. Off = pin held at 0 V.
void setOutput(bool on)
{
  outputOn = on;
  if (on) {
    TCCR1A |= _BV(COM1A1);                        // connect Timer1 to pin D9
  } else {
    TCCR1A &= ~_BV(COM1A1);                       // disconnect it ...
    digitalWrite(PWM_PIN, LOW);                   // ... and hold the pin at 0 V
  }
  digitalWrite(LED_PIN, on);
}

// Check the settings, keep the wave inside 0 - MAX_OUT_V, and convert volts to PWM counts
void applySettings()
{
  freqHz  = constrain(freqHz, MIN_FREQ, MAX_FREQ);
  offsetV = constrain(offsetV, 0.0, MAX_OUT_V);
  ampVpp  = max(ampVpp, 0.0f);

  float room = min(offsetV, MAX_OUT_V - offsetV);       // distance to the nearest limit
  if (ampVpp / 2.0 > room) {
    ampVpp = 2.0 * room;
    Serial.println(F("Amplitude reduced to keep the output between 0 V and 3.0 V."));
  }

  float countsPerVolt = 1024.0 / vccV;
  int16_t a   = (int16_t)(ampVpp / 2.0 * countsPerVolt + 0.5);
  int16_t o   = (int16_t)(offsetV * countsPerVolt + 0.5) - 1;     // -1: duty = OCR1A + 1
  uint32_t pi = (uint32_t)(freqHz * (4294967296.0 / UPDATE_RATE) + 0.5);

  noInterrupts();                                        // change all values together
  ampCnt = a;  offsetCnt = o;  phaseInc = pi;  waveSel = waveform;
  interrupts();
}

void printSettings()
{
  const __FlashStringHelper *names[] = { F("sine"), F("square"), F("triangle"), F("ramp") };
  uint8_t n = (waveform == 's') ? 0 : (waveform == 'q') ? 1 : (waveform == 't') ? 2 : 3;
  Serial.print(F("Output "));         Serial.print(outputOn ? F("ON  ") : F("OFF "));
  Serial.print(names[n]);             Serial.print(F(", "));
  Serial.print(freqHz, 2);            Serial.print(F(" Hz, "));
  Serial.print(ampVpp, 2);            Serial.print(F(" Vpp, offset "));
  Serial.print(offsetV, 2);           Serial.print(F(" V  ->  "));
  Serial.print(offsetV - ampVpp / 2, 2);  Serial.print(F(" V to "));
  Serial.print(offsetV + ampVpp / 2, 2);  Serial.println(F(" V"));
}

void printHelp()
{
  Serial.println(F("f<Hz>  a<Vpp>  o<V>  ws|wq|wt|wr  on|off  v<Vcc>  ?"));
}

void handleCommand(char *cmd)
{
  char  c = tolower(cmd[0]);
  float v = atof(cmd + 1);

  if      (strcasecmp(cmd, "on")  == 0) outputOn = true;
  else if (strcasecmp(cmd, "off") == 0) outputOn = false;
  else if (c == 'f') freqHz  = v;
  else if (c == 'a') ampVpp  = v;
  else if (c == 'o') offsetV = v;
  else if (c == 'v') vccV    = constrain(v, 4.0, 5.5);
  else if (c == 'w') {
    char w = tolower(cmd[1]);
    if (w == 's' || w == 'q' || w == 't' || w == 'r') waveform = w;
  }
  else { printHelp(); return; }

  applySettings();
  setOutput(outputOn);
  printSettings();
}

void readSerial()
{
  static char buf[16];
  static uint8_t n = 0;
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { buf[n] = 0; handleCommand(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

void checkButton()
{
  static bool lastState = HIGH;
  static unsigned long lastChange = 0;
  bool state = digitalRead(BUTTON_PIN);
  if (state != lastState && millis() - lastChange > 50) {      // 50 ms debounce
    lastChange = millis();
    lastState = state;
    if (state == LOW) { setOutput(!outputOn); printSettings(); }
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(PWM_PIN, OUTPUT);   digitalWrite(PWM_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  for (int i = 0; i < 256; i++)                                // build the sine table
    sineTable[i] = (int16_t)(32767.0 * sin(2.0 * PI * i / 256.0));

  // Timer1: 10-bit fast PWM on D9, no prescaler -> 15.625 kHz (output connected in setOutput)
  TCCR1A = _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM12) | _BV(CS10);
  OCR1A  = 0;

  // Timer2: CTC mode, 16 MHz / 64 / 25 = 10 kHz interrupt
  TCCR2A = _BV(WGM21);
  TCCR2B = _BV(CS22);
  OCR2A  = 24;
  TIMSK2 = _BV(OCIE2A);

  applySettings();
  setOutput(outputOn);
  Serial.println(F("Uno signal generator ready. Press the button or type 'on'."));
  printSettings();
}

void loop()
{
  checkButton();
  readSerial();
}