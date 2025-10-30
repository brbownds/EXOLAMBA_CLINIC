// main.c
// Exolamba Clinic
// email
// data
//
// This is the main c code for the phase modulation for the dual active bridge
// Below is the Arduino code in the process to be configured to STM32L432KC MCU

#include <string.h>
#include <stdio.h>
#include "main.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_SPI.h"
#include "STM32L432KC_TIM.h"

// Note the Arduino code is extra in the fact that there was GPIO initialization 
// as well as SPI, etc. We have it so that these are organized in different files

/*

// Fixed Dual Active Bridge (DAB) PWM Controller for Arduino Mega2560
//
// This code generates 4 pairs of complementary PWM signals for a DAB converter.
// - Switches 1, 4, 5, 8 (Group A) operate with duty cycle D.
// - Switches 2, 3, 6, 7 (Group B) operate opposite to Group A.
//
// The fix involves using non-inverting PWM mode for Group A and inverting
// PWM mode for Group B. This creates the required phase shift and dead-time
// without using the flawed TCNT offset method.

#define F_CPU_HZ 16000000UL

// USER DEFAULTS
float duty_default = 0.5;      // 0.0 to 1.0
long freq_default = 20000;     // Hz
float dead_us_default = 0;   // microseconds

// Prescaler options for 16-bit timers
struct PrescalerOption { uint16_t div; uint8_t csBits; };
const PrescalerOption prescalers[] = {
  {1,    0b001},
  {8,    0b010},
  {64,   0b011},
  {256,  0b100},
  {1024, 0b101}
};

// Stops all timers by clearing their clock source bits
void stopAllTimers() {
  TCCR1B &= ~0b111;
  TCCR3B &= ~0b111;
  TCCR4B &= ~0b111;
  TCCR5B &= ~0b111;
}

// Starts all timers synchronously with the same prescaler
void startAllTimersWithPrescaler(uint8_t csBits) {
  // Set the clock source bits for all timers.
  // Starting them sequentially is usually sufficient for this application.
  TCCR1B = (TCCR1B & 0b11111000) | (csBits & 0b111);
  TCCR3B = (TCCR3B & 0b11111000) | (csBits & 0b111);
  TCCR4B = (TCCR4B & 0b11111000) | (csBits & 0b111);
  TCCR5B = (TCCR5B & 0b11111000) | (csBits & 0b111);
}

// Selects the smallest prescaler that allows the frequency to be represented
// within the 16-bit timer range.
bool choosePrescalerForFreq(uint32_t freqHz, uint8_t &outCsBits, uint16_t &outDiv, uint16_t &outTop) {
  for (size_t i = 0; i < sizeof(prescalers)/sizeof(prescalers[0]); ++i) {
    uint32_t div = prescalers[i].div;
    // Calculate TOP = (F_CPU / (freq * prescaler)) - 1
    uint64_t denom = (uint64_t)freqHz * div;
    if (denom == 0) continue;
    uint64_t top64 = (uint64_t)F_CPU_HZ / denom;
    if (top64 == 0) continue;
    top64 = top64 - 1;
    // Check if TOP fits within a 16-bit unsigned integer
    if (top64 <= 0xFFFFu) {
      outTop = (uint16_t)top64;
      outCsBits = prescalers[i].csBits;
      outDiv = (uint16_t)div;
      return true;
    }
  }
  // If no suitable prescaler found, use the largest one and max TOP
  outTop = 0xFFFFu;
  outCsBits = prescalers[sizeof(prescalers)/sizeof(prescalers[0]) - 1].csBits;
  outDiv = prescalers[sizeof(prescalers)/sizeof(prescalers[0]) - 1].div;
  return false;
}

// Configures all timers with the specified frequency, duty cycle, and dead-time
void configurePairs(long freqHz, float duty, float dead_us) {
  if (freqHz <= 0) return;
  duty = constrain(duty, 0.0, 1.0);

  uint8_t chosenCS;
  uint16_t chosenDiv;
  uint16_t TOP;
  choosePrescalerForFreq((uint32_t)freqHz, chosenCS, chosenDiv, TOP);

  // --- Calculate Timer Values ---
  uint32_t ticksPerSec = F_CPU_HZ / (uint32_t)chosenDiv;
  uint16_t deadTicks = (uint16_t)((dead_us * (float)ticksPerSec) / 1e6f);
  uint16_t onTicks = (uint16_t)(duty * (float)TOP);

  // --- Define Compare Values for Both Groups ---
  uint16_t compareA = onTicks;
  uint16_t compareB = onTicks + deadTicks;

  // If the total on-time + dead-time exceeds the period, we must reduce the on-time
  // to preserve the requested dead-time. This prevents shoot-through.
  if (compareB > TOP) {
      compareB = TOP;
      // Check for underflow before subtracting
      if (deadTicks > TOP) {
        compareA = 0;
      } else {
        compareA = TOP - deadTicks;
      }
  }

  // --- Atomically Update Timer Registers ---
  cli(); // Disable interrupts to ensure atomic update

  stopAllTimers();

  // Set TOP value for all timers
  ICR1 = ICR3 = ICR4 = ICR5 = TOP;

  // Group A (S1, S4, S5, S8) use non-inverting mode.
  // PWM is HIGH from TCNT=0 to TCNT=compareA.
  OCR1A = compareA; OCR1B = compareA;
  OCR4A = compareA; OCR4B = compareA;

  // Group B (S2, S3, S6, S7) use inverting mode.
  // PWM is HIGH from TCNT=compareB to TCNT=TOP.
  OCR3A = compareB; OCR3B = compareB;
  OCR5A = compareB; OCR5B = compareB;

  // **FIX**: Reset all timer counters to 0 to align their phase.
  // The incorrect TCNT offset logic has been removed.
  TCNT1 = 0;
  TCNT3 = 0;
  TCNT4 = 0;
  TCNT5 = 0;

  startAllTimersWithPrescaler(chosenCS);

  sei(); // Re-enable interrupts

  // --- Serial Feedback ---
  Serial.print("Freq: "); Serial.print(freqHz);
  Serial.print(" Hz | Duty: "); Serial.print(duty * 100, 2);
  Serial.print("% | TOP: "); Serial.print(TOP);
  Serial.print(" | Prescaler: "); Serial.print(chosenDiv);
  Serial.print(" | dead_us: "); Serial.print(dead_us);
  Serial.print(" | deadTicks: "); Serial.println(deadTicks);
  Serial.print("  -> CompA: "); Serial.print(compareA);
  Serial.print(" | CompB: "); Serial.println(compareB);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Fixed DAB pair-opposite PWM controller");
  Serial.println("Send: <freqHz> <duty> <dead_us>  e.g. 20000 0.6 1.0");

  // Set all PWM pins as outputs
  pinMode(11, OUTPUT); pinMode(12, OUTPUT); // S1, S4 (Timer1 A, B)
  pinMode(5, OUTPUT);  pinMode(2, OUTPUT);  // S2, S3 (Timer3 A, B)
  pinMode(6, OUTPUT);  pinMode(7, OUTPUT);  // S5, S8 (Timer4 A, B)
  pinMode(46, OUTPUT); pinMode(45, OUTPUT); // S6, S7 (Timer5 A, B)

  // Reset all timer control registers
  TCCR1A = TCCR1B = 0;
  TCCR3A = TCCR3B = 0;
  TCCR4A = TCCR4B = 0;
  TCCR5A = TCCR5B = 0;

  // --- Configure All Timers for Fast PWM (Mode 14), TOP set by ICRn ---
  TCCR1B = (1 << WGM12) | (1 << WGM13);
  TCCR3B = (1 << WGM32) | (1 << WGM33);
  TCCR4B = (1 << WGM42) | (1 << WGM43);
  TCCR5B = (1 << WGM52) | (1 << WGM53);

  // --- Configure Output Compare Modes ---
  // **FIX**: Use different modes for the two groups.

  // Group A (Timers 1, 4) -> Switches 1, 4, 5, 8
  // Set on BOTTOM, clear on compare match (non-inverting mode).
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
  TCCR4A = (1 << WGM41) | (1 << COM4A1) | (1 << COM4B1);

  // Group B (Timers 3, 5) -> Switches 2, 3, 6, 7
  // Clear on BOTTOM, set on compare match (inverting mode).
  TCCR3A = (1 << WGM31) | (1 << COM3A1) | (1 << COM3A0) | (1 << COM3B1) | (1 << COM3B0);
  TCCR5A = (1 << WGM51) | (1 << COM5A1) | (1 << COM5A0) | (1 << COM5B1) | (1 << COM5B0);

  // Start with default values
  configurePairs(freq_default, duty_default, dead_us_default);
}

void loop() {
  if (Serial.available()) {
    // Parse arguments from serial input: frequency, duty cycle, dead-time
    long f = Serial.parseInt();
    float d = Serial.parseFloat();
    float dead = Serial.parseFloat();

    // Use defaults if input is invalid
    if (f <= 0) f = freq_default;
    d = constrain(d, 0.0, 1.0);
    if (dead < 0) dead = dead_us_default;

    // Apply the new settings
    configurePairs(f, d, dead);

    // Clear the serial buffer
    while (Serial.available()) Serial.read();
  }
}

*/