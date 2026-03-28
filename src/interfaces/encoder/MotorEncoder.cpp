#include "MotorEncoder.h"

#ifdef ARDUINO
#include <Arduino.h>
#ifdef __AVR__
#include <avr/interrupt.h>
#endif
#endif

namespace Willow {

namespace {
#ifdef __AVR__
uint8_t EnterCritical() {
  const uint8_t previousSreg = SREG;
  cli();
  return previousSreg;
}

void ExitCritical(uint8_t previousSreg) { SREG = previousSreg; }
#elif defined(ARDUINO)
uint8_t EnterCritical() {
  noInterrupts();
  return 0;
}

void ExitCritical(uint8_t /*unused*/) { interrupts(); }
#else
uint8_t EnterCritical() { return 0; }
void ExitCritical(uint8_t /*unused*/) {}
#endif
}

long MotorEncoder::ConsumePulseDelta() {
  const long pulseCount = GetPulseCount();
  const long delta = pulseCount - lastConsumedPulseCount_;
  lastConsumedPulseCount_ = pulseCount;
  return delta;
}

long MotorEncoder::GetPulseCount() const {
  const uint8_t criticalState = EnterCritical();
  const long pulseCount = pulseCount_;
  ExitCritical(criticalState);
  return pulseCount;
}

void MotorEncoder::ResetPulseTracking() {
  pulseCount_ = 0;
  lastConsumedPulseCount_ = 0;
}

void MotorEncoder::AddPulseDelta(long delta) { pulseCount_ += delta; }

}  // namespace Willow
