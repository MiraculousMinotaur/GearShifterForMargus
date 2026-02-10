// Fixed-point utilities for main sketch
#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H

#include <stdint.h>

// Small-scale fixed point for controller gains
typedef int16_t q8_t;    // Q8 fixed-point (fractional precision = 1/256)
typedef int32_t q32_t;

#define SCALE_Q8 256
#define SCALE_10000 10000

// Parse decimal string to Q8 fixed-point value. Supports optional sign and fractional part.
// Examples: "1" -> 256, "0.5" -> 128, "-0.25" -> -64
static inline q8_t parse_fixed_q8(const char *s)
{
  if (s == nullptr) return 0;
  bool neg = false;
  if (*s == '+') ++s;
  if (*s == '-') { neg = true; ++s; }
  long intPart = 0;
  while (*s >= '0' && *s <= '9') { intPart = intPart * 10 + (*s - '0'); ++s; }
  long fracPart = 0;
  int fracLen = 0;
  if (*s == '.') {
    ++s;
    while (*s >= '0' && *s <= '9' && fracLen < 4) { // limit fractional digits
      fracPart = fracPart * 10 + (*s - '0'); ++s; ++fracLen;
    }
  }
  // Compose Q8 value: intPart * 256 + fractional scaled
  long value = intPart * SCALE_Q8;
  if (fracLen > 0) {
    long divisor = 1;
    for (int i = 0; i < fracLen; ++i) divisor *= 10;
    long fracScaled = (fracPart * SCALE_Q8) / divisor;
    value += fracScaled;
  }
  if (neg) value = -value;
  return (q8_t)value;
}

#endif // FIXEDPOINT_H
