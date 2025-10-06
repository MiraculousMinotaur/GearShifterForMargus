#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

template <typename T> inline T limitVal(T value, T minVal, T maxVal)
{
  if(value < minVal) return minVal;
  if(value > maxVal) return maxVal;
  return value;
}

#endif // UTILS_H
