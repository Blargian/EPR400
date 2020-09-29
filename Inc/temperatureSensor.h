#include "stdint.h"
#include "stdio.h"

//Table typedef
typedef struct {
  float x;
  float y;
} thermistor_lookUp;

//Prototype methods


//Interpolates through values of a table to estimate the temperature.
float interpolate_table (const thermistor_lookUp * table, float x, int size);

//Accepts a sampled adc value and prints the temperature in degrees celcius.
uint16_t read_temp(uint16_t sample);
float getTemperature(uint16_t adcValue);
