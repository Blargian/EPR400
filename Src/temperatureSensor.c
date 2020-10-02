#include "temperatureSensor.h"
#include "math.h"

float getTemperature(uint16_t adcValue){
	float y = -0.000000001540*pow(adcValue,3) + 0.000012241208*pow(adcValue,2) - 0.048113478412*adcValue + 91.639615492516; //Third order polynomial fit obtained in Excel
	return y;
}
