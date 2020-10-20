#include "stdint.h"
#include <stdbool.h>

/*Employs Bresenhams algorithm for drawing lines*/
void drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1);

/*Rasterizes an arc into small line segements and runs bresenhams on each of those*/
void drawArc(float x, float y, float I, float J, float direction);

bool check(float x0, float y0, float x1, float y1);
bool complete(float x0, float y0, float x1, float y1);
void arc(float x1mm, float y1mm, float I, float J);
void nextStep(float x0, float y0, float x1, float y1, float r);
