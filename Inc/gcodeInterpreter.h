#include "stdint.h"
#include <stdbool.h>
#include "lwrb/lwrb.h"

/*Employs Bresenhams algorithm for drawing lines*/
void drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1);

bool check(int32_t x0, int32_t y0, int32_t x1, int32_t y1);
bool complete(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int currentQuadrant, int targetQuadrant);
void arc(int32_t x1mm, int32_t y1mm, float I, float J);
void nextStep(int32_t x0, int32_t y0, int32_t r, int quadrant);
