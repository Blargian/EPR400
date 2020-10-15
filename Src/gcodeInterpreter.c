#include "gcodeInterpreter.h"

#define STEPS_PER_MM 25;

/*This function employs Bresenhams line algorithm for drawing straight lines
 * The basic idea of the algorithm is that you can approximate any straight
 * line knowing the starting points and end points through a process of
 * iterative minimisation of errors between the actual line and the 'pixel'
 * or step representation. The x value moves and the algorithm checks for
 * if an increase in Y or no change in Y will get you closer to the actual
 * line. */

void drawLine(int x0, int y0, int x, int y){
	int dx, //X differential
		dy, //Y differential
		error, //Error differential
		x_inc, //increment value for x
		y_inc, //increment value for y
		i
		;

	//Compute horizontal and vertical deltas
	dx = x-x0;
	dy = y-y0;

	//Test slope for x

	if(dx>=0){
		x_inc = STEPS_PER_MM;
	}

	else {
		x_inc =-STEPS_PER_MM;
		dx = -dx;
	}

	//Test slope for y

	if(dy>=0){
		y_inc = STEPS_PER_MM;

	} else {
		y_inc=-STEPS_PER_MM;
		dy=-dy;
	}

	//For the case where dx>dy
	if(dx>dy){
		for (i=0; i<=dx;i++){
			error+=dy;
		}

		if(error>dx){
			error==dx;
		}
	}
}
