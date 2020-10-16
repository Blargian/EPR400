#include "gcodeInterpreter.h"
#include "main.h"

#define STEPS_PER_MM 800

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
		error=0, //Error differential
		x_inc, //increment value for x
		y_inc, //increment value for y
		x_dir, //direction of X
		y_dir, //direction of Y
		i //counter
		;

	//Compute horizontal and vertical deltas between current position and desired position
	dx = x-x0;
	dy = y-y0;
	x_dir = dx<0?1:0; //if dx is negative then move left otherwise move right
	y_dir = dy<0?0:1; //if dy is negative then move down otherwise move up

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
			step_x(x_inc,x_dir);
			//Test if error overflows
			if(error>dx){
				error-=dx;
				step_y(y_inc,y_dir);
			}
				step_x(x_inc,x_dir); //take another step in the x direction
		}
	} else {
		for (i=0;i<dy;i++){
			step_y(y_inc,y_dir);
			error+=dx;
			//Test if error overflows
			if(error>0){
				error-=dy;
				step_x(x_inc,x_dir);
			}

			step_y(y_inc,y_dir);
		}
	}
}
