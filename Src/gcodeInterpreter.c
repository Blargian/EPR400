#include "gcodeInterpreter.h"
#include "main.h"


/*This function employs Bresenhams line algorithm for drawing straight lines
 * The basic idea of the algorithm is that you can approximate any straight
 * line knowing the starting points and end points through a process of
 * iterative minimisation of errors between the actual line and the 'pixel'
 * or step representation. The x value moves and the algorithm checks for
 * if an increase in Y or no change in Y will get you closer to the actual
 * line. */


void drawLine(uint32_t x0, uint32_t y0, uint32_t x, uint32_t y){

	uint32_t dx, //X differential
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
	x_inc=1; //step x by 1 each time (1.25 micro meters)
	y_inc=1; //step y by 1 each time (1.25 micro meters)

	if(dx >=dy){
		for(i=0;i<dx;i++){
			step_x(x_inc,x_dir);
			error+=dy;
			if(error>=dx){
				error-=dx;
				step_y(y_inc,y_dir);
			}
		}
	} else {
		for(i=0;i<dy;i++){
			step_y(y_inc,y_dir);
			error-=dy;
			step_y(y_inc,y_dir);
		}
	}
}
