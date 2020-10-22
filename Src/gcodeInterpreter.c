#include "gcodeInterpreter.h"
#include "main.h"
#include "math.h"
#include <stdbool.h>
#define MM_PER_SEGMENT 10

#define Y_DOWN 1
#define Y_UP 0
#define X_RIGHT 0
#define X_LEFT 1

extern volatile float positionX;
extern volatile float positionY;

int currentQuadrant =0;
int targetQuadrant;


/*This function employs Bresenhams line algorithm for drawing straight lines
 * The basic idea of the algorithm is that you can approximate any straight
 * line knowing the starting points and end points through a process of
 * iterative minimisation of errors between the actual line and the 'pixel'
 * or step representation. The x value moves and the algorithm checks for
 * if an increase in Y or no change in Y will get you closer to the actual
 * line. Used for G1 linear moves */


void drawLine(int32_t x0, int32_t y0, int32_t x, int32_t y){

	int32_t dx, //X differential
		dy, //Y differential
		error=0, //Error differential
		x_inc, //increment value for x
		y_inc, //increment value for y
		x_dir, //direction of X
		y_dir, //direction of Y
		i //counter
		;

	//Compute horizontal and vertical deltas between current position and desired position
	dx = fabs(x-x0);
	dy = fabs(y-y0);
	x_dir = (x0<x)?0:1; //if dx is negative then move left otherwise move right
	y_dir = (y0<y)?1:0; //if dy is negative then move down otherwise move up
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

/* returns the angle of dy/dx as a value from 0...2PI */
static float myArctan(float dy,float dx) {
  float a=atan2(dy,dx);
  if(a<0){
	  a=(M_PI*2.0)+a;
  }
  return a;
}

/*This function is used with G2 clockwise or counterclockwise arcs
 * Takes the current position coordinates X0, Y0 [in steps not mm]
 * Takes the end point coordinates X,Y [in steps not mm]
 * Takes the circle pivot I, J [in steps not mm]
 * Takes the direction Clockwise =1 or Counterclockwise = 0
 *
 * 1. Get the circle radius
 * 2. Find the arc angle
 * 3. From the radius and angle determine the length
 * 4. Split that length into line segments
 * 5. Call Bresenhams line algorithm on each segment
 *  */

void drawArc(float x, float y, float I, float J, float direction){

	float x0 = getPosition('X'); //get the start X position
	float y0 = getPosition('Y'); //get the start Y position

	float cx = x0;
	float cy = y0;
	//First get the start radius and startAngle
	float dx = x0-cx;
	float dy= y0-cy;
	float startRadius = sqrt(dx*dx+dy*dy);
	float startAngle = myArctan(dy,dx);

	//Do the same for the end radius and endAngle
	dx = x - cx;
	dy = y - cy;
	float endRadius = sqrt(dx*dx+dy*dy);
	float endAngle = myArctan(dy,dx);

	//Get the arc angle and arc radius
	float arcRadius = endRadius - startRadius;
	float theta = endAngle - startAngle;

	if(direction==0 && theta<0){
		endAngle+=2*M_PI;
	} else if (direction==1 && theta>0){
		startAngle+=2*M_PI;
	}

	theta = endAngle - startAngle; //recalculate theta after corrections

	float length1 = fabs(theta)*startRadius;
	float length = sqrt(length1*length1+arcRadius*arcRadius);

	int segments = fmax(ceil(length/MM_PER_SEGMENT),1); //at this stage we need to convert to steps?

	float i,r, nx, ny;
	float scale,a;

	for(i=0;i<segments;++i){
		//Do the interpolation
		scale = (i/segments);
		a = (theta*scale) + startAngle;
		r = (arcRadius*scale + startRadius);
		nx = cx + cos(a)*r;
		ny = cy + sin(a)*r;
		drawLine(positionToSteps(getPosition('X')),positionToSteps(getPosition('Y')),positionToSteps(nx),positionToSteps(ny));

		x0=nx;
		y0=ny;
	}

	}

/* This function checks whether the current and final points lie in the same octant
 * If we're in the same quadrant then return true, otherwise return false */

bool check(int32_t x0, int32_t y0, int32_t x1, int32_t y1){
	if( (x0>0 && x1>0) || (x0<0 && x1<0) ){ //if the x coordinate is in the same quadrant as the target x coordinate
		if( (y0>0 && y1>0) || (y0<0 && y1<0) ){ //if the y coordinate is in the same quadrant as the target y coordinate
			if(((fabs(x0)>fabs(y0)) && (fabs(x1)>fabs(y1))) || ((fabs(x0)<fabs(y0)) && (fabs(x1)<fabs(y1)))){ //check if we are in the same octant
				return true;
			}
		}
	} else {
		return false;
	}

	return false; //to stop compiler complaining warning: control reaches end of non-void function
}

/* This function checks to see if the operation has completed or not */

bool complete(int32_t x0, int32_t y0, int32_t x1, int32_t y1, int currentQuadrant, int targetQuadrant){
	if(currentQuadrant == targetQuadrant){ //If we are in the same quadrant check if the point is reached
			if(y0==y1){ //if we reached the destination point return true
				return true;
			} else if(x0==x1){
				return true;
			}
	} else {
		return false;
	}
//
//	if(fabs(y1-y0)<1 && fabs(x1-x0)<5){
//		return true;
//	} else {
	return false; //to stop compiler complaining warning: control reaches end of non-void function

}

int getTargetQuadrant(int32_t x1, int32_t y1){
	if((x1>0 && y1>0)){
		if((fabs(y1)>fabs(x1))){
			return 1;
		} else{
			return 2;
		}
	} else if ((x1>0 && y1<0)){
		if((fabs(y1)>fabs(x1))){
			return 4;
		} else{
			return 3;
		}
	} else if ((x1<0 && y1<0)){
		if((fabs(y1)>fabs(x1))){
			return 5;
		} else{
			return 6;
		}
	} else if ((x1<0 && y1>0)){
		if((fabs(y1)>fabs(x1))){
			return 8;
		} else {
			return 7;
		}
	}

	if(x1>0 && y1==0){
		return 9; //right crossover
	} else if (y1<0 && x1==0){
		return 10; //bottom crossover
	} else if (x1<0 && y1==0){
		return 11; //left crossover
	} else {
		return 12; //top crossover
	}
}


/*This algorithm employs Bresenhams circle procedure and uses the G2 format with r rather than I and J*/
/* Method assumes that the circle is drawn from 0,0 but can be changed by making the subsitution
 * x = xi-xc
 * y = yi -yc*/

/*There are more efficient ways to code this but my brain is smol*/

void arc(float x1mm, float y1mm, float I, float J){

//	float oldPositionX = getPosition('X');
//	float oldPositionY = getPosition('Y');

	float referenceX = getPosition('X') + I; //get the reference position
	float referenceY = getPosition('Y') + J;

	//set where we are now as 0
	//Get the radius

	float r = sqrt(I*I + J*J);

	//Make correction for algorithm assuming that the center is (0,0)
	float x0;
	float y0;
	int32_t x;
	int32_t y;
	int32_t x1;
	int32_t y1;

	//These are the cases we have
	if(referenceX>=0){
		if(x1mm>referenceX){
			x1 = +positionToSteps(fabs(x1mm-referenceX));
		//x1<referencex
		} else if (x1mm<referenceX){
			x1 = -positionToSteps(fabs(x1mm-referenceX));
		}
	} else if (referenceX<0){
		if(x1mm>referenceX){
			x1 = +positionToSteps(fabs(x1mm-referenceX));
		//x1<referencex
		} else if (x1mm<referenceX){
			x1 = -positionToSteps(fabs(x1mm-referenceX));
		}
	}

	if(referenceY>=0){
		if(y1mm<referenceY){
			y1 = -positionToSteps(fabs(y1mm-referenceY));
		} else if (y1mm>referenceY){
			y1 = +positionToSteps(fabs(y1mm-referenceY));
		}
	} else if (referenceY<=0){
		if(y1mm<referenceY){
					y1 = -positionToSteps(fabs(y1mm-referenceY));
				} else if (y1mm>referenceY){
					y1 = +positionToSteps(fabs(y1mm-referenceY));
				}
	}

	if(referenceX<0 && referenceY<0){


	}


	int targetQuadrant = getTargetQuadrant(x1,y1);

	//We haven't finished keep stepping
	while(!complete(x,y,x1,y1, currentQuadrant, targetQuadrant)){
		x0 = getPosition('X');
		y0 = getPosition('Y');

		if(referenceX>=0){
			if(x0<referenceX){
				x = -positionToSteps(fabs(x0 - referenceX));
			} else if (x0>referenceX){
				x = +positionToSteps(fabs(x0 - referenceX));
			}
		}

		if(referenceX<0){
			if(x0<referenceX){
				x = -positionToSteps(fabs(x0 - referenceX));
			} else if (x0 > referenceX){
				x = +positionToSteps(fabs(x0 - referenceX));
			}
		}

		if(referenceY>=0){
			if(y0<referenceY){
				y = -positionToSteps(fabs(y0 - referenceY));
			} else if (y0>referenceY){
				y = +positionToSteps(fabs(y0 - referenceY));
			}
		}

		if(referenceY<0){
			if(y0<referenceY){
				y = -positionToSteps(fabs(y0 - referenceY));
			}else if (y0 > referenceY){
				y = +positionToSteps(fabs(y0 - referenceY));
			}
		}

		nextStep(x,y,positionToSteps(r),targetQuadrant);
	}

//	positionX+=oldPositionX;
//	positionY+=oldPositionY;
}

void nextStep(int32_t x, int32_t  y, int32_t r, int quadrant ){
	int incX =1;
	int incY =1;
	int dirX;
	int dirY;
	int32_t errorTerm1;
	int32_t errorTerm2;

	if((fabs(x) >= fabs(y)) && (y!=0)){ //Quadrants 2,3,6 and 7 || (x==y)
		//do checks to see which quadrant (x,y) is in relative to the center point (I,J)

		//Quadrant 2, y and x positive
		if(y > 0 && x > 0){
			currentQuadrant = 2;
			dirX = X_RIGHT;

			errorTerm1 = fabs((x+1)*(x+1) + (y-1)*(y-1) - r*r);
			errorTerm2 = fabs((x+1)*(x+1) + (y)*(y) - r*r);
			if(errorTerm1 < errorTerm2){
				dirY=Y_UP;
				step_y(incY,dirY); //if error term overflows then step x
			} else {
				step_x(incX,dirX); //otherwise keep stepping y
			}

		}
		//Quadrant 3, y negative, x positive
		if(y<0 && x>0){
			currentQuadrant = 3;
			dirX=X_LEFT;

			errorTerm1 = fabs((x-1)*(x-1) + (y-1)*(y-1) - r*r);
			errorTerm2 = fabs((x-1)*(x-1) + y*y - r*r);

			if(errorTerm1 < errorTerm2){
				dirY=Y_UP;
				step_y(incY,dirY); //if the error term underflows then decrease Y
			} else {
				step_x(incX,dirX); //otherwise keep stepping x
			}

		}
		//Quadrant 6, y and x negative
		if(y <0 && x <0){
			currentQuadrant = 6;
			dirX=X_LEFT; //x decreases

			errorTerm1 = fabs((x-1)*(x-1) + (y+1)*(y+1) - r*r);
			errorTerm2 = fabs((x-1)*(x-1) + (y)*(y) - r*r);

			if(errorTerm1 < errorTerm2){
				dirY=Y_DOWN;
				step_y(incY,dirY); //if error term overflows then increase y
			} else {
				step_x(incX,dirX); //Otherwise keep decreasing x
			}

		}
		//Quadrant 7, y positive and x negative
		if(y > 0 && x < 0){
			currentQuadrant = 7;
			dirX=X_RIGHT;

			errorTerm1 = fabs((x+1)*(x+1) + (y+1)*(y+1) - r*r);
			errorTerm2 = fabs((x+1)*(x+1) + y*y - r*r);

			if(errorTerm1 < errorTerm2){
				dirY=Y_DOWN;
				step_y(incY,dirY);
			} else {
				step_x(incX,dirX);
			}

		}
	}

	if((fabs(x) <= fabs(y)) && (x!=0)){ //Quadrants 1,4,5 and 8  || (x==-y)
		//do checks to see which quadrant (x,y) is in relative to the center point (I,J)

		//Quadrant 1, y>0 and x>0
		if(y>0 && x>0){
			currentQuadrant = 1;
			dirY=Y_UP; //Y decreases

			errorTerm1 = fabs((x+1)*(x+1) + (y-1)*(y-1) - r*r);
			errorTerm2 = fabs(x*x + (y-1)*(y-1) - r*r);

			if(errorTerm1 < errorTerm2){
				dirX=X_RIGHT;
				step_x(incX,dirX);
			} else {
				step_y(incY,dirY);
			}
			}

		//Quadrant 4, x is positive and and y negative
		if(y<0 && x>0){
			currentQuadrant = 4;
			dirY=Y_UP; //Y decreases

			errorTerm1 = fabs((x-1)*(x-1) + (y-1)*(y-1) - r*r);
			errorTerm2 = fabs(x*x + (y-1)*(y-1) - r*r);

			if(errorTerm1 < errorTerm2){
				dirX=X_LEFT;
				step_x(incX,dirX);
			} else {
				step_y(incY,dirY);
			}
		}
		//Quadrant 5, x is negative and y is negative
		if(y<0 && x<0){
			currentQuadrant = 5;
			dirY=Y_DOWN;

			errorTerm1 = fabs((x-1)*(x-1) + (y+1)*(y+1) - r*r);
			errorTerm2 = fabs(x*x + (y+1)*(y+1) - r*r);

			if(errorTerm1 < errorTerm2){
				dirX=X_LEFT;
				step_x(incX,dirX);
			} else {
				step_y(incY,dirY);
			}
		}

		//Quadrant 8, x is negative and y is positive
		if(x<0 && y>0){
			currentQuadrant = 8;
			dirY=Y_DOWN;

			if(positionY>7.79){
					__NOP();
				}

			errorTerm1 = fabs((x+1)*(x+1)+ (y+1)*(y+1) - r*r);
			errorTerm2 = fabs(x*x + (y+1)*(y+1) - r*r);

			if(errorTerm1<errorTerm2){
				dirX=X_RIGHT;
				step_x(incX,dirX);
			} else {
				step_y(incY,dirY);
			}
		}

		}

	if((x==0 && y>0)){
		currentQuadrant = 10;
				dirY=Y_UP; //Y decreases

				errorTerm1 = fabs((x+1)*(x+1) + (y-1)*(y-1) - r*r);
				errorTerm2 = fabs(x*x + (y-1)*(y-1) - r*r);

				if(errorTerm1 < errorTerm2){
					dirX=X_RIGHT;
					step_x(incX,dirX);
				} else {
					step_y(incY,dirY);
				}
		}

	if((x==0 && y<0)){
		currentQuadrant = 12;
		dirX=X_LEFT;

			errorTerm1 = fabs((x-1)*(x-1) + (y-1)*(y-1) - r*r);
			errorTerm2 = fabs((x-1)*(x-1) + y*y - r*r);

			if(errorTerm1 < errorTerm2){
				dirY=Y_UP;
				step_y(incY,dirY); //if the error term underflows then decrease Y
			} else {
				step_x(incX,dirX); //otherwise keep stepping x
			}
		}

	if((y==0 && x>0)){
			currentQuadrant = 9;
			dirX=X_LEFT;

			errorTerm1 = fabs((x-1)*(x-1) + (y-1)*(y-1) - r*r);
			errorTerm2 = fabs((x-1)*(x-1) + y*y - r*r);

			if(errorTerm1 < errorTerm2){
				dirY=Y_UP;
				step_y(incY,dirY); //if the error term underflows then decrease Y
			} else {
				step_x(incX,dirX); //otherwise keep stepping x
			}
		}

	if((y==0 && x<0)){
			currentQuadrant = 11;
			dirY=Y_DOWN;

				errorTerm1 = fabs((x-1)*(x-1) + (y+1)*(y+1) - r*r);
				errorTerm2 = fabs(x*x + (y+1)*(y+1) - r*r);

				if(errorTerm1 < errorTerm2){
					dirX=X_LEFT;
					step_x(incX,dirX);
				} else {
					step_y(incY,dirY);
				}
		}
	}






