#include "math2.h"

#include <stdlib.h>


#define PI 3141
#define MAGIC 281
#define SCALE 1000

#define RAD2DEG(x) (x * 180 * SCALE / PI)

int wall_angle(int d0, int d1) {
	int y = 707*d0+48100-500*d1;
	int x = 866*d1+9800-707*d0;
	return atan(x,y);
}

int atan(int x, int y) {
	int abs_x, abs_y;
	int res;
	
		
	abs_x = abs(x);
	abs_y = abs(y);
	
	if(abs_x > abs_y) //o1
		res = (abs_x * abs_y) / (abs_x * abs_x + (MAGIC * abs_y * abs_y) / SCALE);
	else //o2
		res = (PI / 2) - (abs_y * abs_y) / (abs_y * abs_y + (MAGIC * abs_x * abs_x) / SCALE);		
	
	if((x<0 && y>0) || (x>0 && y<0))
		return -RAD2DEG(res) / SCALE;
	else
		return RAD2DEG(res) / SCALE;
}