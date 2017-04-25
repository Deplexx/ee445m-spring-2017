#include "atan.h"

#include <stdlib.h>


#define PI 314
#define MAGIC 28
#define SCALE 100

#define RAD2DEG(x) (x * 180 * SCALE / PI)

int atan(int x, int y) {
	int abs_x, abs_y;
	int abs_sx, abs_sy;
	int res;
		
	abs_x = abs(x);
	abs_y = abs(y);
	
	abs_sx = abs_x * SCALE;
	abs_sy = abs_y * SCALE;
	
	if(abs_x > abs_y) //o1
		res = (abs_sx * abs_sy) / (abs_sx * abs_sx + MAGIC * abs_y * abs_y);
	else //o2
		res = (PI / 2) - (abs_sy * abs_sy) / (abs_sy * abs_sy + MAGIC * abs_x * abs_x);		
	
	return (x > 0) ? RAD2DEG(res) : RAD2DEG(res + PI / 2) / SCALE;
}