
#include <stdlib.h>
#include <stdint.h>
#include "math2.h"
#include "os.h"

#define PI 314
#define MAGIC 28
#define SCALE 100

#define RAD2DEG(x) ((x * 180) / PI)

// int wall_angle(int d0, int d1) {
	// int y = 707*d0+48100-500*d1;
	// int x = 866*d1+9800-707*d0;
	// return atan(x,y);
// }

uint16_t fxpt_atan2(const int16_t y, const int16_t x);

int wall_angle(int d0, int d1) {
  //y = 1/sqrt(2)*d0 + 48.1 - 1/2*d1
  //x = sqrt(3)/2*d1 + 9.8 - 1/sqrt(2)*d0
  //scaling factor = 46, prescale = 256
  
  int y = (8327*d0 + 566426 - 5888*d1 + 128)/256;
  int x = (10198*d1 + 115405 - 8327*d0 + 128)/256;
  int result = ((int) fxpt_atan2((int16_t) y, (int16_t) x))*360/65536;
	return result;
}

int wall_angle2(int d0, int d1, unsigned long *time) {
  unsigned long first = OS_Time();
  int y = (8327*d0 + 566426 - 5888*d1 + 128)/256;
  int x = (10198*d1 + 115405 - 8327*d0 + 128)/256;
  int result = ((int) fxpt_atan2((int16_t) y, (int16_t) x))*360/65536;
  *time = OS_Time() - first;
	return result;
}

int atan(int x, int y) {
	int abs_x, abs_y;
	int res;
		
	abs_x = abs(x);
	abs_y = abs(y);
	
	if(abs_x > abs_y) //o1 
		res = (abs_x * abs_y * SCALE) / (abs_x * abs_x + (MAGIC * abs_y * abs_y) / SCALE);
	else if(x == 0)
    res = PI / 2;
  else if(y == 0)
    res = 0;
  else //o2
		res = (PI / 2) - (abs_y * abs_y * SCALE) / (abs_y * abs_y + (MAGIC * abs_x * abs_x) / SCALE);		
	
	if((x<0 && y>0) || (x>0 && y<0))
		return -RAD2DEG(res);
	else
		return RAD2DEG(res);
}
