
#include <stdlib.h>
#include <stdint.h>
#include "math2.h"
#include "os.h"
#include "us.h"

#define PI 314
#define MAGIC 28
#define SCALE 100

#define RAD2DEG(x) ((x * 180) / PI)

uint16_t fxpt_atan2(const int16_t y, const int16_t x);

int wall_angle(int d0, int d1) {
  //y = 1/sqrt(2)*d0 + 48.1 - 1/2*d1
  //x = sqrt(3)/2*d1 + 9.8 - 1/sqrt(2)*d0
  //scaling factor = 46, prescale = 256
  //prescale by 46*256 to get better nums, but true scaling is only 46
  //can only scale by 46 due to limit of signed 16 bits
  
  int y = (8327*d0 + 566426 - 5888*d1 + 128)/256;
  int x = (10198*d1 + 115405 - 8327*d0 + 128)/256;
  //convert from 1/65536th turns to degrees
  int result = ((int) fxpt_atan2((int16_t) y, (int16_t) x))*360/65536;
  if(result>270){ //map 270->360 degrees as -90->0
    result = result - 270;
  }
	return result;
}
