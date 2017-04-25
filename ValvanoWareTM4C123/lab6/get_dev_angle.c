#include "atan.h"
#include "get_dev_angle.h"


int get_dev_angle(int d1, int d2, int d12) {
	return atan(d2 - d1, d12);
}