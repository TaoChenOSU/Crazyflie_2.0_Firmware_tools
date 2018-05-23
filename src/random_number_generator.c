#include "random_number_generator.h"

float random_float(float lower, float upper) {
  int random_int = rand();
  // not inclusive
  if(random_int == RAND_MAX) random_int--;
  if(random_int == 0) random_int++;
  float random_float = random_int/(float)RAND_MAX;  // between 0 and 1
  float range = upper - lower;
  return lower + range * random_float;
}

float uniform_float(){
  float x, y, s;
  do{
    x = random_float(-1, 1);
    y = random_float(-1, 1);
    s = x*x+y*y;
  }while(s >= 1);

  return x*sqrt(-2*log(s)/s);
}
