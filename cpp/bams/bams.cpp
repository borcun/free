/**
 * @file bams.cpp
 * @author boo
 *
 * Implementation of binary scaling algorithm
 */

#include <cstdio>
#include <cstdlib>
#include <cmath>

/**
 * The function that converts signed integer number to signed floating point number
 * 
 * @param [in] num - integer number that holds floating-point value
 * @param [in] max - max. limit of floating-point number
 * @param [in] min - min. limit of floating-point number
 * @param [in] bc - bit count
 * @param [in] sc - scale parameter
 * @return decoded floating-point number
 */
float bscal_decode(unsigned int num, const int max, const int min, const int bc, const int sc) {
  const unsigned int pbc = pow(2, bc - 1);
  float sign = 1.0f;

  // check whether num is pos. or neg.
  if (num >= pbc) {
    sign = -1.0f;
  }

  /* 
   * if num is bigger than max. pos. number, remove sign bit from it.
   * if num is equal to max. pos. number, do nothing, because it's neg. limit.
   */
  if (num > pbc) {
    num -= pbc;
  }
  
  return sign * (float) (num / sc) * ((float) (abs(min) + abs(max)) / (2 * pbc));
}


/**
 * The function that converts signed floating point number to signed integer number
 * 
 * @param [in] num - floating-point number to be converted to integer
 * @param [in] max - max. limit of floating-point number
 * @param [in] min - min. limit of floating-point number
 * @param [in] bc - bit count
 * @param [in] sc - scale parameter
 * @return encoded integer number
 */
unsigned int bscal_encode(float num, const int max, const int min, const int bc, const int sc) {
  // @todo not completed
  return (num * sc * pow(2, bc)) / (abs(min) + abs(max));
}

int main(int argc, char **argv) {
  unsigned int numi = (unsigned int) atoi(argv[1]);
  float numf = atof(argv[2]);

  printf("%f == %d\n", numf, bscal_encode(numf, 8, -8, 8, 1));
  printf("%d == %f\n", numi, bscal_decode(numi, 8, -8, 8, 1));

  printf("\n");
  
  printf("%f == %d\n", numf, bscal_encode(numf, 16, -16, 8, 1));
  printf("%d == %f\n", numi, bscal_decode(numi, 16, -16, 8, 1));
  
  return 0;
}
