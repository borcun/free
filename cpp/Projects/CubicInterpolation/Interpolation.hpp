#ifndef INTERPOLATION_HPP
#define INTERPOLATIN_HPP

#include "Solver.hpp"
#include <tbb/concurrent_queue.h>
#include <pthread.h>
#include <unistd.h>

// function that estimates interpolation
void *estimate(void *);
// function that prints data into output file
void printResult();

class Interpolation
{
public:
  // default constructor
  Interpolation();
  // constructor
  Interpolation(const char *ipath, const char *opath);
  //destructor
  virtual ~Interpolation();
  // function that sets input and ouput paths
  void setPaths(const char *ipath, const char *opath);
  // function that starts interpolation
  void interpolate();

private:
  // function that calls callback function via timer
  void timer(const int microsec, void(*callback)());
  friend void *estimate(void *param);
  friend void printResult();
};

#endif
