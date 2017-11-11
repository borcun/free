#ifndef SOLVER_HPP
#define SOLVER_HPP

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <string.h>
#include <eigen3/Eigen/Dense>

#define MAX_CHAR 255

// Solver Class
class Solver
{
public:
  // default constructor
  Solver();
  // constructor
  Solver(const char *ipath, const char *opath);
  // destructor
  virtual ~Solver();
  // function that set files paths
  void setPaths(const char *ipath, const char *opath);
  // function that estimates coefficients of function
  void estimate();

private:
  // input file path
  char *m_ipath;
  // output file path
  char *m_opath;
  // input file pointer
  FILE *m_ifile;
  // output file pointer
  FILE *m_ofile;
};

#endif
