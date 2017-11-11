#ifndef PROFILE_HPP
#define PROFILE_HPP

#include "Solver.hpp"
#include <iomanip>
#include <vector>

// polynom coefficient structure
struct PolynomCoefficients
{
  float a;
  float b;
  float c;
  float d;
  int stime;
};

typedef struct PolynomCoefficients Coefficients;

class Profile
{
public:
  // default constructor
  Profile();
  // destructor
  ~Profile();
  // function that loads coefficients
  void load(const char *path);
  // functiton that calculates result of polynom with parameter
  float value(const float param);

private:
  // coefficient vector
  std::vector<Coefficients> m_coeff_vec;

};

#endif
