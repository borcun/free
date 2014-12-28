#include "Profile.hpp"

// default constructor
Profile::Profile()
{
}

// destructor
Profile::~Profile()
{
  m_coeff_vec.clear();
}

// load function
void Profile::load(const char *path)
{
  FILE *m_input_file;
  char line[MAX_CHAR];
  Coefficients coefficients;
  int time_stamp = 1;

  // open input file
  if(NULL == (m_input_file = fopen(path, "r"))) {
    std::cerr << "Input file can not be opened" << std::endl;
    return;
  }

  // load coeffcient file
  while(NULL != fgets(line, MAX_CHAR, m_input_file)) {
    coefficients.a = atof(strtok(line, ","));
    coefficients.b = atof(strtok(NULL, ","));
    coefficients.c = atof(strtok(NULL, ","));
    coefficients.d = atof(strtok(NULL, ","));
    coefficients.stime = time_stamp;
 
    m_coeff_vec.push_back(coefficients);
 
   // increase time stamp by one because of one second between time instances
    ++time_stamp;
  }

  fclose(m_input_file);

  return;
}

// value function
float Profile::value(const float param)
{
  int i;

  for(i=0 ; i < m_coeff_vec.size() ; ++i) {
    // ! binary search will be added instead of linear search
    if(m_coeff_vec.at(i).stime > param) {
      // get true range with decreasing i
      --i;
      break;
    }
  }

  // after last time instance, function goes on as linear
  if(i == m_coeff_vec.size())
    --i;

  return m_coeff_vec.at(i).a * param * param * param + 
    m_coeff_vec.at(i).b * param * param + 
    m_coeff_vec.at(i).c * param + 
    m_coeff_vec.at(i).d;
}
