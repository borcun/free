#include "Solver.hpp"

using namespace std;
using namespace Eigen;

// default constructor
Solver::Solver()
{
}

// constructor
Solver::Solver(const char *ipath, const char *opath)
{
  setPaths(ipath, opath);
}

// destructor
Solver::~Solver()
{
  delete m_ipath;
  delete m_opath;
}

// function that set path
void Solver::setPaths(const char *ipath, const char *opath)
{
  m_ipath = new char[ strlen(ipath) ];
  m_opath = new char[ strlen(opath) ];
  strcpy(m_ipath, ipath);
  strcpy(m_opath, opath);

  return;
}

// function that estimate coefficients
void Solver::estimate()
{
  char read_data[MAX_CHAR] = {'\0'};
  int t1 = 0, t2 = 0;
  float Q1 = 0.0, Q2 = 0.0;
  float V1 = 0.0, V2 = 0.0;
  float a = 0.0, b = 0.0, c = 0.0, d = 0.0;
  MatrixXd left_mat(2, 2);
  VectorXd ab_vec(2);
  VectorXd res_mat(2);
  float det = 0.0;

  // open input file
  if(NULL == (m_ifile = fopen(m_ipath, "r"))) {
    std::cerr << "Input file can not be opened" << std::endl;
    return;
  }

  // open output file
  if(NULL == (m_ofile = fopen(m_opath, "w+"))) {
    std::cerr << "Output file can not be opened" << std::endl;
    return;
  }

  // read data from input file
  for(int i=0 ; NULL != fgets(read_data, MAX_CHAR, m_ifile) ; ++i) {      
    t2 = atoi(strtok(read_data, ","));
    Q2 = atof(strtok(NULL, ","));
    V2 = atof(strtok(NULL, ","));

    if(i != 0) {
      // set d coeffecient
      d = Q1;
      // set c coefficient
      c = V1;

      // calculate determinant
      det = pow(t2 - t1, 3) * 2 * (t2 - t1) - pow(t2 - t1, 2) * 3 * pow(t2 - t1, 2);
      // set left size multiplication matrix
      left_mat(0, 0) = 2 * (t2 - t1) / det; // pow(t2 - t1, 3);
      left_mat(0, 1) = -1 * pow(t2 - t1, 2) / det;
      left_mat(1, 0) = -1 * 3 * pow(t2 - t1, 2) / det;
      left_mat(1, 1) = pow(t2 - t1, 3) / det; // 2 * (t2 - t1);

      // set result matrix
      res_mat(0) = (Q2 - V1 * (t2 - t1) - Q1);
      res_mat(1) = (V2 - V1 * (t2 - t1));
      
      // solve linear system
      //ab_vec = left_mat.fullPivHouseholderQr().solve(res_mat);
      ab_vec = left_mat * res_mat;

      // set b coefficient
      b = ab_vec(1);
      // set a coefficient
      a = ab_vec(0);
      
      // write coefficients into file
      fprintf(m_ofile, "%f,%f,%f,%f\n", a, b, c, d);
    }

    // set first paramaters with second parameters for next estimation
    t1 = t2;
    Q1 = Q2;
    V1 = V2;
  } // end of for

  // close files
  fclose(m_ifile);
  fclose(m_ofile);

  return;
}
