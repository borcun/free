#include <iostream>
#include <armadillo>

using namespace std;
using namespace arma;

int main(int argc, char **argv) {
  std::cout << arma_version::as_string() << std::endl;

  mat Z = mat();
  mat A = mat(2, 2, fill::zeros);
  mat B = mat(2, 2, fill::ones);
  mat C = mat(2, 2);
  mat D = mat(2, 2);

  std::cout << "A(row, col) : (" << A.n_rows << "," << A.n_cols << ")" << std::endl;
  std::cout << "B(row, col) : (" << B.n_rows << "," << B.n_cols << ")" << std::endl;
  std::cout << "Z(row, col) : (" << Z.n_rows << "," << Z.n_cols << ")" << std::endl;

  return 0;
}
