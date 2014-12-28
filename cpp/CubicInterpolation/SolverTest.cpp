#include "Solver.hpp"

int main(int argc, char **argv)
{
  if(argc != 3)
    return -1;

  Solver solver(argv[1], argv[2]);
  solver.estimate();

  return 0;
}
