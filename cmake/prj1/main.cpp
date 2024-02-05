#include <iostream>
#include "version.h"

int main(int argc, char **argv) {
  //int value = std::stod(argv[1]);

  std::cout << "Version: " << VERSION_MAJOR << "." << VERSION_MINOR << std::endl;

  //std::cout << "value: " << value << std::endl;

  return 0;
}
