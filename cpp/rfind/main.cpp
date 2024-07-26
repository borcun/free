#include <iostream>
#include <set>
#include <string>

int main() {
  std::set<std::string> src {"Worldx", "ABCx", "Hellox"};
  std::string target = "Hello";

  for (const auto &str : src) {
    if (0 == str.rfind("Hello", 0)) {
      std::cout << str << " starts with " << target << std::endl;
    }
  }
  
  return 0;
}
