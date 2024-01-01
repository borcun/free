#include <iostream>

class Test {
public:
  Test(void) = default;
  Test(const int x) = delete;
};

int main() {
  Test test1;
  Test test2(5);
  
  return 0;
}
