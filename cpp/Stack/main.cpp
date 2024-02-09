#include <iostream>
#include "iStack.h"

template <class T>
void testStack(int cap, T incr) {
  Stack<float> stack(cap);

  std::cout << "stack capability: " << cap << std::endl;
  std::cout << "stack size: " << stack.size() << std::endl;

  for (int i = 0; i < cap; ++i) {
    stack.push(static_cast<T>(i) * incr);
  }

  std::cout << "stack size: " << stack.size() << std::endl;

  for (int i = 0; i < cap; ++i) {
    std::cout << "top: " << static_cast<T>(*(stack.top())) << std::endl;
    stack.pop();
  }

  std::cout << "stack size: " << stack.size() << std::endl;

  return;
}

int main() {
  testStack(5, 1.0);
  std::cout << "------------------------" << std::endl;
  
  testStack(10, 3.3);
  std::cout << "------------------------" << std::endl;

  testStack(7, 7.2);
  
  return 0;
}
