#include <iostream>

class MyClass {
public:
  explicit MyClass(int x) : m_x(x) {
    std::cout << "constructor: " << m_x << std::endl;
  }

  MyClass(const MyClass &ins) {
    m_x = ins.m_x + 1;
    std::cout << "copy ctor: " << m_x << std::endl;
  }

  MyClass(const MyClass &&ins) {
    m_x = std::move(ins.m_x + 2);
    std::cout << "move ctor: " << m_x << std::endl;
  }

  virtual ~MyClass() {
    std::cout << "destructor: " << m_x << std::endl;
  }

  MyClass &operator=(const MyClass &ins) {
    std::cout << "copy assignment operator" << std::endl;
    return *this;
  }

  MyClass &operator=(const MyClass &&ins) {
    std::cout << "move assignment operator" << std::endl;
    return *this;
  }

  int m_x;
};

MyClass print(const MyClass ins) {
  std::cout << "ins.x : " << ins.m_x << std::endl;
  return ins;
}

int main() {
  /*
  MyClass ins1 {1};
  MyClass ins2(2);
  MyClass ins3 = std::move(ins1);
  MyClass ins4 {20};
  
  ins1.m_x = 10;
  ins2.m_x = 11;
  ins3.m_x = 12;

  std::cout << "-------" << std::endl;
  ins4 = ins1;
  ins4 = std::move(ins1);
  std::cout << "-------" << std::endl;
  print(ins1);   std::cout << "-------" << std::endl;
  print(ins2);   std::cout << "-------" << std::endl;
  print(ins3);   std::cout << "-------" << std::endl;
  print(MyClass(100));  std::cout << "-------" << std::endl;
  */

  MyClass ins1 {1};
  MyClass ins2 = print(ins1);

  std::cout << "ins2.m_x: " << ins2.m_x << std::endl;
  
  return 0;
}
