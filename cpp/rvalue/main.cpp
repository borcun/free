#include <iostream>

class A {
public:
  A(int x) : m_x(x) { std::cout << "constructor - " << m_x << std::endl; }
  A(const A &obj) { m_x = obj.m_x; std::cout << "copy constructor - " << m_x << std::endl; }
  A(const A &&obj) { m_x = obj.m_x; std::cout << "move constructor - " << m_x << std::endl; }
  virtual ~A() { std::cout << "*destructor - "  << m_x << std::endl; }
  A &operator=(A &a) { std::cout << "assign operator - " << a.m_x << std::endl; return a; }
  
  int m_x = 100;
};

A getA1(int x) {
  return A(x);
}

A getA2(int x) {
  A a(x);
  std::cout << "local var addr: " << &a << std::endl;
  return a;
}

A &getARef(int x) {
  A a(x);
  std::cout << "local ref addr: " << &a << std::endl;
  return a;
}

A &&getAMove(int x) {
  return A(x);
}

int main() {
  A obj0   = std::move(A(3));
  A obj1   = getA1(1);     std::cout << "----------------" << std::endl;  
  A obj2   = getA2(2);     std::cout << "---------------- " << &obj2 << " - " << obj2.m_x << std::endl;
  
  A &obj3  = getARef(3);   std::cout << "---------------- " << &obj3 << " - " << obj3.m_x << std::endl;
  
  A &&obj4 = getAMove(4);  std::cout << "---------------- " << &obj4 << std::endl;
  
  A obj7   = A(7);        std::cout << "----------------" << std::endl;
  
  A obj5   = getARef(5);   std::cout << "---------------- " << &obj5 << " - " << obj5.m_x << std::endl;
  
  A obj6   = getAMove(6);  std::cout << "---------------- " << obj6.m_x << std::endl;

  std::cout << "x-x-x-x-x-x" << std::endl;
  obj5 = getARef(8);
  
  return 0;
}
