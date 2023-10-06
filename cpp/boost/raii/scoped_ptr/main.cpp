#include <iostream>
#include <boost/scoped_ptr.hpp>

class ScopedPtrTest {
public:
  ScopedPtrTest() { std::cout << __FUNCTION__ << std::endl; }
  ~ScopedPtrTest() { std::cout << __FUNCTION__ << std::endl; }
};

int main() {
  boost::scoped_ptr<int> iptr {new int{1}};
  boost::scoped_ptr<ScopedPtrTest> spt_ptr {new ScopedPtrTest};
  
  std::cout << "*iptr: " << *iptr << std::endl;
  iptr.reset(new int{2});
  std::cout << "*iptr.get(): " << *iptr.get() << std::endl;
  iptr.reset();
  std::cout << std::boolalpha << static_cast<bool>(iptr) << std::endl;
  
  return 0;
}
