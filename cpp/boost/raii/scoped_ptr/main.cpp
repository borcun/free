#include <iostream>
#include <boost/scoped_ptr.hpp>

int main() {
  boost::scoped_ptr<int> iptr {new int{1}};

  std::cout << "*iptr: " << *iptr << std::endl;
  iptr.reset(new int{2});
  std::cout << "*iptr.get(): " << *iptr.get() << std::endl;
  iptr.reset();
  std::cout << std::boolalpha << static_cast<bool>(iptr) << std::endl;
  
  return 0;
}
