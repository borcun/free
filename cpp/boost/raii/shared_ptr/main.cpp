#include <iostream>
#include <boost/shared_ptr.hpp>

void terminate(int *ptr) {
  std::cout << "shared_ptr is destroyed" << std::endl;
  return;
}

int main() {
  boost::shared_ptr<int> iptr1 {new int{1}, terminate};

  std::cout << "iptr1: " << *iptr1 << std::endl;
  std::cout << "iptr1 unique: " << std::boolalpha << iptr1.unique() << std::endl;
  std::cout << "iptr1 use count: " << std::boolalpha << iptr1.use_count() << std::endl;

  boost::shared_ptr<int> iptr2 {iptr1};
  
  std::cout << "iptr1 unique: " << std::boolalpha << iptr1.unique() << std::endl;
  std::cout << "iptr1 use count: " << std::boolalpha << iptr1.use_count() << std::endl;
  std::cout << "iptr2 use count: " << std::boolalpha << iptr2.use_count() << std::endl;
  std::cout << "iptr2: " << *iptr2 << std::endl;

  iptr1.reset();

  std::cout << "iptr1: " << std::boolalpha << static_cast<bool>(iptr1) << std::endl;
  std::cout << "iptr2: " << std::boolalpha << static_cast<bool>(iptr2) << std::endl;
  
  return 0;
}
