#include "list.hpp"

int main(int argc, const char * argv[]) {
  LinkedList linkedList;
  int value = 0;
  
  value = 5;
  linkedList.push_front(value);
  linkedList.print();

  value = 2;
  linkedList.push_back(value);
  linkedList.print();

  value = 4;
  linkedList.push_back(value);
  linkedList.print();

  value = 8;
  linkedList.push_front(value);
  linkedList.print();
  
  value = 6;
  linkedList.push_front(value);
  linkedList.print();
  
  std::cout << "size of the list: " << linkedList.size() << std::endl;

  linkedList.pop_front();
  linkedList.print();

  linkedList.pop_back();
  linkedList.print();
  
  linkedList.pop_back();
  linkedList.print();
  
  linkedList.pop_front();
  linkedList.print();

  std::cout << "size of the list: " << linkedList.size() << std::endl;

  return 0;
}
