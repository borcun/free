#include <iostream>
#include <list>
#include <algorithm> // for for_each
#include <functional>

int main() {
  std::list<int> iList {1, 2, 4, 8, 10};
  std::function<void(int)> fxn = [](auto &&num) {
				   std::cout << num << " ";
				 };
  
  std::for_each(iList.begin(),
		iList.end(),
		fxn);

  std::cout << std::endl;				      

  return 0;
}
