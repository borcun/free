#include <iostream>
#include <string>
#include <vector>
#include <exception>

bool valid(std::string addr) {
  size_t pos = 0;
  std::string delimiter = ".";
  std::string token;
  std::vector<int> fields;
  
  try {
    while ((pos = addr.find(delimiter)) != std::string::npos) {
      token = addr.substr(0, pos);
      fields.push_back(std::stoi(token));
      addr.erase(0, pos + delimiter.length());
    }

    fields.push_back(std::stoi(addr));

    if (4 != fields.size()) {
      return false;
    }

    for(auto &field : fields) {
      if (field > 255) {
	return false;
      }
    }
  } catch (std::exception &ex) {
    std::cout << "error: " << ex.what() << std::endl;
    return false;
  }
    
  return true;
}

int main() {
  std::cout << valid("127.0.0.1") << std::endl;
  std::cout << valid("192.168.0.23") << std::endl;
  std::cout << valid("192.168.0") << std::endl;
  std::cout << valid("192.168.0.256") << std::endl;
  std::cout << valid("A.B.C.D") << std::endl;
  std::cout << valid("0.1.2.3") << std::endl;
    
  return 0;
}
