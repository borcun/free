#include <iostream>
#include <string>
#include <fstream>
#include <regex>

int main() {
  std::string str = "2304 (RDD Process) S 1763 some other fields";

  // Define the regex pattern
  std::regex pattern(R"((\d+)\s\(([^)]+)\)\s([A-Z])\s(\d+))");
  std::smatch matches;

  // Perform regex matching
  if (std::regex_search(str, matches, pattern)) {
    if (matches.size() == 5) { // 1 full match and 4 sub-matches expected
      int pid = std::stoi(matches[1]);
      std::string name = matches[2];
      char status = matches[3].str()[0]; // convert the match to a char
      int ppid = std::stoi(matches[4]);

      // Output the parsed values
      std::cout << "PID: " << pid << std::endl;
      std::cout << "Name: " << name << std::endl;
      std::cout << "Status: " << status << std::endl;
      std::cout << "PPID: " << ppid << std::endl;
    } else {
      std::cerr << "Unexpected number of matches" << std::endl;
    }
  } else {
    std::cerr << "No match found" << std::endl;
  }

  return 0;
}  

int mainx() {
  std::ifstream file("/proc/3165/stat");
  std::string line;
  std::string delimiter = " ";
  size_t pos = 0;
  std::string token;
  int count = 0;

  if (!file.is_open()) {
    std::cerr << "could not open file" << std::endl;
    return -1;
  }

  getline(file, line);
  
  while ((pos = line.find(delimiter)) != std::string::npos && count < 4) {
    token = line.substr(0, pos);
    std::cout << token << std::endl;
    line.erase(0, pos + delimiter.length());
    ++count;
  }

  file.close();
    
  return 0;
}
