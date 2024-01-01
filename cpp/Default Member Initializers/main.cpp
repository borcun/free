#include <iostream>

using namespace std;

struct BitWise {
  int x : 8 = 32;
  int y : 4 = 5;
};

int main() {
  BitWise bw;

  cout << "bw.x : " << bw.x << endl;
  cout << "bw.y : " << bw.y << endl;
  
  return 0;
}
