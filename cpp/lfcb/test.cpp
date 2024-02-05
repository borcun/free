#include <iostream>
#include "lfcb.h"

int main() {
  int enqElem = 4;
  int deqElem = 0;

  if (0 != lfcb_init()) {
    std::cerr << "[FAILED] Initialization failed" << std::endl;
    return -1;
  }

  std::cout << "[DONE] Init" << std::endl;
    
  if (0 != lfcb_enqueue(enqElem)) {
    std::cerr << "[FAILED] Enqueuing failed" << std::endl;
    return -1;
  }

  std::cout << "[DONE] Enqueue" << std::endl;
  
  if (0 != lfcb_dequeue(deqElem)) {
    std::cerr << "[FAILED] Dequeuing failed" << std::endl;
    return -1;
  }

  std::cout << "[DONE] Dequeue" << std::endl;

  if (enqElem != deqElem) {
    std::cerr << "[FAILED] Could not match element that consecutively enqueued and dequeued" << std::endl;
    return -1;
  }
  
  return 0;
}
