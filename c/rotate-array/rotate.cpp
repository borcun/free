#include <iostream>
#include <vector>

/**
 * function that reverses array, and fulfills it into vector
 * @param [in]
 * @param [in]
 * @param [out]
 * @return
 */
void reverse(int *arr, int size, std::vector<int> &vec) {
  if (size > 1) {
    reverse(arr + 1, size - 1, vec);
  }

  vec.push_back(arr[0]);
}

void rotate(int *arr, int size, int step) {
  std::vector<int> vec;  

  // reverse first k elements, then push them into vector
  reverse(arr, size - step, vec);
  // reverse elements after index k, then push them into vector
  reverse(&arr[size - step], step, vec);
  
  /*
   * reverse copy is a simple copy operation that copies
   * elements from end of vector to begin of vector to array
   */
  std::reverse_copy(vec.begin(), vec.end(), arr);
  
  return;
}

int main() {
  const int n = 5;
  int k = 0;
  int arr[n];

  for (int i = 1; i <= 4; ++i) {
    for (int j = 0; j < n; ++j) {
      arr[j] = j + 1;
    }
    
    k = i;
    std::cout << "[n: " << n << ", k: " << k << "] original: ";
  
    for (int i = 0; i < n; ++i) {
      std::cout << arr[i] << " ";
    }

    std::cout << "=> rotated: ";

    rotate(arr, n, k);

    for (int i = 0; i < n; ++i) {
      std::cout << arr[i] << " ";
    }

    std::cout << std::endl;
  }
  
  return 0;
}
