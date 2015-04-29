#include "tbb/parallel_sort.h"
#include <math.h>
#include <iostream>
// namespace
using namespace tbb;
using namespace std;
// create two array and their size
const int N = 10;
float a[N];
float b[N];

// function that sorts array with using parallel_sort function
void SortExample() {
	// fill arrays
    for( int i = 0; i < N; i++ ) {
       a[i] = sin((double)i);
       b[i] = cos((double)i);
    }
    // sort a array from less to great
    parallel_sort(a, a + N, std::less<float>());
    // sort b array from great to less
    parallel_sort(b, b + N, std::greater<float>());

    return;
}

// main function
int main()
{
	// sort sequence
	SortExample();

	// print a array on screen
	for(int i=0 ; i < N ; ++i) {
		cout << a[i] << " ";
	}
	cout << endl;
	// print b array on screen
	for(int i=0 ; i < N ; ++i) {
		cout << b[i] << " ";
	}
	cout << endl;

	return 0;
}
