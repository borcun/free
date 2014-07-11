#include <stdio.h>

void quickSort( int *arr, int first_index, int last_index);
int partition( int *arr, int left, int right);
void swap(int *number1, int *number2);

int main() 
{
  const int size = 20;
  int arr[size] = {18, 10, 19, 6, -1, 4, 4, 7, -9, 0, 11, 12, -6, 8, -8, 1, 5, 22, 3, 20};
  int i;

  printf("Unsorted array : ");
  for(i = 0; i < size ; ++i)
    printf("%d ", arr[i]);

  // call quick sort function
  quickSort(arr, 0, size - 1);

  printf("\nSorted array   : ");
  for(i = 0 ; i < size ; ++i)
    printf("%d ", arr[i]);

  printf("\n");

  return 0;
}

// quick sort function
void quickSort(int *arr, int first_index, int last_index)
{
  if(first_index < last_index)  {
    quickSort(arr, first_index, partition(arr, first_index, last_index) - 1);
    quickSort(arr, partition(arr, first_index, last_index) + 1, last_index);
  }

  return;
}

// wrapper partition function
int partition(int *arr, int left, int right) 
{
  int i = left;
  int j = right;
  
  while(i < j) {
    for( ; arr[i] <= arr[left] && i <= right ; ++i);
    for( ; arr[j] > arr[left] ; --j);
  
    if(i < j)
      swap(&arr[i], &arr[j]);
  }

  swap(&arr[left], &arr[j]);

  return j;
}

// wrapper swap function
void swap(int *number1, int *number2)
{
  int temp = *number1;
  *number1 = *number2;
  *number2 = temp;

  return;
}
