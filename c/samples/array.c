#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define SIZE 10

void print(int arr[], int size);
void modify(int arr[], int size);

int main()
{
	int arr[SIZE] = {0};

	print(arr, SIZE);
	modify(arr, SIZE);
	print(arr, SIZE);
		
	return 0;
}

void print(int arr[], int size)
{
	int i;
	
	for(i=0 ; i < size ; ++i)
		printf("%d ", arr[i]);
		
	printf("\n");
	return;
}

void modify(int arr[], int size)
{
	int i;

	srand(time(NULL));
	
	for(i=0 ; i < size ; ++i)
		arr[i] = 1 + rand() % 20;
		
	return;
}
