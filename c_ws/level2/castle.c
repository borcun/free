#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define ROW 8
#define COL 8

int row, col;

void print(int arr[][COL], int _row, int _col)
{
	int i, j;
	
	for(i=0 ; i < _row ; ++i) {
		for(j=0 ; j < _col ; ++j) {
			if(i == row && j == col)
				printf("C ");
			else		
				printf("%d ", arr[i][j]);
		}
		printf("\n");
	}
	
	return;
}

int main()
{
	int arr[ROW][COL] = {{0}};
	int i,j;
	
	srand(time(NULL));
	row = rand() % ROW;
	col = rand() % COL;
	
	arr[row][col] = 1;
	
	for(i=0 ; i < ROW ; ++i) {
		for(j=0 ; j < COL ; ++j) {
			if(j == col && i == row)
				arr[i][j] = 8;
			else if(j == col || i == row)
				arr[i][j] = 1;
		}
	}

	print(arr, ROW, COL);

	return 0;
}
