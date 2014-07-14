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
				printf("E ");
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
	int temp_row, temp_col;
	
	srand(time(NULL));
	temp_row = row = rand() % ROW;
	temp_col = col = rand() % COL;
	
	arr[row][col] = 1;
	
	for(i=0 ; i < ROW ; ++i) {
		for(j=0 ; j < COL ; ++j) {
			if(temp_row != 0 && temp_col != 0) {
				--temp_row;
				--temp_col;
				arr[temp_row][temp_col] = 1;
			}
		}
	}
	
	temp_row = row;
	temp_col = col;

	for(i=0 ; i < ROW ; ++i) {
		for(j=0 ; j < COL ; ++j) {
			if(temp_row != ROW - 1 && temp_col != COL - 1) {
				++temp_row;
				++temp_col;
				arr[temp_row][temp_col] = 1;
			}
		}
	}
	
	temp_row = row;
	temp_col = col;

	for(i=0 ; i < ROW ; ++i) {
		for(j=0 ; j < COL ; ++j) {
			if(temp_row != 0 && temp_col != COL - 1) {
				--temp_row;
				++temp_col;
				arr[temp_row][temp_col] = 1;
			}
		}
	}
	
	temp_row = row;
	temp_col = col;

	for(i=0 ; i < ROW ; ++i) {
		for(j=0 ; j < COL ; ++j) {
			if(temp_row != ROW - 1 && temp_col != 0) {
				++temp_row;
				--temp_col;
				arr[temp_row][temp_col] = 1;
			}
		}
	}
	
	print(arr, ROW, COL);

	return 0;
}
