#include <stdio.h>

int add(int, int);
int eval(int (*)(int, int), int, int);

int main()
{
	int num1 = 3;
	int num2 = 5;

	printf("sum: %d\n", eval(add, num1, num2));

	return 0;
}

int add(int num1, int num2)
{
	return num1 + num2;
}

int eval(int(*callback)(int, int), int num1, int num2)
{
	return callback(num1, num2);
}
