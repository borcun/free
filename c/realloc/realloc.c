#include <stdio.h>
#include <stdlib.h>

#define SIZE 5

int main(int argc, char **argv)
{
	char *ptr = (char *)malloc(sizeof(char) * SIZE);
	int i;

	for(i=0 ; i < SIZE ; ++i)
		ptr[i] = *argv[1];

	for(i=0 ; i < SIZE ; ++i)
		printf("%c", ptr[i]);
	printf("\n");

	ptr = realloc(ptr, SIZE * 2 * sizeof(char));

	for(i=SIZE ; i < SIZE * 2; ++i)
		ptr[i] = *argv[1] + 1;

	for(i=0 ; i < SIZE * 2 ; ++i)
		printf("%c", ptr[i]);
	printf("\n");

	return 0;
}
