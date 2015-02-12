#include <stdio.h>

int main()
{
	int key;
	char ch;

	scanf("%d", &key);
	
	if(5 == key)
		printf("bingo\n");
	else
		printf("try again\n");
	
	return 0;
}
