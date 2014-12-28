#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(int argc, char **argv)
{
	int random;
	int pass;

	srand(time(NULL));
	random = rand() % 50;

	scanf("%d", &pass);

	if(pass == random) {
		printf("Congrulations ! Password is cracked.\n");
		return 0;
	}
	else
		printf(" try again: %d %d\n", random, pass);

	return 0;
}
