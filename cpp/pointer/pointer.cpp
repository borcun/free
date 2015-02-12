#include <iostream>
#include <cstdio>

int main()
{
	int **p2;

	{
		int *p1 = new int[2];

		p1[0] = 4;
		p1[1] = 6;

		printf("p1[0] : %d, p1[1] : %d\n", p1[0], p1[1]);
		printf("p1 addr: %x\n", p1);

		p2 = &p1;

		delete p1;
		p1 = NULL;
	}

	printf("p1 addr via p2: %x\n", *p2);

	printf("p2[0] : %d, p2[1] : %d\n", *p2[0], *p2[1]);

	return 0;
}
