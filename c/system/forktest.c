#include <stdio.h>
#include <unistd.h>

int main()
{
	pid_t pid;
	
	pid = fork();
	
	if(!pid)
		printf("Mommy\n");
	else
		printf("Kid\n");
	
	return 0;
}
