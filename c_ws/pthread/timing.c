#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

void *foo(void *param);

int main()
{
	pthread_t thread;

	pthread_create(&thread, NULL, foo, NULL);
	pthread_join(thread, NULL);

	return 0;
}

void *foo(void *param)
{
	int i=0;

	for(i=0 ; i < 10 ; ++i) {
		printf("%d\n", i);
		sleep(1);
	}
	
	return NULL;
}
