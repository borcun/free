#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

// thread function
void *foo(void *param);

int main()
{
	pthread_t thread;

	// create thread
	pthread_create(&thread, NULL, foo, NULL);
	// wait parent thread until child one finishs its job
	pthread_join(thread, NULL);

	return 0;
}

void *foo(void *param)
{
	int i;

	for(i=0 ; i < 10 ; ++i) {
		printf("%d\n", i);
		sleep(1);
	}
	
	return NULL;
}
