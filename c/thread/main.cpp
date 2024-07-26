#include <stdio.h>
#include <threads.h>
#include <time.h>

mtx_t mutex;
cnd_t cond_var;

int func1(void *args) {
    timespec ts = {1, 0};
    
    for (int i = 0; i < 5; ++i) {
	printf("%s iteration %d\n", __PRETTY_FUNCTION__, i + 1);
	thrd_sleep(&ts, NULL); // sleep 1 sec

	if (2 == i) {
	    cnd_signal(&cond_var);
	}
    }

    return 0;
}

int func2(void *args) {
    timespec ts = {1, 0};

    cnd_wait(&cond_var, &mutex);
	
    for (int i = 0; i < 3; ++i) {
	printf("%s iteration %d\n", __PRETTY_FUNCTION__, i + 1);
	thrd_sleep(&ts, NULL); // sleep 1 sec
    }

    return 0;
}

int main() {
    thrd_t thr1, thr2;
    int res_code;

    if (thrd_success != mtx_init(&mutex, mtx_plain)) {
	printf("%s\n", "could not create mutex");
	return -1;
    }

    if (thrd_success != cnd_init(&cond_var)) {
	printf("%s\n", "could not create condition variable");
	return -1;
    }

    if (thrd_success != thrd_create(&thr1, func1, nullptr)) {
	printf("%s\n", "could not create thread");
	return -1;
    }

    if (thrd_success != thrd_create(&thr2, func2, nullptr)) {
	printf("%s\n", "could not create thread");
	return -1;
    }

    printf("%s\n", "threads created");

    thrd_join(thr1, &res_code);
    thrd_join(thr2, &res_code);
    cnd_destroy(&cond_var);
    mtx_destroy(&mutex);

    printf("%s\n", "threads joined");    
    
    return 0;
}
