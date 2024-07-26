#include <pthread.h>
#include <stdio.h>
#include <unistd.h> // for sleep

// Thread function
void* thread_func(void* arg) {
    // Set cancellation state to enable cancellation
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);

    // Set cancellation type to asynchronous cancellation
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

    // Perform some blocking operation
    while (1) {
        // Simulate a blocking operation
        sleep(1);
        printf("Thread running...\n");
    }

    //pthread_cleanup_pop(1); // Remove cleanup handler
    pthread_exit(NULL);     // Terminate thread

    return NULL;
}

int main() {
    pthread_t thread;

    // Create the thread
    if (pthread_create(&thread, NULL, thread_func, NULL)) {
        fprintf(stderr, "Error creating thread\n");
        return 1;
    }

    // Wait for a while
    sleep(3);

    // Cancel the thread
    if (pthread_cancel(thread)) {
        fprintf(stderr, "Error canceling thread\n");
        return 1;
    }

    // Wait for the thread to terminate
    if (pthread_join(thread, NULL)) {
        fprintf(stderr, "Error joining thread\n");
        return 1;
    }

    printf("Thread terminated\n");

    return 0;
}
