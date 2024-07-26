#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

pthread_mutex_t rsrc_lock[3];
pthread_cond_t rsrc_add;

void *get_resources_1(void *arg)
{
    //pthread_mutex_lock(&rsrc_lock);
    printf("%s waiting...\n", __FUNCTION__);
    pthread_cond_wait(&rsrc_add, &rsrc_lock[0]);
    printf("%s released...\n", __FUNCTION__);
    //pthread_mutex_unlock(&rsrc_lock);

    return NULL;
}

void *get_resources_2(void *arg)
{
    //pthread_mutex_lock(&rsrc_lock);
    printf("%s waiting...\n", __FUNCTION__);
    pthread_cond_wait(&rsrc_add, &rsrc_lock[1]);
    printf("%s released...\n", __FUNCTION__);
    //pthread_mutex_unlock(&rsrc_lock);

    return NULL;
}

void *get_resources_3(void *arg)
{
    //pthread_mutex_lock(&rsrc_lock);
    printf("%s waiting...\n", __FUNCTION__);
    pthread_cond_wait(&rsrc_add, &rsrc_lock[2]);
    printf("%s released...\n", __FUNCTION__);
    //pthread_mutex_unlock(&rsrc_lock);

    return NULL;
}

void add_resources()
{
    pthread_cond_broadcast(&rsrc_add);
}

int main() {
    pthread_t id[3];
    
    pthread_mutex_init(&rsrc_lock[0], NULL);
    pthread_mutex_init(&rsrc_lock[1], NULL);
    pthread_mutex_init(&rsrc_lock[2], NULL);
    
    pthread_create(&id[0], NULL, get_resources_1, NULL);
    pthread_create(&id[1], NULL, get_resources_2, NULL);
    pthread_create(&id[2], NULL, get_resources_3, NULL);

    sleep(3);
    add_resources();
    
    pthread_join(id[0], NULL);
    pthread_join(id[1], NULL);
    pthread_join(id[2], NULL);
    
    pthread_mutex_destroy(&rsrc_lock[0]);
    pthread_mutex_destroy(&rsrc_lock[1]);
    pthread_mutex_destroy(&rsrc_lock[2]);
    
    return 0;
}
