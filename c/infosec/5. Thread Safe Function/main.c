#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>

// thread function that is used to exploit non-thread safety
void *parse(void *arg) {
  char *str = (char *) arg;
  char *word = NULL;

  /*
   * sleep time to postpone thread function processing, because I wanna show main
   * function processing firstly without any conflict.
   *
   * additionally, I intentionally use prime numbers to delay functions processings,
   * they seems providing undetermined timing at the first glance
   */
  usleep(47);
  
  word = strtok(str, " ");
  
  while (NULL != word) {
    printf("\'%s\' is parsed in %s function\n", word, __FUNCTION__);
    usleep(19);
    word = strtok(NULL, " ");
  }

  return NULL;
}

int main() {
  char str1[] = "Hello cruel world, how do you do?";
  // do you remember the below text from the first lecture? the ceaser cipher :)
  char str2[] = "The quick brown fox jumps over the lazy dog";
  char *word = NULL;
  pthread_t tid;

  printf("The \'%s\' string is passed to thread function\n", str1);
  printf("The \'%s\' string is processed into main function\n\n", str2);

  // when the thread is created, its processing also starts at the moment
  pthread_create(&tid, NULL, parse, str1);
  
  word = strtok(str2, " ");
  
  while (NULL != word) {
    printf("\'%s\' is parsed in %s function\n", word, __FUNCTION__);
    usleep(17);
    word = strtok(NULL, " ");
  }

  pthread_join(tid, NULL);
  
  return 0;
}
