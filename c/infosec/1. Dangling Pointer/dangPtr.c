#include <stdlib.h>
#include <string.h>
#include "dangPtr.h"

/*
 * == ATTENTION ==
 * this is my private variable, you see static keyword
 * therefore my intention is hiding it to be updated by anybody
 */
static char *_username;


int init(void) {
   _username = (char *) calloc(DEF_UNAME_LEN, sizeof(char));
   
   if (NULL == _username) {
      return -1;
   }

   // I put default user name during initialization phase
   memcpy(_username, DEF_UNAME, strlen(DEF_UNAME));
   
   return 0;
}

void deinit(void) {
   if (NULL != _username) {
      free(_username);
   }
   
   return;
}

char *getUsername(void) {
   return _username;
}

const char * const getUsername_s(void) {
   return _username;
}
