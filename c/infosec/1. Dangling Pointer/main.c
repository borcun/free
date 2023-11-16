#include <stdlib.h>
#include "dangPtr.h"

int main(int argc, const char * argv[]) {
   if (-1 == init()) {
      fprintf(stderr, "%s\n", "Could not initialize the library");
      return -1;
   }

   printf("%s\n", "The library is initialized, so a default user is created");
   printf("Default username is '\%s\'\n", getUsername());

   // dangling pointer
   getUsername()[4] = 't';

   printf("Default username is updated to \'%s\'\n", getUsername());
   
   deinit();
   
   return 0;
}
