#include "cbufTest.h"

int cbufTest_init(void) {
   if (0 != cbuf_init()) {
      fprintf(stderr, "[x] %s\n", "Circular Buffer initialization failed");
      return -1;
   }

   fprintf(stdout, "[o] %s\n", "Circular Buffer initialization is done successfully");
   return 0;
}

int cbufTest_transaction(void) {
   int enqElement = 5;
   int deqElement = 0;
   
   if (0 != cbuf_enqueue(enqElement)) {
      fprintf(stderr, "[x] %s\n", "Circular Buffer enqueue failed");
      return -1;
   }
   
   fprintf(stdout, "[o] %s\n", "Circular Buffer enqueue is done successfully");
   
   if (0 != cbuf_dequeue(&deqElement)) {
      fprintf(stderr, "[x] %s\n", "Circular Buffer dequeue failed");
      return -1;
   }

   fprintf(stdout, "[o] %s\n", "Circular Buffer dequeue is done successfully");
   
   if (enqElement != deqElement) {
      fprintf(stderr, "[x] %s\n", "Enqueue element and dequeue element are different each other");
   }
   
   fprintf(stdout, "[o] %s\n", "Enqueue element and dequeue element are same");
   
   return 0;
}

int cbufTest_deinit(void) {
   if (0 != cbuf_deinit()) {
      fprintf(stderr, "[x] %s\n", "Circular Buffer de-initialization failed");
      return -1;
   }
   
   fprintf(stdout, "[o] %s\n", "Circular Buffer de-initialization is done successfully");
   return 0;

}
