#include <stdlib.h>
#include <string.h>
#include "miserable.h"

#define DEF_UNAME ((char *) "user")
#define DEF_UNAME_LEN (8)

// this is my private variable, and not want to be updated from anybody
static char *_username;

int init(void) {
   _username = (char *) calloc(DEF_UNAME_LEN, sizeof(char));
   
   if (NULL == _username) {
      return -1;
   }
   
   memcpy(_username, DEF_UNAME, strlen(DEF_UNAME));

   return 0;
}

void deinit(void) {
   if (NULL != _username) {
      free(_username);
   }
   
   return;
}

void setStructWithPadding(StructWPadding *swp, const uint8_t *data, const uint8_t size) {
   memset(swp, 0x00, sizeof(StructWPadding));
   memcpy(swp, data, size);
   
   return;
}

void setStructWithoutPadding(StructWOPadding *swop, const uint8_t *data, const uint8_t size) {
   memset(swop, 0x00, sizeof(StructWOPadding));
   memcpy(swop, data, size);
   
   return;
}

void displayStructWithPadding(const StructWPadding swp) {
   printf("%s\n", "Struct with padding fields:");
   printf(" - id   : %d\n", swp.uuid);
   printf(" - code : %d\n", swp.code);
   printf(" - len  : %d\n", swp.len);
   
   return;
}

void displayStructWithoutPadding(const StructWOPadding swop) {
   printf("%s\n", "Struct w/o padding fields:");
   printf(" - id   : %d\n", swop.uuid);
   printf(" - code : %d\n", swop.code);
   printf(" - len  : %d\n", swop.len);
   
   return;
}

char *getUsername(void) {
   return _username;
}

const char * const getUsername_s(void) {
   return _username;
}

void callByValue(int num) {
   printf("[Before Set] num in %s: %d\n", __FUNCTION__, num);
   num = 10;
   printf("[After Set] num in %s: %d\n", __FUNCTION__, num);
}

void callByRef(int *num) {
   printf("[Before Set] num in %s: %d\n", __FUNCTION__, *num);
   *num = 20;
   printf("[After Set] num in %s: %d\n", __FUNCTION__, *num);
}

void memAllocP(int *ptr) {
   printf("[Before Alloc] ptr address in %s: %p\n", __FUNCTION__, ptr);
   
   ptr = (int *) malloc(sizeof(int) * 4);
   ptr[0] = 100;

   printf("[After Alloc] ptr address in %s: %p\n", __FUNCTION__, ptr);

   return;
}

void memAllocPP(int **ptr) {
   printf("[Before Alloc] ptr address in %s: %p\n", __FUNCTION__, *ptr);
   printf("[Before Alloc] ptr of ptr address in %s: %p\n", __FUNCTION__, ptr);

   *ptr = (int *) malloc(sizeof(int) * 4);
   *ptr[0] = 100;

   printf("[After Alloc] ptr address in %s: %p\n", __FUNCTION__, *ptr);
   printf("[After Alloc] ptr of ptr address in %s: %p\n", __FUNCTION__, ptr);

   return;
}
