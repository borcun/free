#include <stdlib.h>
#include "refAndValue.h"

int main() {
  int num = 0;
  int *ptr1 = NULL;
  int *ptr2 = NULL;

  // call by value
  printf("%s\n", "Call by Value Function for Primitive Type");
  printf(" [Before Pass] number is %d\n", num);
  callByValue(num);
  printf(" [After Pass] number is %d\n\n", num);

  // call by reference by modifying function parameter as pointer
  printf("%s\n", "Call by Reference Function for Primitive Type");
  printf(" [Before Pass] number is %d\n", num);
  callByRef(&num);
  printf(" [After Pass] number is %d\n\n", num);

  /*
   * I pass pointer to the function to allocate memory for it
   * but it is passed by value too although the function gets pointer, means call by reference
   */
  printf("%s\n", "Memory Allocation Function for One Degree Pointer");
  printf(" [Before Pass] pointer address is %p in function %s\n", ptr1, __FUNCTION__);
  memAllocP(ptr1);
  printf(" [After Pass] pointer address is %p in function %s\n", ptr1, __FUNCTION__);
   
  // can not reach to any element of the ptr, because of invalid address
  if (NULL != ptr1) {
    printf(" The first element of pointer is %d\n\n", ptr1[0]);
    free(ptr1);
  }
  else {
    fprintf(stderr, " * %s\n\n", "Could not reach ptr1 element");
  }


  /*
   * I pass address of pointer to the function, because I want to allocate memory
   * for its address instead of location it points
   */
  printf("%s\n", "Memory Allocation Function for Double Degree Pointer");
  printf(" [Before Pass] pointer address is %p in function %s\n", ptr2, __FUNCTION__);
  memAllocPP(&ptr2);
  printf(" [After Pass] pointer address is %p in function %p\n", ptr2, __FUNCTION__);
   
  if (NULL != ptr2) {
    printf(" The first element of pointer is %d\n", ptr2[0]);
    free(ptr2);
  }
  else {
    fprintf(stderr, " * %s\n", "Could not reach ptr2 element");
  }

  return 0;
}
