#include <stdlib.h>
#include "refAndValue.h"

void callByValue(int num) {
  printf(" [Before Update] number is %d in function %s\n", num, __FUNCTION__);
  num = 10;
  printf(" [After Update] number is %d in function %s\n", num, __FUNCTION__);

  return;
}

void callByRef(int *num) {
  printf(" [Before Update] number is %d in function %s\n", *num, __FUNCTION__);
  *num = 10;
  printf(" [After Update] number is %d in function %s\n", *num, __FUNCTION__);

  return;
}

void memAllocP(int *ptr) {
  printf(" [Before Allocation] ptr address is %p in function %s\n", ptr, __FUNCTION__);

  // I allocate 4 integers for the pointer, and set its first element 10
  ptr = (int *) malloc(sizeof(int) * 4);
  ptr[0] = 10;

  printf(" [After Allocation] ptr address is %p in function %s\n", ptr, __FUNCTION__);

  return;
}

void memAllocPP(int **ptr) {
  printf(" [Before Allocation] pointer address is %p in function %s\n", *ptr, __FUNCTION__);
  printf(" [Before Allocation] pointer of pointer address is %p in function %s\n", ptr, __FUNCTION__);

  /*
   * I allocate 4 integers for the pointer, and set its first element 10,
   * the difference in allocation process, allocation is done for pointer address via *ptr instead of ptr
   */  
  *ptr = (int *) malloc(sizeof(int) * 4);
  *ptr[0] = 10;

  printf(" [After Allocation] pointer address is %p in function %s\n", *ptr, __FUNCTION__);
  printf(" [After Allocation] pointer of pointer address is %p in function %s\n", *ptr, __FUNCTION__);

  return;
}
