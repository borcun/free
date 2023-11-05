#include <stdlib.h>
#include "miserable.h"

// #define DANGLING_POINTER_TEST
// #define DANGEROUS_MACRO_TEST
// #define PADDING_STRUCT_TEST
#define CALL_BY_REFERENCE_TEST

int main(int argc, const char * argv[]) {
   if (-1 == init()) {
      fprintf(stderr, "%s\n", "Could not initialize the library");
      return -1;
   }

#ifdef DANGLING_POINTER_TEST
   printf("  default username : %s\n", getUsername());

   // existence of dangling pointer
   getUsername()[3] = 'd';

   // correct usage
   // getUsername_s()[0] = 'x';
   
   printf("* username after modification : %s\n", getUsername());
#endif

#ifdef DANGEROUS_MACRO_TEST
   printf("Fault Sum   : MISERABLE_SUM_MACRO(3, 5) * 10 = %d\n", MISERABLE_SUM_MACRO(3, 5) * 10);
   printf("Correct Sum : CORRECT_SUM_MACRO(3, 5) * 10   = %d\n", CORRECT_SUM_MACRO(3, 5) * 10);

   printf("Fault Mul   : MISERABLE_MUL_MACRO(2, 3 + 7)  = %d\n", MISERABLE_MUL_MACRO(2, 3 + 7));
   printf("Correct Mul : CORRECT_MUL_MACRO(2, 3 + 7)    = %d\n", CORRECT_MUL_MACRO(2, 3 + 7));

#endif
   
#ifdef PADDING_STRUCT_TEST
   StructWPadding swp1;
   StructWPadding swp2;
   StructWOPadding swop;
   uint8_t data1[7] = {0x01, 0x20, 0x00, 0x00, 0x00, 0xFF, 0x00};
   uint8_t data2[12] = {0x01, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};
   uint8_t data3[7] = {0x01, 0x20, 0x00, 0x00, 0x00, 0xFF, 0x00};

   printf("Size of struct with padding : %lu\n", sizeof(StructWPadding));
   printf("Size of struct w/o padding  : %lu\n", sizeof(StructWOPadding));

   puts("\n== Samples ==");
   
   setStructWithPadding(&swp1, data1, 7);
   displayStructWithPadding(swp1);

   puts("");
   
   setStructWithPadding(&swp2, data2, 12);
   displayStructWithPadding(swp2);

   puts("");
   
   setStructWithoutPadding(&swop, data3, 7);
   displayStructWithoutPadding(swop);

#endif
   
#ifdef CALL_BY_REFERENCE_TEST
   int num = 0;
   int *ptr1 = NULL;
   int *ptr2 = NULL;
   
   puts("== Samples ==");
   printf("[Before Pass to Value Function] Num: %d\n", num);
   callByValue(num);
   printf("[After Pass to Value Function] Num: %d\n", num);

   puts("");
   
   printf("[Before Pass to Ref Function] Num: %d\n", num);
   callByRef(&num);
   printf("[After Pass to Ref Function] Num: %d\n", num);

   puts("\n==================================================\n");

   printf("[Before Pass] ptr1 address in %s: %p\n", __FUNCTION__, ptr1);
   memAllocP(ptr1);
   printf("[After Pass] ptr1 address in %s: %p\n", __FUNCTION__, ptr1);
   
   // can not reach to any element of the ptr, because of invalid address
   if (NULL != ptr1) {
      printf("First element of ptr1: %d\n", ptr1[0]);
      free(ptr1);
   }
   else {
      fprintf(stderr, "%s\n", "Could not reach ptr1 element");
   }

   puts("");
   
   printf("[Before Pass] ptr2 address in %s: %p\n", __FUNCTION__, ptr2);
   memAllocPP(&ptr2);
   printf("[After Pass] ptr2 address in %s: %p\n", __FUNCTION__, ptr2);
   
   if (NULL != ptr2) {
      printf("First element of ptr2: %d\n", ptr2[0]);
      free(ptr2);
   }
   else {
      fprintf(stderr, "%s\n", "Could not reach ptr2 element");
   }

#endif
   
   deinit();
   
   return 0;
}
