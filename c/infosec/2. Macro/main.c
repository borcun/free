#include "macro.h"

int main() {
   printf("Fault Sum   : MISERABLE_SUM_MACRO(3, 5) * 10 = %d\n", MISERABLE_SUM_MACRO(3, 5) * 10);
   printf("Correct Sum : CORRECT_SUM_MACRO(3, 5) * 10   = %d\n", CORRECT_SUM_MACRO(3, 5) * 10);

   printf("Fault Mul   : MISERABLE_MUL_MACRO(2, 3 + 7)  = %d\n", MISERABLE_MUL_MACRO(2, 3 + 7));
   printf("Correct Mul : CORRECT_MUL_MACRO(2, 3 + 7)    = %d\n", CORRECT_MUL_MACRO(2, 3 + 7));
  
  return 0;
}
