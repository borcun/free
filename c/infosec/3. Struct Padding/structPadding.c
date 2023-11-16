#include <stdlib.h>
#include <string.h>
#include "structPadding.h"

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
