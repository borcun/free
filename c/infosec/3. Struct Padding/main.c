#include <stdint.h>
#include "structPadding.h"

int main() {
  // three different instances to demo padding effect in struct of C
  StructWPadding swp1;
  StructWPadding swp2;
  StructWOPadding swop;

  // raw data without padding for both kind of struct instances
  uint8_t data1[7] = {0x01, 0x20, 0x00, 0x00, 0x00, 0xFF, 0x00};
  // raw data with padding data for struct with padding, as you can see, there are extra data into the stream
  uint8_t data2[12] = {0x01, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00};

  // let's check size of two types of struct
  printf("Size of struct with padding : %lu\n", sizeof(StructWPadding));
  printf("Size of struct w/o padding  : %lu\n\n", sizeof(StructWOPadding));

  // data without padding for struct with padding
  setStructWithPadding(&swp1, data1, 7);
  displayStructWithPadding(swp1);

  puts("");

  // data without padding for struct without padding
  setStructWithoutPadding(&swop, data1, 7);
  displayStructWithoutPadding(swop);

  puts("");

  // data with padding for struct with padding
  setStructWithPadding(&swp2, data2, 12);
  displayStructWithPadding(swp2);

  return 0;
}
