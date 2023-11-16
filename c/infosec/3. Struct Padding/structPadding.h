#ifndef STRUCT_PADDING_H
#define STRUCT_PADDING_H

#include <stdio.h>

/*
 * == ATTENTION == 
 * the struct includes padding, used to indicate padding problem
 */
typedef struct {
   uint8_t uuid;
   uint32_t code;
   uint16_t len;
} StructWPadding;

/*
 * == ATTENTION ==
 * the struct does not include padding because of alignment
 * there is one more alternative way, it is #pragma pack
 */
typedef struct {
   uint8_t uuid;
   uint32_t code;
   uint16_t len;
} __attribute__((packed, aligned(1))) StructWOPadding;

// functions that are used to set and display structures with and w/o padding
void setStructWithPadding(StructWPadding *swp, const uint8_t *data, const uint8_t size);
void setStructWithoutPadding(StructWOPadding *swop, const uint8_t *data, const uint8_t size);
void displayStructWithPadding(const StructWPadding swp);
void displayStructWithoutPadding(const StructWOPadding swop);

#endif
