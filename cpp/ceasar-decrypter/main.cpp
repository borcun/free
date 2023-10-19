/**
 * @file ceasar-decrypter.cpp
 * @brief function that decryptes cipher text via ceasar cipher method with encrypted unknown  shift amount
 * @author boo
 */

#include <iostream>
#include <stdint.h>

#define SPACE_ASCII_CODE  (32)  // ignore space character
#define ASCII_BEGIN_INDEX (65U)  // upper A
#define ASCII_END_INDEX   (90U)  // upper Z

/**
 * @brief function that encryptes cipher text to plain text
 * @param [in] cipher_text - cipher text
 * @param [in] size - size of set which includes all characters
 * @param [in] shift - shift count
 * @return plain text if cipher text is valid. otherwise, return empty text.
 */
std::string decrypt(const std::string &cipher_text, const uint8_t &size, const uint8_t &shift) {
   std::string plain_text;
   bool is_invalid_char = false;
   
   for (int i = 0; i < cipher_text.length() && !is_invalid_char; ++i) {
      // do not care case sensitive, give output uppercase
      int cipher_char = toupper(cipher_text.at(i));
      
      // ignore space character
      if (SPACE_ASCII_CODE == cipher_char) {
         plain_text.push_back(cipher_char);
      }
      else if (ASCII_BEGIN_INDEX <= cipher_char && ASCII_END_INDEX >= cipher_char) {
         // shift cipher char, and make a circular shift if it is bigger than maximum limit
         int shifted = (cipher_char + shift);
         int tmp = shifted > ASCII_END_INDEX ? (shifted + ASCII_BEGIN_INDEX) % (ASCII_END_INDEX + 1) : shifted;
         
         plain_text.push_back((char) tmp);
      }
      else {
         plain_text.clear();
         is_invalid_char = true;
      }
   }
   
   return plain_text;
}

int main(int argc, const char * argv[]) {
   std::string cipher = "QEB NRFZH YOLTK CLU GRJMP LSBO QEB IXWV ALD";
   uint32_t set_size = ASCII_END_INDEX - ASCII_BEGIN_INDEX + 1;
   
   // optional case to get input from command line
   if (2 == argc) {
      cipher = argv[1];
   }
   
   std::cout << "cipher : " << cipher << std::endl;
   
   for (int i = 0; i < set_size; ++i) {
      std::cout << "plain  : " << decrypt(cipher, set_size, i) << " (k: " << i << ")" << std::endl;
   }
   
   return 0;
}
