#include "Director.h"

int main(int argc, const char * argv[]) {
   AppleBuilder apple;
   SamsungBuilder samsung;
   Director director(&apple);
   
   std::cout << "Phone:\n" << director.build() << std::endl << std::endl;
   director.setBuilder(&samsung);
   std::cout << "Phone:\n" << director.build() << std::endl;

   return 0;
}
