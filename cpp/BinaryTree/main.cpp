#include "btree.hpp"
#include <cstdlib>
#include <ctime>

int main(int argc, const char * argv[]) {
   BinaryTree btree;
   int value = 0;
   int count = 8;
   
   srand((unsigned int) time(NULL));
   
   while (count > 0) {
      value = rand() % 20;
      
      if (btree.insert(value)) {
         std::cout << value << " is inserted" << std::endl;
      }
      else {
         std::cout << "Could not inserted " << value << std::endl;
      }
      
      --count;
   }
   
   std::cout << "preorder (root - left - right)" << std::endl;
   btree.traverse(TraverseMethod::TM_PREORDER);

   std::cout << std::endl << "inorder (left - root - right) " << std::endl;
   btree.traverse(TraverseMethod::TM_INORDER);

   std::cout << std::endl << "postorder (left - right - root) " << std::endl;
   btree.traverse(TraverseMethod::TM_POSTORDER);

   return 0;
}
