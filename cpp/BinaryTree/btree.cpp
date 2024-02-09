#include "btree.hpp"

BinaryTree::BinaryTree(void) {
   m_root = nullptr;
}

BinaryTree::~BinaryTree() {
   for (Node *node : m_nodeList) {
      delete node;
   }
}

bool BinaryTree::insert(int &value) {
   Node *node = nullptr;
   bool isInserted = false;
   
   if (nullptr == m_root) {
      node = new Node();
      node->data = value;
      node->left = nullptr;
      node->right = nullptr;
      
      m_root = node;
      m_nodeList.push_back(node);
      
      isInserted = true;
   }
   else {
      // we must check whether the value was already inserted into the tree
      if (!search(value)) {
         Node *pseudoRoot = m_root;
         
         node = new Node();
         node->data = value;
         node->left = nullptr;
         node->right = nullptr;
         
         while (!isInserted && nullptr != pseudoRoot) {
            if (pseudoRoot->data > value) {
               if (nullptr == pseudoRoot->left) {
                  pseudoRoot->left = node;
                  m_nodeList.push_back(node);

                  isInserted = true;
               }
               else {
                  pseudoRoot = pseudoRoot->left;
               }
            }
            else {
               if (nullptr == pseudoRoot->right) {
                  pseudoRoot->right = node;
                  m_nodeList.push_back(node);

                  isInserted = true;
               }
               else {
                  pseudoRoot = pseudoRoot->right;
               }
            }
         } // end of while
      }
   }
   
   return isInserted;
}

bool BinaryTree::search(int &value) {
   Node *pseudoRoot = m_root;
   bool isFound = false;
   
   while (!isFound && nullptr != pseudoRoot) {
      if (value == pseudoRoot->data) {
         isFound = true;
      }
      else if (value < pseudoRoot->data) {
         pseudoRoot = pseudoRoot->left;
      }
      else {
         pseudoRoot = pseudoRoot->right;
      }
   }
   
   return isFound;
}

void BinaryTree::traverse(enum TraverseMethod tm) {
   Node *pseudoRoot = m_root;
   
   switch (tm) {
      case TraverseMethod::TM_PREORDER:
         preorderTraverse(pseudoRoot);
         break;
      case TraverseMethod::TM_INORDER:
         inorderTraverse(pseudoRoot);
         break;
      case TraverseMethod::TM_POSTORDER:
         postorderTraverse(pseudoRoot);
         break;
   }
   
   return;
}

void BinaryTree::preorderTraverse(Node *root) {
   if (nullptr == root) {
      return;
   }
   
   // root - left - right
   std::cout << root->data << " ";
   preorderTraverse(root->left);
   preorderTraverse(root->right);
      
   return;
}

void BinaryTree::inorderTraverse(Node *root) {
   if (nullptr == root) {
      return;
   }
   
   // left - root - right
   inorderTraverse(root->left);
   std::cout << root->data << " ";
   inorderTraverse(root->right);
   
   return;
}

void BinaryTree::postorderTraverse(Node *root) {
   if (nullptr == root) {
      return;
   }
   
   // left - right - root
   postorderTraverse(root->left);
   postorderTraverse(root->right);
   std::cout << root->data << " ";

   return;
}
