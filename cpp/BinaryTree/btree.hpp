#ifndef BTREE_H
#define BTREE_H

#include <iostream>
#include <vector>

enum class TraverseMethod {
   TM_PREORDER  = 1,
   TM_INORDER   = 2,
   TM_POSTORDER = 3
};

struct Node {
   // data of the node
   int data;
   // pointer to left child
   Node *left;
   // pointer to right child
   Node *right;
};

class BinaryTree {
public:
   // default constructor
   BinaryTree(void);
   // destructor
   virtual ~BinaryTree();
   // function that inserts value into the tree
   bool insert(int &value);
   // function that search value into the tree
   bool search(int &value);
   // function that traverses the tree
   void traverse(enum TraverseMethod tm);
   
private:
   // root of the tree
   Node *m_root;
   // node list
   std::vector<Node *> m_nodeList;
   
   // function that traverse the tree pre-orderly
   void preorderTraverse(Node *root);
   // function that traverse the tree inorderly
   void inorderTraverse(Node *root);
   // function that traverse the tree post-orderly
   void postorderTraverse(Node *root);
};

#endif
