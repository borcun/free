#ifndef LIST_H
#define LIST_H

#include <iostream>

struct Node {
   int data;
   struct Node *next;
};

class LinkedList {
public:
   LinkedList(void);
   virtual ~LinkedList();
   void push_front(const int &value);
   void push_back(const int &value);
   void pop_front(void);
   void pop_back(void);
   int size(void) const;
   void print(void);
   
private:
   Node *m_head;
   int m_size;
};

#endif
