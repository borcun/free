#include "list.hpp"

LinkedList::LinkedList(void) {
   m_head = nullptr;
   m_size = 0;
}

LinkedList::~LinkedList() {
   Node *tmp = m_head;
   
   // delete all the elements until the head points nullptr
   while (nullptr != m_head) {
      m_head = m_head->next;
      delete tmp;
      tmp = m_head;
   }
   
   m_size = 0;
}

void LinkedList::push_front(const int &value) {
   Node *node = new Node();
   
   node->data = value;

   if (nullptr == m_head) {
      node->next = nullptr;
   }
   else {
      node->next = m_head;
   }

   m_head = node;
   ++m_size;

   return;
}

void LinkedList::push_back(const int &value) {
   Node *node = new Node();
   
   node->data = value;
   node->next = nullptr;

   if (nullptr == m_head) {
      m_head = node;
   }
   else {
      Node *tmp = m_head;
            
      while (nullptr != tmp->next) {
         tmp = tmp->next;
      }
      
      tmp->next = node;
   }

   ++m_size;
   
   return;
}

void LinkedList::pop_front(void) {
   Node *node = m_head;
   
   if (nullptr == m_head) {
      return;
   }
   
   m_head = m_head->next;
   --m_size;
   
   delete node;
}

void LinkedList::pop_back(void) {
   Node *former = m_head;
   Node *latter = nullptr;
   
   if (nullptr == m_head) {
      return;
   }
   
   latter = m_head->next;
   
   // if next of head is nullptr, it means that the list includes one element only
   if (nullptr == latter) {
      delete former;
      m_head = nullptr;
   }
   else {
      while (nullptr != latter->next) {
         former = latter;
         latter = latter->next;
      }
      
      delete latter;
      former->next = nullptr;
   }
   
   --m_size;
   
   return;
}

int LinkedList::size(void) const {
   return m_size;
}

void LinkedList::print(void) {
   Node *begin = m_head;
   
   while (nullptr != begin) {
      std::cout << begin->data << " -> ";
      begin = begin->next;
   }
   
   std::cout << "nullptr" << std::endl;
   
   return;
}
