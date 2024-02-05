#include <iostream>
#include "stack.h"

template <class T>
Stack<T>::Stack(void) : m_top(STACK_INV_TOP) {
  m_cap = STACK_DEF_CAP;
  m_data = new T[m_cap];
}

template <class T>
Stack<T>::Stack(unsigned int cap) : m_top(STACK_INV_TOP) {
  if (0 == cap) {
    m_cap = STACK_DEF_CAP;
  }
  else {
    m_cap = cap;
  }

  m_data = new T[m_cap];
}

template <class T>
Stack<T>::~Stack() {
  if (nullptr != m_data) {
    delete m_data;
    m_data = nullptr;
  }
}

template <class T>
void Stack<T>::push(const T elem) {
  if (m_top + 1 < m_cap) {
    ++m_top;
    m_data[m_top] = elem;
  }

  return;
}

template <class T>
void Stack<T>::pop(void) {
  if (m_top > STACK_INV_TOP) {
    --m_top;
  }
}

template <class T>
T *Stack<T>::top(void) {
  if (m_top > STACK_INV_TOP) {
    return &m_data[m_top];
  }

  return nullptr;
}

template <class T>
int Stack<T>::size(void) const {
  return m_top + 1;
}
