#ifndef STACK_H
#define STACK_H

//! default stack capacity
#define STACK_DEF_CAP (10U)

//! invalid top index
#define STACK_INV_TOP (-1)

template <class T>
class Stack {
 public:
  /**
   * @brief default constructor
   * @note stack is initialized with default capacity value
   */
  Stack<T>(void);

  /**
   * @brief constructor that takes capacity
   * @param [in] cap - stack capacity
   */
  explicit Stack<T>(unsigned int cap);

  /**
   * @brief destructor
   */
  virtual ~Stack();

  /**
   * @brief function that pushes element into the stack
   * @param [in] elem - element to be pushed
   * @return -
   */
  void push(const T elem);

  /**
   * @brief function that pops element from the stack
   * @return -
   */
  void pop(void);

  /**
   * @brief function that gets top element of the stack
   * @return reference of top element
   */
  T *top(void);

  /**
   * @brief function that gets stack size
   * @return size of stack
   */
  int size(void) const;
  
 private:
  //! stack content
  T *m_data;
  
  //! index of top element into the stack
  int m_top;
  
  //! stack capacity
  unsigned int m_cap;
};

#endif
