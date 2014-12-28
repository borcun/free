#include "list.h"

/* function that initializes the list */
void init(List *list)
{
  list->head = NULL;
  list->size = 0;
}

/* function that add the element in end of list */
int push_back(List *list, int elem)
{
  Node *node;
  Node *iter;

  // if the list is empty
  if(NULL == list->head) {
    node = (Node *)malloc(sizeof(Node));
    list->head = node;
    node->elem = elem;
    node->next = NULL;
    ++list->size;

    return TRUE;
  }

  iter = list->head;

  // go to the end of list
  while(NULL != iter->next)
    iter = iter->next;

  iter->next = (Node *)malloc(sizeof(Node));
  iter->next->elem = elem;
  iter->next->next = NULL;
  ++list->size;

  return TRUE;
}

/* function that removes the element from end of list */
int pop_back(List *list)
{
  Node *iter;

  if(0 == list->size) {
    fprintf(stderr, "The list is empty.\n");
    return FALSE;
  }
  else if(1 == list->size) {
    iter = list->head;
    free(iter);
    list->head = NULL;
    --list->size;
  }
  else {
    iter = list->head;

    // go to the end of list
    while(NULL != iter->next->next)
      iter = iter->next;

    free(iter->next);
    iter->next = NULL;
    --list->size;
  }

  return TRUE;
}

/* function that adds the element in front of list */
int push_front(List *list, int elem)
{
  Node *node;

  // if the list is empty
  if(NULL == list->head) {
    node = (Node *)malloc(sizeof(Node));
    list->head = node;
    node->elem = elem;
    node->next = NULL;
    ++list->size;

    return TRUE;
  }

  node = (Node *)malloc(sizeof(Node));
  node->elem = elem;
  node->next = list->head;
  list->head = node;
  ++list->size;

  return TRUE;
}

/* function that removes the element from front of list */
int pop_front(List *list)
{
  Node *iter;

  if(0 == list->size) {
    fprintf(stderr, "The list is empty.\n");
    return FALSE;
  }
  else if(1 == list->size) {
    iter = list->head;
    free(iter);
    list->head = NULL;
    --list->size;
  }
  else {
    iter = list->head;
    list->head = iter->next;
    free(iter);
    --list->size;
  }

  return TRUE;
}

/* function that gets the last element of list */
int back(List list)
{
  Node *iter;

  if(0 == list.size || NULL == list.head) {
    fprintf(stderr, "The list is empty.\n");
    return FALSE;
  }

  iter = list.head;

  while(NULL != iter->next)
    iter = iter->next;

  return iter->elem;
}

/* function that gets the first element of list */
int front(List list)
{
  if(0 == list.size) {
    fprintf(stderr, "The list is empty.\n");
    return -1;
  }

  return list.head->elem;
}

/* function that inserts the element into indication position of list */
int insert(List *list, int elem, int pos)
{
  Node *iter;
  Node *new_node;
  int i;

  if((NULL == list->head && 0 != pos) || (pos > list->size) || (pos < 0)) {
    fprintf(stderr, "Invalid Position\n");
    return FALSE;
  }
  // when the element which will be inserted, if the list is empty, call push_front
  else if(NULL == list->head && 0 == pos)
    return push_front(list, elem);
  else {
    iter = list->head;

    for(i=1 ; i < pos - 1 ; ++i)
      iter = iter->next;

    new_node = (Node *)malloc(sizeof(Node));
    new_node->elem = elem;
    new_node->next = iter->next;
    iter->next = new_node;
    ++list->size;

    return TRUE;
  }
}

/* function that removes the element from list */
int delete(List *list, int elem)
{
  Node *iter;
  Node *temp;

  iter = list->head;

  // if the element which will be removed is first element
  if(iter->elem == elem)
    return pop_front(list);

  while(iter != NULL && iter->next->elem != elem)
    iter = iter->next;

  if(NULL == iter)
    return FALSE;

  temp = iter->next;
  iter->next = iter->next->next;
  free(temp);
  --list->size;

  return TRUE;
}

/* function that gets element count of the list */
int size(List list)
{
  return list.size;
}

/* function that prints the all list */
void print(List list)
{
  Node *iter;

  if(0 == list.size) {
    printf("The list is empty.\n");
    return;
  }

  iter = list.head;

  while(NULL != iter) {
    printf("[%d] -> ", iter->elem);
    iter = iter->next;
  }

  printf("[NULL]\n");
  return;
}

/* function that deallocate the list content */
void dealloc(List list)
{
  Node *cur;
  Node *iter;

  if(0 == list.size)
    return;
  else if(1 == list.size)
    free(list.head);
  else {
    cur = list.head;
    iter = cur->next;

    while(NULL != iter) {
      free(cur);
      cur = iter;
      iter = iter->next;
    }
  }

  return;
}
