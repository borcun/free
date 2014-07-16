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
int back(List list);

/* function that gets the first element of list */
int front(List list);

/* function that inserts the element into indication position of list */
int insert(List *list, int elem, int pos);

/* function that removes the element which is indicates with position value from list*/
int delete(List *list, int pos);

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
  else if(1 == list.size) {
    free(list.head);
    return;
  }
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
