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

  // if the list is empty
  if(NULL == list->head) {
    node = (Node *)malloc(sizeof(Node));
    list->head = node;
    node->next = NULL;
    ++list->size;

    return TRUE;
  }

  // go to the end of list
  while(NULL != list->head)
    list->head = list->head->next;

  node = (Node *)malloc(sizeof(Node));
  list->head = node;
  node->next = NULL;
  ++list->size;

  return TRUE;
}

/* function that removes the element from end of list */
int pop_back(List *list);

/* function that adds the element in front of list */
int push_front(List *list, int elem);

/* function that removes the element from front of list */
int pop_front(List *list);

/* function that gets the last element of list */
int back(List list);

/* function that gets the first element of list */
int front(List list);

/* function that inserts the element into indication position of list */
int insert(List *list, int elem, int pos);

/* function that removes the element which is indicates with position value from list*/
int delete(List *list, int pos);

/* function that prints the all list */
void print(List list)
{
  if(0 == list.size) {
    printf("The list is empty.\n");
    return;
  }

  while(NULL != list.head) {
    printf("%d -> ", list.head->elem);
    list.head = list.head->next;
  }

  printf("NULL\n");
  return;
}

/* function that deallocate the list content */
void dealloc(List *list);
