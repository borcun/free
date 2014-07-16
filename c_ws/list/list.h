#ifndef LIST_H
#define LIST_H

#include <stdio.h>
#include <stdlib.h>

#define TRUE 1
#define FALSE 0

/* node structure */
typedef struct Node
{
  int elem;
  struct Node *next;
} Node;

/* head pointer of list */
typedef struct List
{
  Node *head;
  int size;
} List;


/* function that initializes the list */
void init(List *list);
/* function that add the element in end of list */
int push_back(List *list, int elem);
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
/* function that gets element count of the list */
int size(List *list);
/* function that prints the all list */
void print(List list);
/* function that deallocate the list content */
void dealloc(List *list);

#endif
