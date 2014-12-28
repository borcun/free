#include <stdio.h>
#include <stdlib.h>

// queue node structure
typedef struct Node
{
  int elem;
  struct Node *next;
} Node;

// queue structure
typedef struct Queue
{
  Node *head;
  Node *tail;
  int size;
} Queue;

// function that initialize queue
void init(Queue *);
// function that prints queue
void print(Queue);
// function that pushs element to queue
void push(Queue *, int);
// function that pops element from queue
int pop(Queue *);
// function that gets first element of queue
int peek(Queue);
// function that gets size of queue
int size(Queue);
// function that checks whether queue is empty
int empty(Queue);
// function that dellocate queue
void dealloc(Queue *);
