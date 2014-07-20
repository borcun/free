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

// function that prints queue
void print(Queue *queue)
{
  Node *iter;

  if(queue->head == NULL) {
    fprintf(stderr, "The queue is empty");
    return;
  }

  iter = queue->head;

  while(iter != queue->tail->next) {
    printf("%d -> ", iter->elem);
    iter = iter->next;
  }

  printf("NULL\n");

  return;
}

// function that adds element into queue
void push(Queue *queue, int elem)
{
  Node *node;

  if(queue->head == NULL) {
    node = (Node *)malloc(sizeof(Node));
    node->elem = elem;
    node->next = NULL;
    queue->head = node;
    queue->tail = node;
    queue->size++;

    return;
  }

  node = (Node *)malloc(sizeof(Node));
  node->elem = elem;
  node->next = NULL;
  queue->tail->next = node;
  queue->tail = node;
  queue->size++;

  return;
}

// function that removes element from queue
int pop(Queue *queue)
{
  int elem;

  if(queue->head == NULL) {
    fprintf(stderr, "The queue is empty");
    return -1;
  }

  elem = queue->head->elem;
  queue->head = queue->head->next;
  queue->size--;

  return elem;
}

// function that returns first element of queue
int peek(Queue *queue)
{
  if(queue->head == NULL) {
    fprintf(stderr, "The queue is empty");
    return -1;
  }

  return queue->head->elem;
}

// function that gets size of queue
int size(Queue *queue)
{
  return queue->size;
}

// function that checks queue size
int isEmpty(Queue *queue)
{
  return queue->size == 0;
}

// main function
int main()
{
  // head and tail pointer
  Queue *queue = (Queue *)malloc(sizeof(Queue));
  int elem;

  queue->head = NULL;
  queue->tail = NULL;
  queue->size = 0;

  push(queue, 2);
  push(queue, 5);
  push(queue, -1);
  push(queue, 8);
  push(queue, -3);
  
  print(queue);

  elem = pop(queue);
  printf("First elem: %d, Removen elem: %d, Queue size: %d\n", peek(queue), elem, size(queue));
  print(queue);
  elem = pop(queue);
  printf("First elem: %d, Removen elem: %d, Queue size: %d\n", peek(queue), elem, size(queue));
  print(queue);

  return 0;
}
