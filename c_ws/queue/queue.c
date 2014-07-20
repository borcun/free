#include "queue.h"

// function that initialize queue
void init(Queue *queue)
{
  queue->head = NULL;
  queue->tail = NULL;
  queue->size = 0;

  return;
}

// function that prints queue
void print(Queue queue)
{
  Node *iter;

  if(queue.head == NULL) {
    printf("[NULL]\n");
    return;
  }

  iter = queue.head;

  // walk on queue until reaching NULL
  while(NULL != iter) {
    printf("[%d] -> ", iter->elem);
    iter = iter->next;
  }

  printf("[NULL]\n");

  return;
}

// function that adds element into queue
void push(Queue *queue, int elem)
{
  Node *node;

  // if queue is empty
  if(queue->head == NULL) {
    node = (Node *)malloc(sizeof(Node));
    node->elem = elem;
    node->next = NULL;
    // both head and tail point to same node
    queue->head = node;
    queue->tail = node;
    queue->size++;

    return;
  }

  node = (Node *)malloc(sizeof(Node));
  node->elem = elem;
  node->next = NULL;
  // just tail pointer change where it point
  // it points last node anymore
  queue->tail->next = node;
  queue->tail = node;
  queue->size++;

  return;
}

// function that removes element from queue
int pop(Queue *queue)
{
  int elem;
  Node *node;

  if(queue->head == NULL) {
    fprintf(stderr, "The queue is empty");
    return -1;
  }

  elem = queue->head->elem;
  // head points to next node anymore
  node = queue->head;
  queue->head = queue->head->next;
  queue->size--;
  free(node);

  return elem;
}

// function that returns first element of queue
int peek(Queue queue)
{
  if(queue.head == NULL) {
    fprintf(stderr, "The queue is empty");
    return -1;
  }

  return queue.head->elem;
}

// function that gets size of queue
int size(Queue queue)
{
  return queue.size;
}

// function that checks queue size
int isEmpty(Queue queue)
{
  return queue.size == 0;
}

// function that deallocates queue
void dealloc(Queue *queue)
{
  Node *iter = queue->head;

  while(NULL != iter) {
    queue->head = queue->head->next;
    free(iter);
    iter = queue->head;
  }

  queue->tail = NULL;
  printf("The queue is deallocated.\n");

  return;
}
