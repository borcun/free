#include "queue.h"

// main function
int main()
{
  // head and tail pointer
  Queue *queue = (Queue *)malloc(sizeof(Queue));
  int elem;

  init(queue);
  push(queue, 2);
  push(queue, 5);
  push(queue, -1);
  push(queue, 8);
  push(queue, -3);
  
  print(*queue);

  elem = pop(queue);
  printf("Removen element: %d\nQueue size: %d\nFirst element: %d\n", 
	 elem, size(*queue), peek(*queue));
  print(*queue);

  elem = pop(queue);
  printf("Removen element: %d\nQueue size: %d\nFirst element: %d\n", 
	 elem, size(*queue), peek(*queue));
  print(*queue);

  return 0;
}
