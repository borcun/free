#include "list.h"

int main()
{
  List *list = (List *)malloc(sizeof(List));

  init(list);
  print(*list);
  printf("list size: %d\n", size(*list));

  push_back(list, 2);
  print(*list);
  printf("list size: %d\n", size(*list));

  push_back(list, 3);
  print(*list);
  printf("list size: %d\n", size(*list));

  push_back(list, 4);
  print(*list);
  printf("list size: %d\n", size(*list));

  printf("Front: %d\n", front(*list));
  printf("Back: %d\n", back(*list));

  push_back(list, 4);
  print(*list);
  printf("list size: %d\n", size(*list));

  printf("Front: %d\n", front(*list));
  printf("Back: %d\n", back(*list));

  push_front(list, 5);
  print(*list);
  printf("list size: %d\n", size(*list));

  printf("Front: %d\n", front(*list));

  push_front(list, -7);
  print(*list);
  printf("list size: %d\n", size(*list));

  printf("Front: %d\n", front(*list));
  printf("Back: %d\n", back(*list));

  pop_back(list);
  print(*list);
  printf("list size: %d\n", size(*list));

  delete(list, 5);
  print(*list);
  printf("list size: %d\n", size(*list));

  insert(list, -9, 3);
  print(*list);
  printf("list size: %d\n", size(*list));

  insert(list, 6, 5);
  print(*list);
  printf("list size: %d\n", size(*list));

  pop_back(list);
  print(*list);
  printf("list size: %d\n", size(*list));

  printf("Back: %d\n", back(*list));

  pop_front(list);
  print(*list);
  printf("list size: %d\n", size(*list));

  pop_front(list);
  print(*list);
  printf("list size: %d\n", size(*list));

  printf("Back: %d\n", back(*list));

  pop_back(list);
  print(*list);
  printf("list size: %d\n", size(*list));

  printf("Back: %d\n", back(*list));

  pop_back(list);
  print(*list);
  printf("list size: %d\n", size(*list));

  dealloc(*list);

  return 0;
}
