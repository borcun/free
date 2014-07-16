#include "list.h"

int main()
{
  List *list;

  init(list);

  push_back(list, 2);
  push_back(list, 3);
  push_back(list, 4);
  push_back(list, 4);
  push_front(list, 5);
  push_front(list, -7);

  print(*list);
  printf("list size: %d\n", size(*list));

  dealloc(*list);

  return 0;
}
