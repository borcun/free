#include "list.h"

int main()
{
  List *list;

  init(list);

  push_back(list, 4);

  print(*list);
  return 0;
}
