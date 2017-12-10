#ifndef FINISH_POINT_H
#define FINISH_POINT_H

#include "GameObject.h"

class FinishPoint : public GameObject {
 public:
  FinishPoint( void );
  virtual ~FinishPoint();

  void information( void );
};

#endif
