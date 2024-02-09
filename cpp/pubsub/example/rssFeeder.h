#ifndef RSS_FEEDER_H
#define RSS_FEEDER_H

#include "publisher.h"

class RSSFeeder : public Publisher {
 public:
  RSSFeeder(const std::string &name);
  ~RSSFeeder();
};

#endif
