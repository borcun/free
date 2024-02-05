#include "topicFactory.h"

TopicFactory *TopicFactory::instance = nullptr;

TopicFactory::~TopicFactory() {
  if (nullptr != instance) {
    delete instance;
  }

  m_topics.clear();
}

TopicFactory *TopicFactory::getInstance(void) {
  if (nullptr == instance) {
    instance = new TopicFactory();
  }

  return instance;
}

Topic TopicFactory::create(void) {
  Topic topic;
  
  m_topics.insert({topic.getId(), topic});

  return topic;
}

bool TopicFactory::contains(Topic &topic) {
  return m_topics.end() != m_topics.find(topic.getId());
}
