/**
 * @file topicFactory.h
 * @brief singleton topic factory class
 * @author B. Orcun Ozkablan
 */

#ifndef TOPIC_FACTORY_H
#define TOPIC_FACTORY_H

#include <map>
#include "topic.h"

/**
 * @class TopicFactory
 */
class TopicFactory {
 public:
  /**
   * @brief function that gets class instance
   * @return TopicFactory pointer
   */
  static TopicFactory *getInstance(void);

  /**
   * @brief destructor
   */
  virtual ~TopicFactory();

  /**
   * @brief function that creates a Topic instance
   * @return Topic instance
   */
  Topic create(void);

  /**
   * @brief function that checks whether Topic instance is into topic list
   * @param [in] topic - Topic reference
   * @return true if the Topic instance is in topic list. Otherwise, return false.
   */
  bool contains(Topic &topic);

 private:
  //! class pointer
  static TopicFactory *instance;
  
  //! topics
  std::map<unsigned int, Topic> m_topics;

  //! default constructor
  TopicFactory(void) = default;
  
  //! copy constructor
  TopicFactory(const TopicFactory &topicFactory) = delete;

  //! copy assignment operator
  TopicFactory &operator=(const TopicFactory &topicFactory) = delete;
};

#endif
