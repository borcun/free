/**
 * @file topic.h
 * @brief Topic is a class that acts role as context providing communication between publishers and subscribers
 * @author B. Orcun Ozkablan
 */

#ifndef TOPIC_H
#define TOPIC_H

#include <list>
#include <mutex>
#include <shared_mutex>
#include "subscriber.h"

/**
 * @class Topic
 */
class Topic {
public:
  /**
   * @brief friend function used to print Topic instance
   * @param [in|out] os - output stream instance
   * @param [in] topic - Topic reference
   * @return output stream filled
   */
  friend std::ostream& operator<<(std::ostream &os, const Topic &topic);

  /**
   * @brief default constructor
   */
  Topic(void);

  /**
   * @brief copy constructor
   * @param [in] topic - Topic reference
   */
  Topic(const Topic &topic);

  /**
   * @brief destructor
   */
  virtual ~Topic();

  /**
   * @brief function that gets topic id
   * @return topic id
   * @remark topic id is unique for each topic within process
   */
  virtual unsigned int getId(void) final;

  /**
   * @brief function that registers a subscriber to the topic
   * @param [in] subscsriber - Subscriber pointer
   * @return true if the subscribers is registered into the list. Otherwise, return false.
   */
  bool subscribe(Subscriber *subscriber);

  /**
   * @brief function that removes subscriber from the topic
   * @param [in] subscsriber - Subscriber pointer
   * @return true if the subscribers is removed from the list. Otherwise, return false.
   */
  bool unsubscribe(Subscriber *subscriber);

  /**
   * @brief function that publishes message to its all subscribers registered into the topic
   * @param [in] message - Message reference to be transferred within the topic
   * @return -
   */
  void publish(const Message &message);

  /**
   * @brief function that overloads equality operator
   * @param [in] topic - Topic reference
   * @return true if the topic is equal to this reference. Otherwise, return false.
   */
  bool operator==(const Topic &topic);

  /**
   * @brief function that overloads non-equality operator
   * @param [in] topic - Topic reference
   * @return true if the topic is not equal to this reference. Otherwise, return false.
   */
  bool operator!=(const Topic &topic);

private:  
  //! subscriber list
  std::list<Subscriber *> m_subscribers;

  //! unique topic id
  unsigned int m_id;

  //! mutex for publishing functionality
  mutable std::shared_mutex m_mutex;

  //! topic id counter
  static unsigned int count;
};

#endif
