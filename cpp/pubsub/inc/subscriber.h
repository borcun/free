/**
 * @file subscriber.h
 * @brief Subscriber is a class that registers a topic to listen any message from publishers
 * @author B. Orcun Ozkablan
 */

#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include "message.h"

class Subscriber {
 public:
  /**
   * @brief constructor
   * @param [in] name - name of subscriber
   */
  Subscriber(const std::string &name);

  /**
   * @brief destructor
   */  
  virtual ~Subscriber();

  /**
   * @brief function that sets name of subscriber
   * @param [in] name - name of subscriber
   * @return -
   */  
  std::string getName(void) const;

  /**
   * @brief function that matches subscriber to this
   * @param [in] subscriber - Subscriber reference
   * @return true if two instance are same. Otherwise, return false.
   */
  bool operator==(const Subscriber &subscriber);

  /**
   * @brief function that matches subscriber to this
   * @param [in] subscriber - Subscriber reference
   * @return true if two instance are different. Otherwise, return false.
   */
  bool operator!=(const Subscriber &subscriber);
  
  /**
   * @brief function that is triggered when message's publisher notify
   * @param [in] message - Message instance send by Publisher inside the same context
   * @return -
   */
  virtual void onReceived(const Message &message) = 0;

 private:
  //! name of subscriber
  std::string m_name;
};

#endif
