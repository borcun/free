/**
 * @file publisher.h
 * @brief Publisher is a class that includes a topic to send message to subscribers registered
 * @author B. Orcun Ozkablan
 */

#ifndef PUBLISHER_H
#define PUBLISHER_H

#include "topic.h"

/**
 * @class Publisher
 */
class Publisher {
 public:
  /**
   * @brief constructor
   * @param [in] name - name of publisher
   */
  Publisher(const std::string &name);

  /**
   * @brief copy constructor
   * @param [in] publisher - Publisher reference
   */
  Publisher(const Publisher &publisher);

  /**
   * @brief destructor
   */  
  virtual ~Publisher();

  /**
   * @brief function that gets name of publisher
   * @param [in] name - name of publisher
   * @return -
   */  
  std::string getName(void) const;

  /**
   * @brief function that sets topic of publisher
   * @param [in] topic - topic of publisher
   * @return -
   */  
  void setTopic(Topic *topic);

  /**
   * @brief function that matches publisher to this
   * @param [in] publisher - Publisher reference
   * @return true if two instance are different. Otherwise, return false.
   */
  bool operator==(const Publisher &publisher);

  /**
   * @brief function that matches publisher to this
   * @param [in] publisher - Publisher reference
   * @return true if two instance are different. Otherwise, return false.
   */
  bool operator!=(const Publisher &publisher);

  /**
   * @brief function that notifies its all subscribers with Message object
   * @param [in] message - Message object to be sent to the subscribers
   * @return -
   */
  virtual void notify(Message message) final;
  
 private:
  //! name of publisher
  std::string m_name;

  //! topic of publisher
  Topic *m_topic;
};

#endif
