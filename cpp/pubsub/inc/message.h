/**
 * @file message.h
 * @brief Message is a class that refers to data shared between publishers and subscribers
 * @author B. Orcun Ozkablan
 */

#ifndef MESSAGE_H
#define MESSAGE_H

#include <iostream>
#include <string>
#include <sys/time.h>

/**
 * @class Message
 */
class Message {
public:
  /**
   * @brief friend function used to print Message instance
   * @param [in|out] os - output stream instance
   * @param [in] message - Message reference
   * @return output stream filled
   */
  friend std::ostream& operator<<(std::ostream &os, const Message &message);

  /**
   * @brief default constructor
   */
  Message(void);
  
  /**
   * @brief constructor
   * @param [in] payload - payload of message
   */
  Message(const std::string &payload);

  /**
   * @brief copy constructor
   * @param [in] message - Message reference
   */
  Message(const Message &message);

  /**
   * @brief move constructor
   * @param [in] message - Message reference
   */
  Message(const Message &&message);

  /**
   * @brief destructor
   */
  virtual ~Message();

  /**
   * @brief function that sets time label of the message
   * @param [in] tv - message time label
   * @return -
   */
  void setTime(const struct timeval &tv);

  /**
   * @brief function that sets payload of the message
   * @param [in] payload - message payload
   * @return -
   */
  void setPayload(const std::string &payload);

  /**
   * @brief function that gets time label of the message
   * @return message time label
   */
  struct timeval getTime(void) const;

  /**
   * @brief function that gets payload of the message
   * @return message payload
   */
  std::string getPayload(void) const;

  /**
   * @brief function that overloads copy assignment operator
   * @param [in] message - Message reference
   * @return lvalue of the copy assignment operator
   */
  Message &operator=(const Message &message);

  /**
   * @brief function that overloads move assignment operator
   * @param [in] message - Message reference
   * @return lvalue of the move assignment operator
   */
  Message &operator=(const Message &&message);

  /**
   * @brief function that overloads equal operator
   * @param [in] message - Message reference
   * @return true if the lvalue and rvalue are same. Otherwise, return false.
   */
  bool operator==(const Message &message);

  /**
   * @brief function that overloads not equal operator
   * @param [in] message - Message reference
   * @return true if the lvalue and rvalue are different. Otherwise, return false.
   */
  bool operator!=(const Message &message);
  
private:
  //! time label
  struct timeval m_tv;
  
  //! payload data
  std::string m_payload;
};

#endif
