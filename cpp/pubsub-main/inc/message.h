/**
 * @file message.h
 * @brief
 */

#ifndef MESSAGE_H
#define MESSAGE_H

#include <iostream>
#include <string>
#include <sys/time.h>

class Message {
 public:
  friend std::ostream& operator<<(std::ostream &os, const Message &message);
  
  explicit Message(const std::string &topic);
  Message(const Message &topic);
  Message(const Message &&topic);
  virtual ~Message();
  void setPayload(const std::string &payload);
  std::string getTopic(void) const;
  struct timeval getTime(void) const;
  std::string getPayload(void) const;
  Message &operator=(const Message &message);
  Message &operator=(const Message &&message);
  bool operator==(const Message &message);
  bool operator!=(const Message &message);
  
 private:
  std::string m_topic;
  struct timeval m_time;
  std::string m_payload;
};

#endif
