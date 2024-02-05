#include "topicFactory.h"
#include "rssFeeder.h"
#include "rssReader.h"
#include <thread>

//! subscriber count
#define READER_COUNT  (4)

//! publisher count
#define FEEDER_COUNT  (4)

// feeder simulation function
void feed(RSSFeeder feeder, Message message);


int main(int argc, char **argv) {
  // create a topic by using factory class
  Topic topic = TopicFactory::getInstance()->create();

  // create subscribers
  RSSReader readers[READER_COUNT] = {
    RSSReader("Reader-1"),
    RSSReader("Reader-2"),
    RSSReader("Reader-3"),
    RSSReader("Reader-4")
  };
  
  // create publishers
  RSSFeeder feeders[FEEDER_COUNT] = {
    RSSFeeder("Feeder-1"),
    RSSFeeder("Feeder-2"),
    RSSFeeder("Feeder-3"),
    RSSFeeder("Feeder-4")
  };
  
  // subscribe all subscribers to one topic
  for (int i = 0; i < READER_COUNT; ++i) {
    topic.subscribe(&readers[i]);
  }

  std::cout << "Topic" << std::endl << "-----" << std::endl;
  std::cout << topic << std::endl;

  // set one topic to all publishers
  for (int i = 0; i < FEEDER_COUNT; ++i) {
    feeders[i].setTopic(&topic);
  }

  // create threads for feeders with different messages, then run them
  std::thread threads[FEEDER_COUNT] = {
    std::thread(feed, feeders[0], Message("Message from Feeder-1")),
    std::thread(feed, feeders[1], Message("Message from Feeder-2")),
    std::thread(feed, feeders[2], Message("Message from Feeder-3")),
    std::thread(feed, feeders[3], Message("Message from Feeder-4"))
  };

  for (int i = 0; i < FEEDER_COUNT; ++i) {
    threads[i].join();
  }
  
  return 0;
}
		
void feed(RSSFeeder feeder, Message message) {  
  /// @attention you might increase arbitrary limit value to view concurrency better
  for (int i = 0; i < 4; ++i) {
    feeder.notify(message);
  }
  
  return;
}
