#include "topicFactory.h"
#include "rssFeeder.h"
#include "rssReader.h"
#include <cassert>

void testTopic() {
  std::cout << "===========" << std::endl;
  std::cout << "Topic Tests" << std::endl;
  std::cout << "===========" << std::endl << std::endl;

  Topic topic1 = TopicFactory::getInstance()->create();
  Topic topic2 = TopicFactory::getInstance()->create();
  Topic topic3(topic2);
  Topic topic4;
  RSSReader reader1("Reader-1");
  RSSReader reader2("Reader-2");

  topic1.subscribe(&reader1);
  topic1.subscribe(&reader2);
  topic2.subscribe(&reader1);
  topic2.subscribe(&reader2);
  topic3.subscribe(&reader1);
  topic3.subscribe(&reader2);
  topic4.subscribe(&reader1);
  topic4.subscribe(&reader2);
  
  std::cout << "Topic 1" << std::endl << "-------" << std::endl << topic1 << std::endl;
  std::cout << "Topic 2" << std::endl << "-------" << std::endl << topic2 << std::endl;
  std::cout << "Topic 3" << std::endl << "-------" << std::endl << topic3 << std::endl;
  std::cout << "Topic 4" << std::endl << "-------" << std::endl << topic4 << std::endl;

  assert(topic1 != topic2);
  std::cout << "Topic 1 is not equal to Topic 2" << std::endl;
  assert(topic2 == topic3);
  std::cout << "Topic 2 is equal to Topic 3" << std::endl;
  assert(TopicFactory::getInstance()->contains(topic1));
  std::cout << "Topic 1 is into topic list" << std::endl;
  assert(TopicFactory::getInstance()->contains(topic3));
  std::cout << "Topic 3 is into topic list" << std::endl;
  assert(!TopicFactory::getInstance()->contains(topic4));
  std::cout << "Topic 4 is not into topic list" << std::endl;
  
  std::cout << "Topic tests are completed successfully" << std::endl << std::endl;
  
  return;
}

void testPublisher() {
  std::cout << "===============" << std::endl;
  std::cout << "Publisher Tests" << std::endl;
  std::cout << "===============" << std::endl << std::endl;

  Topic topic1 = TopicFactory::getInstance()->create();
  Topic topic2 = TopicFactory::getInstance()->create();
  RSSReader reader1("Reader-1");
  RSSReader reader2("Reader-2");  
  RSSFeeder feeder1("Feeder-1");
  RSSFeeder feeder2("Feeder-2");
  RSSFeeder feeder3("Feeder-2");
  RSSFeeder feeder4(feeder2);

  topic1.subscribe(&reader1);
  topic1.subscribe(&reader2);
  topic2.subscribe(&reader1);
  topic2.subscribe(&reader2);

  feeder1.setTopic(&topic1);
  feeder2.setTopic(&topic2);
  feeder3.setTopic(&topic1);
  feeder4.setTopic(&topic1);

  std::cout << "Topic 1" << std::endl << "-------" << std::endl;
  std::cout << "    [Feeders] " << feeder1.getName() << " " << feeder3.getName() << " " << feeder4.getName() << std::endl;
  std::cout << topic1 << std::endl;

  std::cout << "Topic 2" << std::endl << "-------" << std::endl;
  std::cout << "    [Feeders] " << feeder2.getName() << std::endl;
  std::cout << topic2 << std::endl;
  
  assert(feeder1 != feeder2);
  std::cout << "Feeder 1 is not equal to Feeder 2, because their names are different" << std::endl;
  assert(feeder2 != feeder3);
  std::cout << "Feeder 2 is not equal to Feeder 3, because their topics are different" << std::endl;
  assert(feeder3 == feeder4);
  std::cout << "Feeder 3 is equal to Feeder 4" << std::endl;
  
  std::cout << "Publisher tests are completed successfully" << std::endl << std::endl;
  
  return;
}

int main(int argc, char **argv) {
  testTopic();
  testPublisher();

  return 0;
}
