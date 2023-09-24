/**
 * @file main.cpp
 * @brief OSG uses a smart pointer called osg::ref_ptr similar to smart pointer defined in C++
 *        It has similar functions such as get, release. Moreover, there is another important 
 *        class called osg::Referenced, which makes any class derived reference, so the class
 *        derived can be used as a type for osg::ref_ptr. The osg::Referenced class has three
 *        functions ref, unref and referenceCount that are used to increase, decrease and count
 *        available references of current instance respectively. Critically, if there is more
 *        osg::ref_ptr instances points to a location already pointed from another osg::ref_ptr
 *        instances, their referenced counts are bigger than one. All memory management operations
 *        are done automatically by OSG internal algorithms.
 * @author boo
 */

#include <iostream>
#include <osg/ref_ptr>
#include <osg/Referenced>

class MonitorTarget : public osg::Referenced {
public:
  MonitorTarget(int id) : m_id(id) {
    std::cout << "constructing target " << m_id << std::endl;
  }

protected:
  virtual ~MonitorTarget() {
    std::cout << "destruction target " << m_id << std::endl;
  }

  int m_id = 0;
};

MonitorTarget *createMonitoringTarget(int id) {
  osg::ref_ptr<MonitorTarget> target = new MonitorTarget(id);

  return target.release();
}

int main(int argc, char **argv) {
  osg::ref_ptr<MonitorTarget> target1 = createMonitoringTarget(20); // new MonitorTarget(10);

  std::cout << "referenced count by target 1 before referring: " << target1->referenceCount() << std::endl;

  osg::ref_ptr<MonitorTarget> target2 = target1;

  std::cout << "referenced count by target 2: " << target2->referenceCount() << std::endl;
  std::cout << "referenced count by target 1 after referring: " << target1->referenceCount() << std::endl;

  return 0;
}
