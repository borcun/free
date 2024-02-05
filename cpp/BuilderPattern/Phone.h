#ifndef Phone_h
#define Phone_h

#include <iostream>
#include <string>

class Phone {
   friend std::ostream &operator<<(std::ostream &os, const Phone &phone);
   
public:
   Phone(void);
   Phone(const std::string &name);
   virtual ~Phone();
   void setName(const std::string &name);
   void setScreenSize(const std::string &screenSize);
   void setColor(const std::string &color);
   void setCamera(const std::string &camera);
   
private:
   std::string name;
   std::string screenSize;
   std::string camera;
   std::string color;
};

#endif /* Phone_h */
