#ifndef PhoneBuilder_hpp
#define PhoneBuilder_hpp

#include "Phone.h"

class PhoneBuilder {
public:
   PhoneBuilder() = default;
   virtual ~PhoneBuilder() = default;
   virtual void buildColor() = 0;
   virtual void buildScreen() = 0;
   virtual void buildCamera() = 0;
   Phone getPhone() { return phone; }
   
protected:
   Phone phone;
};

#endif /* PhoneBuilder_hpp */
