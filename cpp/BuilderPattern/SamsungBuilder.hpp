//
//  SamsungBuilder.hpp
//  BuilderPattern
//
//  Created by B. Orçun Özkablan on 27.08.2023.
//

#ifndef SamsungBuilder_hpp
#define SamsungBuilder_hpp

#include "PhoneBuilder.hpp"

class SamsungBuilder : public PhoneBuilder {
public:
   SamsungBuilder();
   virtual ~SamsungBuilder() = default;
   virtual void buildScreen() override;
   virtual void buildColor() override;
   virtual void buildCamera() override;
};

#endif /* SamsungBuilder_hpp */
