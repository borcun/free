//
//  Apple.hpp
//  BuilderPattern
//
//  Created by B. Orçun Özkablan on 27.08.2023.
//

#ifndef Apple_hpp
#define Apple_hpp

#include "PhoneBuilder.hpp"

class AppleBuilder : public PhoneBuilder {
public:
   AppleBuilder();
   virtual ~AppleBuilder() = default;
   virtual void buildScreen() override;
   virtual void buildColor() override;
   virtual void buildCamera() override;
};

#endif /* Apple_hpp */
