/**
 * @file psc.h
 * @brief
 * @author boo
 */

#ifndef PSC_H
#define PSC_H

#include "fobject.h"

class PSC : public FObject {
public:
	PSC(const std::string &name);
	virtual ~PSC();
};

#endif
