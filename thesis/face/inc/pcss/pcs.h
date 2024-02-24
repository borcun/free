/**
 * @file pcs.h
 * @brief
 * @author boo
 */

#ifndef PCS_H
#define PCS_H

#include "fobject.h"

class PCS : public FObject {
public:
	PCS(const std::string &name);
	virtual ~PCS();
};

#endif
