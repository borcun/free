/**
 * @file tss.h
 * @brief
 * @author boo
 */

#ifndef TSS_H
#define TSS_H

#include "fobject.h"

class TSS : public FObject {
public:
	TSS(const std::string &name);
	virtual ~TSS();
};

#endif
