/**
 * @file psd.h
 * @brief
 * @author boo
 */

#ifndef PSD_H
#define PSD_H

#include "fobject.h"

class PSD : public FObject {
public:
	PSD(const std::string &name);
	virtual ~PSD();
};

#endif
