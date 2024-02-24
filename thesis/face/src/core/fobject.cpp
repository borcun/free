#include "fobject.h"

uint32_t FObject::m_id = FOBJECT_INITIAL_ID;

FObject::FObject(const std::string &name) {
	m_id++;
	m_name = name;
}

FObject::~FObject() {

}

uint32_t FObject::getId() const {
	return m_id;
}

std::string FObject::getName() const {
	return m_name;
}
