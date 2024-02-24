/**
 * @file fobject.h
 * @brief
 * @author boo
 */

#ifndef FOBJECT_H
#define FOBJECT_H

#include <cstdint>
#include <string>

//! initial fobject id
#define FOBJECT_INITIAL_ID (1)

/**
 * @class FObject that is base class for each service base class
 */
class FObject {
public:
	/**
	 * @brief constructor
	 */
	FObject(const std::string &name);

	/**
	 * @brief destructor
	 */
	virtual ~FObject();

	/**
	 * @brief function that gets module id
	 * @return module id
	 */
	uint32_t getId() const;

	/**
	 * @brief function that gets module name
	 * @return module name
	 */
	std::string getName() const;

private:
	//! unique module id
	static uint32_t m_id;

	//! module name
	std::string m_name;

	/**
	 * for non-copyable object
	 */
	FObject(const FObject &obj) {}

};


#endif
