/*
 * Description : Robot Arm Class
 * Author	   : Burak Orcun OZKABLAN
 * Date		   : 14.06.2012
 * Hint		   : destructor of osgViewer::View is protected !
 */

#ifndef ROBOTARM_H
#define ROBOTARM_H

#include <iostream>
#include <vector>
#include <osgViewer/CompositeViewer>

using namespace std;
using namespace osgViewer;

// RobotArm Class
class RobotArm : public osgViewer::CompositeViewer
{
public:
	// constructors
	RobotArm();
	// destructor
	virtual ~RobotArm();

}; // end of RobotArm Class

#endif