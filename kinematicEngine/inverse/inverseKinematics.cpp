/*
 * inverserKinematicsh.cpp
 *
 *  Created on: 25.06.2014
 *      Author: lutz
 */

#include "inverseKinematics.h"

#define DEFAULT_MAX_VALUE_CHANGE 1.

namespace kinematicEngine {

InverseKinematics::InverseKinematics() : m_maxValueChange(DEFAULT_MAX_VALUE_CHANGE)
{
}


InverseKinematics::~InverseKinematics()
{
}

}
