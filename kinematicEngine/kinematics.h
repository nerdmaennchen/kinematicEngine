/*
 * kinematics.h
 *
 *  Created on: 02.03.2015
 *      Author: lutz
 */

#ifndef KINEMATICENGINE_KINEMATICS_H_
#define KINEMATICENGINE_KINEMATICS_H_

#include <map>
#include <vector>
#include <utility>
#include <armadillo>

namespace kinematicEngine {

#define MOTOR_NONE (0xDEADBEEF)

typedef int32_t MotorID;
typedef int32_t SensorID;

struct MotorData {
	double position;
	double speed;
};

namespace kinematics {
	typedef uint NodeID;

	typedef std::map<MotorID, double> MotorValuesMap;
	typedef std::pair<MotorID, double> MotorValuesMapEntry;

	typedef std::vector<MotorID> MotorIDs;

	typedef std::pair<MotorID, uint> Motor2Int;
	typedef std::map<MotorID, uint> Motor2IntMap;

	typedef std::pair<uint, MotorID> Int2Motor;
	typedef std::map<uint, MotorID> Int2MotorMap;

	typedef std::pair<uint, arma::colvec3> JacobianValue;
	typedef std::vector<JacobianValue> JacobianValues;
}

}


#endif /* KINEMATICENGINE_KINEMATICS_H_ */
