#ifndef KINEMATICENGINE_UTILS_MOTORIDS_H
#define KINEMATICENGINE_UTILS_MOTORIDS_H

#include <utility>

namespace kinematicEngine {

#define MOTOR_NONE (0xDEADBEEF)

typedef int32_t MotorID;

struct MotorData {
	double position;
	double speed;
};

}

#endif

