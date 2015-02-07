#ifndef UTILS_MOTORIDS_H
#define UTILS_MOTORIDS_H

#include <utility>

#define MOTOR_NONE (0xDEADBEEF)

typedef int32_t MotorID;

typedef struct {
	double position;
	double speed;
} MotorData;

#endif

