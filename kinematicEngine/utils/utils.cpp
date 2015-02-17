#include "utils.h"

#include <cmath>
#include <cstdio>

namespace kinematicEngine {
	namespace utils {

		// Normalizes an angle in radians
		// to [-pi, pi[
		double normalize(double _angle) {
			if (_angle < pi && _angle >= -pi)
				return _angle;

			/* from -2pi to 2*pi */
			double normalizedAngle = fmod(_angle, 2.*pi);

			/* from -pi to pi */
			if (normalizedAngle >= pi) {
				normalizedAngle = normalizedAngle - 2.*pi;
			} else if (normalizedAngle < -pi)
				normalizedAngle = normalizedAngle + 2.*pi;

			return normalizedAngle;
		}

		bool fileExists(std::string const& _fileName) {
			FILE *file = fopen(_fileName.c_str(), "r");
			if (nullptr == file)
				return (errno != ENOENT);

			fclose(file);
			return true;
		}



	}
}
