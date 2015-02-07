#include "utils.h"

#include <cstdio>

namespace utils {

	/**
	 * Normalization of an angle into [-pi to +pi[
	 * (excluding +pi)
	 *
	 *
	 * @param _angle value to normalize
	 */
	Radian normalize(Radian _angle) {
		if (_angle < pi * radians && _angle >= -pi * radians)
			return _angle;

		/* from -2pi to 2*pi */
		double normalizedAngle = fmod(_angle.value(), 2.*pi);

		/* from -pi to pi */
		if (normalizedAngle >= pi) {
			normalizedAngle = normalizedAngle - 2.*pi;
		} else if (normalizedAngle < -pi)
			normalizedAngle = normalizedAngle + 2.*pi;

		return normalizedAngle * radians;
	}

	/*
	 * checks if file exists
	 */
	bool fileExists(std::string const& _fileName) {
		FILE *file = fopen(_fileName.c_str(), "r");
		if (nullptr == file)
			return (errno != ENOENT);

		fclose(file);
		return true;
	}

	Second getCurrentTime() {
		auto now = std::chrono::steady_clock::now().time_since_epoch();
		double timeSinceEpoch = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
		return timeSinceEpoch / 1000000000 * seconds;
	}

	void delay(Second s) {
		std::chrono::milliseconds duration(int(s.value() * 1000.));
		std::this_thread::sleep_for(duration);
	}

}
