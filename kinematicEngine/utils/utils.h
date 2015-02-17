#ifndef KINEMATICENGINE_UTILS_UTILS_H
#define KINEMATICENGINE_UTILS_UTILS_H

#include <string>

namespace kinematicEngine {
	namespace utils {

		const double pi = 3.1415926535897932384626433832795;

		/**
		 * limits a value to a certain range
		 *
		 * @param _value value to check if it's in range, otherwise it's being clipped
		 * @param _minValue
		 * @param _maxValue
		 *
		 * Note: _minValue needs to be smaller or equal to _maxValue
		 */
		template<typename T>
		T limited(T const& _value, T const& _minValue, T const& _maxValue) {
			assert(_minValue <= _maxValue);
			if (_value < _minValue) {
				return _minValue;
			} else if (_value > _maxValue) {
				return _maxValue;
			}
			return _value;
		}

		// Normalizes an angle in radians
		// to [-pi, pi[
		double normalize(double _angle);
		bool fileExists(std::string const& _fileName);
	}
}

#ifndef UNUSED
	#define UNUSED(stuff) (void)(stuff);
#endif


#endif
