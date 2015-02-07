#ifndef UTILS_UNITS_H
#define UTILS_UNITS_H

#include <boost/units/cmath.hpp>
#include <boost/units/unit.hpp>
#include <boost/units/make_scaled_unit.hpp>
#include <boost/units/quantity.hpp>
#include <boost/units/systems/si/frequency.hpp>
#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <boost/units/systems/si/time.hpp>
#include <boost/units/systems/angle/degrees.hpp>


// Define Meter and meters
typedef boost::units::quantity<boost::units::si::length, double> Meter;
using boost::units::si::meters;

// Define Centimeter and centimeters
typedef boost::units::make_scaled_unit<
        boost::units::si::length, boost::units::scale<10, boost::units::static_rational<-2> >
        >::type _centimeter;
BOOST_UNITS_STATIC_CONSTANT(centimeters, _centimeter);
typedef boost::units::quantity<_centimeter, double> Centimeter;

// Define Millimeter and millimeters
typedef boost::units::make_scaled_unit<
        boost::units::si::length, boost::units::scale<10, boost::units::static_rational<-3> >
        >::type _millimeter;
BOOST_UNITS_STATIC_CONSTANT(millimeters, _millimeter);
typedef boost::units::quantity<_millimeter, double> Millimeter;

// Define Radian and radians
typedef boost::units::quantity<boost::units::si::plane_angle> Radian;
using boost::units::si::radians;

// Define Degree and degrees
typedef boost::units::quantity<boost::units::degree::plane_angle> Degree;
using boost::units::degree::degrees;

// Define Second and seconds
typedef boost::units::quantity<boost::units::si::time, double> Second;
using boost::units::si::seconds;

// Define Minute and minutes
typedef boost::units::make_scaled_unit<
		boost::units::si::time, boost::units::scale<60, boost::units::static_rational<1> >
	>::type _minute;
BOOST_UNITS_STATIC_CONSTANT(minutes, _minute);
typedef boost::units::quantity<_minute, double> Minute;

// Define RPS and rounds_per_secondsi
typedef boost::units::make_scaled_unit<
		boost::units::si::frequency, boost::units::scale<60, boost::units::static_rational<-1> >
	>::type rpm_type;
BOOST_UNITS_STATIC_CONSTANT(rounds_per_minute, rpm_type);
typedef boost::units::quantity<rpm_type, double> RPM;


#endif
