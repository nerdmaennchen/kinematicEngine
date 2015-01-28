/*
 * kinematicNodeFactory.cpp
 *
 *  Created on: 14.05.2014
 *      Author: lutz
 */

#include "kinematicNodeFactory.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/optional/optional.hpp>

#include "kinematicNodeRotation.h"
#include "kinematicNodeWheel.h"
#include "kinematicNodePropeller.h"
#include "kinematicNodeParallelRotation.h"
#include "kinematicNodeDummy.h"
#include "kinematicNodeFixed.h"

enum class KinematicNodeType  {
	KINEMATIC_NODE_TYPE_DUMMY,
	KINEMATIC_NODE_TYPE_ROTATION,
	KINEMATIC_NODE_PARALLEL_TYPE_ROTATION,
	KINEMATIC_NODE_TYPE_PRISMATIC,
	KINEMATIC_NODE_TYPE_OMNIWHEEL
};

class KinematicNodeFactoryPrivClass {
public:
	KinematicNodeFactoryPrivClass() {
	}

	KinematicNodeRotation *generateRotationNode(boost::property_tree::ptree::value_type &ptree) const {
		MotorID id = MotorID(0);
		std::string name = "";
		Millimeter translationX = 0 * millimeters;
		Millimeter translationY = 0 * millimeters;
		Millimeter translationZ = 0 * millimeters;
		Degree alphaX = 0 * degrees;
		Degree alphaY = 0 * degrees;
		Degree alphaZ = 0 * degrees;
		double defaultValue = 0.;
		double minValue = 0.;
		double maxValue = 0.;
		double maxForce = 0.;
		double maxSpeed = 0.;
		assembleAttrs(ptree, id, name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ, defaultValue, minValue, maxValue, maxForce, maxSpeed);

		return new KinematicNodeRotation(id,
						nullptr,
						name,
						minValue,
						maxValue,
						defaultValue,
						maxForce,
						maxSpeed * rounds_per_minute,
						translationX,
						translationY,
						translationZ,
						alphaX,
						alphaY,
						alphaZ);
	}

	KinematicNodeWheel *generateWheelNode(boost::property_tree::ptree::value_type &ptree) const {
		MotorID id = MotorID(0);
		std::string name = "";
		Millimeter translationX = 0 * millimeters;
		Millimeter translationY = 0 * millimeters;
		Millimeter translationZ = 0 * millimeters;
		Degree alphaX = 0 * degrees;
		Degree alphaY = 0 * degrees;
		Degree alphaZ = 0 * degrees;
		double defaultValue = 0.;
		double minValue = 0.;
		double maxValue = 0.;
		double maxForce = 0.;
		double maxSpeed = 0.;

		assembleAttrs(ptree, id, name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ, defaultValue, minValue, maxValue, maxForce, maxSpeed);

		return new KinematicNodeWheel(id,
						nullptr,
						name,
						maxForce,
						maxSpeed * rounds_per_minute,
						translationX,
						translationY,
						translationZ,
						alphaX,
						alphaY,
						alphaZ);
	}

	KinematicNodeWheel *generatePropellerNode(boost::property_tree::ptree::value_type &ptree) const {
		MotorID id = MotorID(0);
		std::string name = "";
		Millimeter translationX = 0 * millimeters;
		Millimeter translationY = 0 * millimeters;
		Millimeter translationZ = 0 * millimeters;
		Degree alphaX = 0 * degrees;
		Degree alphaY = 0 * degrees;
		Degree alphaZ = 0 * degrees;
		double defaultValue = 0.;
		double minValue = 0.;
		double maxValue = 0.;
		double maxForce = 0.;
		double maxSpeed = 0.;

		assembleAttrs(ptree, id, name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ, defaultValue, minValue, maxValue, maxForce, maxSpeed);


		double speedToForceFactor = 0.;
		boost::optional<double> speedToForceFactorProp = ptree.second.get_optional<double>("<xmlattr>.speedtoforcefactor");
		if (speedToForceFactorProp.is_initialized()) {
			speedToForceFactor = speedToForceFactorProp.get();
		}

		return new KinematicNodePropeller(id,
						nullptr,
						name,
						maxForce,
						maxSpeed * rounds_per_minute,
						translationX,
						translationY,
						translationZ,
						alphaX,
						alphaY,
						alphaZ,
						speedToForceFactor);
	}

	KinematicNodeParallelRotation *generateParallelRotationNode(boost::property_tree::ptree::value_type &ptree) const {
		MotorID id = MotorID(0);
		std::string name = "";
		Millimeter translationX = 0 * millimeters;
		Millimeter translationY = 0 * millimeters;
		Millimeter translationZ = 0 * millimeters;
		Degree alphaX = 0 * degrees;
		Degree alphaY = 0 * degrees;
		Degree alphaZ = 0 * degrees;
		double defaultValue = 0.;
		double minValue = 0.;
		double maxValue = 0.;
		double maxForce = 0.;
		double maxSpeed = 0.;
		assembleAttrs(ptree, id, name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ, defaultValue, minValue, maxValue, maxForce, maxSpeed);

		Millimeter limbLength = 0 * millimeters;
		boost::optional<double> limbLengthProp = ptree.second.get_optional<double>("<xmlattr>.limbLength");
		if (limbLengthProp.is_initialized()) {
			limbLength = limbLengthProp.get() * millimeters;
		}

		return new KinematicNodeParallelRotation(id,
						nullptr,
						name,
						minValue,
						maxValue,
						defaultValue,
						maxForce,
						maxSpeed * rounds_per_minute,
						translationX,
						translationY,
						translationZ,
						alphaX,
						alphaY,
						alphaZ,
						limbLength);
	}

	KinematicNodeDummy *generateDummyNode(boost::property_tree::ptree::value_type &ptree) const {
		MotorID id = MotorID(0);
		std::string name = "";
		Millimeter translationX = 0 * millimeters;
		Millimeter translationY = 0 * millimeters;
		Millimeter translationZ = 0 * millimeters;
		Degree alphaX = 0 * degrees;
		Degree alphaY = 0 * degrees;
		Degree alphaZ = 0 * degrees;
		double defaultValue = 0.;
		double minValue = 0.;
		double maxValue = 0.;
		double maxForce = 0.;
		double maxSpeed = 0.;
		assembleAttrs(ptree, id, name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ, defaultValue, minValue, maxValue, maxForce, maxSpeed);

		return new KinematicNodeDummy(id, nullptr, name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ);
	}

	KinematicNodeFixed *generateFixedNode(boost::property_tree::ptree::value_type &ptree) const {
		MotorID id = MotorID(0);
		std::string name = "";
		Millimeter translationX = 0 * millimeters;
		Millimeter translationY = 0 * millimeters;
		Millimeter translationZ = 0 * millimeters;
		Degree alphaX = 0 * degrees;
		Degree alphaY = 0 * degrees;
		Degree alphaZ = 0 * degrees;
		double defaultValue = 0.;
		double minValue = 0.;
		double maxValue = 0.;
		double maxForce = 0.;
		double maxSpeed = 0.;
		assembleAttrs(ptree, id, name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ, defaultValue, minValue, maxValue, maxForce, maxSpeed);

		return new KinematicNodeFixed(id, nullptr, name, translationX, translationY, translationZ, alphaX, alphaY, alphaZ);
	}

private:

	void assembleAttrs(boost::property_tree::ptree::value_type &ptree
			, MotorID &id
			, std::string &name
			, Millimeter &translationX
			, Millimeter &translationY
			, Millimeter &translationZ
			, Degree &alphaX
			, Degree &alphaY
			, Degree &alphaZ
			, double &defaultValue
			, double &minValue
			, double &maxValue
			, double &maxForce
			, double &maxSpeed) const {
		try {
			name = ptree.second.get<std::string>("<xmlattr>.name");
		} catch (boost::exception const& e) {
			ERROR("RobotDescription node is missing name.");
			return;
		}

		auto optId = ptree.second.get_optional<int>("<xmlattr>.id");

		// if no (valid) ID was given, assign an automatic ID
		if (optId) {
			id = optId.get();
		} else {
			static CriticalSection cs;
			CriticalSectionLock lock(cs);

			static int autoID = 100000;
			id = MotorID(autoID++);
		}


		boost::optional<std::string> rotationProp = ptree.second.get_optional<std::string>("<xmlattr>.rpy");
		if (rotationProp.is_initialized()) {
			double aX, aY, aZ;
			std::istringstream i(rotationProp.get());
			i >> aX >> aY >> aZ;
			alphaX = aX * degrees;
			alphaY = aY * degrees;
			alphaZ = aZ * degrees;
		}

		boost::optional<std::string> positionProp = ptree.second.get_optional<std::string>("<xmlattr>.position");
		if (positionProp.is_initialized()) {
			std::istringstream i(positionProp.get());
			double tX, tY, tZ;
			i >> tX >> tY >> tZ;
			translationX = tX * millimeters;
			translationY = tY * millimeters;
			translationZ = tZ * millimeters;
		}

		boost::optional<std::string> anglesProp = ptree.second.get_optional<std::string>("<xmlattr>.defaultMinMaxAngle");
		if (anglesProp.is_initialized()) {
			double dA, minA, maxA;
			std::istringstream i(anglesProp.get());
			i >> dA >> minA >> maxA;
			defaultValue = dA;
			minValue     = minA;
			maxValue     = maxA;
		}

		boost::optional<std::string> maxForceProp = ptree.second.get_optional<std::string>("<xmlattr>.maxForce");
		if (maxForceProp.is_initialized()) {
			std::istringstream i(maxForceProp.get());
			i >> maxForce;
		}

		boost::optional<std::string> maxSpeedProp = ptree.second.get_optional<std::string>("<xmlattr>.maxSpeed");
		if (maxSpeedProp.is_initialized()) {
			std::istringstream i(maxSpeedProp.get());
			i >> maxSpeed;
		}
	}
};

KinematicNodeFactory::KinematicNodeFactory() {

}

KinematicNodeFactory::~KinematicNodeFactory() {
}

KinematicNode* KinematicNodeFactory::createNodeFromPTree(boost::property_tree::ptree::value_type ptree)
{
	KinematicNode* kinematicNode = nullptr;
	KinematicNodeFactoryPrivClass nodeBuilder;

	std::string typeName = ptree.second.get<std::string>("<xmlattr>.type");
	std::transform(typeName.begin(), typeName.end(), typeName.begin(), ::tolower);

	if (typeName == "dummy")
	{
		kinematicNode = nodeBuilder.generateDummyNode(ptree);
	}  else if (typeName == "rotation")
	{
		kinematicNode = nodeBuilder.generateRotationNode(ptree);
	} else if (typeName == "parallelrotation")
	{
		kinematicNode = nodeBuilder.generateParallelRotationNode(ptree);
	} else if (typeName == "fixed")
	{
		kinematicNode = nodeBuilder.generateFixedNode(ptree);
	} else if (typeName == "wheel")
	{
		kinematicNode = nodeBuilder.generateWheelNode(ptree);
	} else if (typeName == "propeller")
	{
		kinematicNode = nodeBuilder.generatePropellerNode(ptree);
	} else if (typeName == "piston")
	{
		// TODO
	}

	/* build the visual stuff: */
	BOOST_FOREACH(boost::property_tree::ptree::value_type const &subChild, ptree.second)
	{
		if ("body" == subChild.first)
		{
			double mass = subChild.second.get<double>("<xmlattr>.mass");
			double pX, pY, pZ;
			std::string posString = subChild.second.get<std::string>("<xmlattr>.position");
			std::string name = "";
			boost::optional<std::string> nameStr = subChild.second.get_optional<std::string>("<xmlattr>.name");
			if (nameStr.is_initialized()) {
				name = nameStr.get();
			}
			std::istringstream i(posString);
			i >> pX >> pY >> pZ;
			arma::colvec3 positionMM = arma::colvec({pX, pY, pZ}) * 0.001;
			kinematicNode->addMass(mass, positionMM, name);
		}
		if ("visual" == subChild.first)
		{
			BOOST_FOREACH(boost::property_tree::ptree::value_type const &geometryChild, subChild.second.get_child("geometry"))
			{

				double pX = 0;
				double pY = 0;
				double pZ = 0;
				boost::optional<std::string> posString = geometryChild.second.get_optional<std::string>("<xmlattr>.center");
				if (posString.is_initialized()) {
					std::istringstream posSS(posString.get());
					posSS >> pX >> pY >> pZ;
				}

				double rX = 0;
				double rY = 0;
				double rZ = 0;
				boost::optional<std::string> rotString = geometryChild.second.get_optional<std::string>("<xmlattr>.rpy");
				if (rotString.is_initialized()) {
					std::istringstream rotSS(rotString.get());
					rotSS >> rX >> rY >> rZ;
				}

				std::string name = "";
				boost::optional<std::string> nameStr = geometryChild.second.get_optional<std::string>("<xmlattr>.name");
				if (nameStr.is_initialized()) {
					name = nameStr.get();
				}

				boost::optional<std::string> colorProp = geometryChild.second.get_optional<std::string>("<xmlattr>.color");
				KinematicVisual::ColorVec colorVec({1, 1, 0, 1});
				if (colorProp.is_initialized()) {
					std::istringstream colStrS(colorProp.get());
					colStrS >> colorVec[0] >> colorVec[1] >> colorVec[2] >> colorVec[3];
				}

				boost::optional<std::string> textureProp = geometryChild.second.get_optional<std::string>("<xmlattr>.textureNo");
				int textureNo = 1;
				if (textureProp.is_initialized()) {
					std::istringstream textureSS(textureProp.get());
					textureSS >> textureNo;
					textureNo = Math::limited(textureNo, 0, 4);
				}

				boost::optional<std::string> visibleProp = geometryChild.second.get_optional<std::string>("<xmlattr>.visible");
				bool visible = true;
				if (visibleProp.is_initialized()) {
					std::string visibleStr = visibleProp.get();
					std::transform(visibleStr.begin(), visibleStr.end(), visibleStr.begin(), ::tolower);
					if ("true" != visibleStr) {
						visible = false;
					}
				}

				boost::optional<std::string> canCollideProp = geometryChild.second.get_optional<std::string>("<xmlattr>.cancollide");
				bool canCollide = true;
				if (canCollideProp.is_initialized()) {
					std::string canCollideStr = canCollideProp.get();
					std::transform(canCollideStr.begin(), canCollideStr.end(), canCollideStr.begin(), ::tolower);
					if ("true" != canCollideStr) {
						canCollide = false;
					}
				}

				if ("box" == geometryChild.first) {

					std::string dimString = geometryChild.second.get<std::string>("<xmlattr>.dimensions");
					double dX, dY, dZ;
					std::istringstream d(dimString);
					d >> dX >> dY >> dZ;

					kinematicNode->addVisual(
						new KinematicVisualBox(
								name,
								pX*millimeters,
								pY*millimeters,
								pZ*millimeters,
								dX*millimeters,
								dY*millimeters,
								dZ*millimeters,
								rX*degrees,
								rY*degrees,
								rZ*degrees,
								colorVec,
								textureNo,
								visible,
								canCollide));
				} else if ("cylinder" == geometryChild.first) {

					std::string lenString = geometryChild.second.get<std::string>("<xmlattr>.length");
					double length;
					std::istringstream dSS(lenString);
					dSS >> length;

					std::string radiusString = geometryChild.second.get<std::string>("<xmlattr>.radius");
					double radius;
					std::istringstream radSS(radiusString);
					radSS >> radius;

					kinematicNode->addVisual(
						new KinematicVisualCylinder(
								name,
								pX*millimeters,
								pY*millimeters,
								pZ*millimeters,
								radius*millimeters,
								length*millimeters,
								rX*degrees,
								rY*degrees,
								rZ*degrees,
								colorVec,
								textureNo,
								visible,
								canCollide));
				} else if ("sphere" == geometryChild.first) {
					std::string radiusString = geometryChild.second.get<std::string>("<xmlattr>.radius");
					double radius;
					std::istringstream radSS(radiusString);
					radSS >> radius;

					kinematicNode->addVisual(
						new KinematicVisualSphere(
								name,
								pX*millimeters,
								pY*millimeters,
								pZ*millimeters,
								radius*millimeters,
								rX*degrees,
								rY*degrees,
								rZ*degrees,
								colorVec,
								textureNo,
								visible,
								canCollide));
				}
			}
		}
	}

	return kinematicNode;

}
