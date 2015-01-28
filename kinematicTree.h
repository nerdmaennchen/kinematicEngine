/*
 * kinematicTree.h
 *
 *  Created on: 25.11.2013
 *      Author: lutz
 */

#ifndef KINEMATICTREE_H_
#define KINEMATICTREE_H_

#include "debug.h"
#include "debugging/debug3d.h"
#include "utils/units.h"

#include "node/kinematicNode.h"
#include "tasks/kinematicPath.h"

#include "hardware/robot/robotDescription.h"
#include "hardware/robot/motorIDs.h"

#include <boost/property_tree/ptree.hpp>

#include <array>
#include <map>
#include <armadillo>

/**
 * @ingroup representations
 */
class KinematicTree {
public:
	KinematicTree();
	virtual ~KinematicTree();

	void setup(const RobotDescription &robotDescription);

	void setMotorValue(MotorID id, Degree value);
	void setMotorValues(std::map<MotorID, Degree> const& values);

	void setMotorSpeed(MotorID id, RPM value);
	void setMotorSpeeds(std::map<MotorID, RPM> const& values);

	void getMotorValues(std::map<MotorID, Degree> & values) const;
	void getMotorSpeeds(std::map<MotorID, RPM> & values) const;

	void setGyroscopeAngles(arma::mat33 _rotMat);

	void drawPose(DebugStream3D &debug3d, bool includeMasses, bool includeEquivalentMasses, bool drawCOM, bool drawGroundPlane) const;

	arma::mat44 getTransitionMatrixFromTo(MotorID from, MotorID to) const;

	Degree clipAngleForMotor(MotorID id, Degree angle) const;

	KinematicPath getPathFromNodeToNode(MotorID from, MotorID to) const;

	std::string getNameOfMotor(MotorID id) const {
		if (m_nodes.count(id)) {
			return m_nodes.at(id)->getName();
		}
		return "unknown motor";
	}

	/**
	 * get the center of mass in the coordinate frame of the root node
	 * @return
	 */
	KinematicMass getCOM() const;

	/**
	 * get the center of mass in the coordinate frame of the node
	 * @return
	 */
	KinematicMass getCOM(MotorID node) const;

	/**
	 * get the center of mass in the coordinate frame of the base node including all limbs to end
	 * @return
	 */
	KinematicMass getCOM(MotorID base, MotorID end) const;

	/**
	 * calculate the linear momentum of the entire robot from the perspective of the node at referenceFrame
	 * @param referenceFrame
	 * @return
	 */
	arma::colvec3 getLinearMomentum(MotorID referenceFrame) const;

	const KinematicNode *getRootNode() const;
	const KinematicNode *getNode(MotorID id) const;
	MotorID getNodeID(const std::string &effectorName) const;

	inline int getMotorCt() const {
		return m_nodeToExt.size();
    }
    inline int toInt(MotorID _id) const {
		return m_nodeToInt.at(_id);
    }
    inline MotorID toExt(int _id) const {
		return m_nodeToExt[_id];
    }

	KinematicNode* getNode(MotorID id) {
		return const_cast<KinematicNode*>(((const KinematicTree*)(this))->getNode(id));
	}

	/**
	 * calculate all positions of the endeffectors
	 * @param positions matrix where the positions reside
	 * @param node node from which to look at the tree
	 * @param prev parents coordinate frame
	 */
	void calculateEffectorsPositions(std::map<MotorID, arma::colvec3> &positions, const KinematicNode *node, arma::mat44 const& prev) const;

private:
	std::map<MotorID, KinematicNode*> m_nodes;
	std::map<MotorID, int>            m_nodeToInt;
	std::vector<MotorID>              m_nodeToExt;

	void drawPoseSub(DebugStream3D &debug3d, const KinematicNode *baseNode, arma::mat44 prev, bool includeMasses, bool includeEquivalentMasses) const;

	KinematicMass getCOMSub(const KinematicNode *node, arma::mat44 prev) const;


	arma::colvec4 getLinearMomentumSub(const KinematicNode *node, bool traversingUp, KinematicMass& o_massToPassBack) const;

};

#endif /* KINEMATICTREE_H_ */
