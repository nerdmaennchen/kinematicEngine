/*
 * kinematicTree.h
 *
 *  Created on: 25.11.2013
 *      Author: lutz
 */

#ifndef KINEMATICTREE_H_
#define KINEMATICTREE_H_

#include "utils/units.h"

#include "kinematicEngine/node/kinematicNode.h"
#include "kinematicEngine/tasks/kinematicPath.h"

#include "kinematicEngine/robotDescription.h"
#include "kinematicEngine/kinematics.h"

#include <boost/property_tree/ptree.hpp>

#include <array>
#include <map>
#include <armadillo>

namespace kinematicEngine {

/**
 * @ingroup representations
 */
class KinematicTree {
	typedef kinematics::NodeID NodeID;

public:

	KinematicTree();
	virtual ~KinematicTree();

	void setup(const RobotDescription &robotDescription);

	arma::mat44 getTransitionMatrixFromTo(NodeID from, NodeID to) const;

	KinematicPath getPathFromNodeToNode(NodeID from, NodeID to) const;

	std::string getNameOfNode(NodeID id) const {
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
	KinematicMass getCOM(NodeID node) const;

	/**
	 * get the center of mass in the coordinate frame of the base node including all limbs to end
	 * @return
	 */
	KinematicMass getCOM(NodeID base, NodeID end) const;

	/**
	 * calculate the linear momentum of the entire robot from the perspective of the node at referenceFrame
	 * @param referenceFrame
	 * @return
	 */
	arma::colvec3 getLinearMomentum(NodeID referenceFrame) const;

	const KinematicNode *getRootNode() const;

	NodeID getNodeID(const std::string &effectorName) const;

	inline int getMotorCnt() const {
		return m_MotorCnt;
	}

	inline int getNodeCnt() const {
		return m_nodes.size();
	}

	const KinematicNode *getNode(NodeID id) const;
	KinematicNode* getNode(NodeID id) {
		return const_cast<KinematicNode*>(((const KinematicTree*)(this))->getNode(id));
	}


	std::map<NodeID, KinematicNode*> const& getNodes() const {
		return m_nodes;
	}

	std::map<NodeID, KinematicNode*>& getNodes() {
		return m_nodes;
	}

	/**
	 * calculate all positions of the endeffectors
	 * @param positions matrix where the positions reside
	 * @param node node from which to look at the tree
	 * @param prev parents coordinate frame
	 */
	void calculateEffectorsPositions(std::map<NodeID, arma::colvec3> &positions, const KinematicNode *node, arma::mat44 const& prev) const;

private:
	std::map<NodeID, KinematicNode*>   m_nodes;
	KinematicNode * m_root;
	uint m_MotorCnt;

	KinematicMass getCOMSub(const KinematicNode *node, arma::mat44 prev) const;


	arma::colvec4 getLinearMomentumSub(const KinematicNode *node, bool traversingUp, KinematicMass& o_massToPassBack) const;

};

}

#endif
