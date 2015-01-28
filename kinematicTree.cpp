/*
 * kinematicTree.cpp
 *
 *  Created on: 25.11.2013
 *      Author: lutz
 */

#include "kinematicTree.h"
#include "hardware/robot/motorIDs.h"
#include "debugging/debug3d.h"
#include "utils/utils.h"

#include "services.h"
#include "hardware/robot/robotModel.h"
#include "hardware/robot/robotDescription.h"

#include "node/kinematicNodeFactory.h"
#include "node/kinematicNode.h"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/optional/optional.hpp>


KinematicTree::KinematicTree() {
}

KinematicTree::~KinematicTree() {
}

void KinematicTree::setup(const RobotDescription &robotDescription) {
	// clone the nodes in a detached state (no parents, no children)
	for (const auto& in : robotDescription.getNodes()) {
		m_nodes[in.first] = in.second->cloneDetached();
		if (in.second->isServo()) {
			m_nodeToInt[in.first] = m_nodeToExt.size();
			m_nodeToExt.push_back(in.first);
		}
	}

	// for each node, set the correct parent (this will auto-fill the children)
	for (const auto& in : robotDescription.getNodes()) {
		const KinematicNode* otherParentNode = robotDescription.getNodes().at(in.first)->getParent();
		if (otherParentNode)
			m_nodes[in.first]->setParent( m_nodes[ otherParentNode->getID() ] );
	}
}

void KinematicTree::setMotorValue(MotorID id, Degree value) {
	m_nodes[id]->setValue(value.value());
}

void KinematicTree::setMotorValues(std::map<MotorID, Degree> const& values) {
	for (auto e : values) {
		m_nodes[e.first]->setValue(e.second.value());
	}
}

void KinematicTree::getMotorValues(std::map<MotorID, Degree> & values) const {
	for (auto node : m_nodes) {
		if (!(node.second->isFixedNode())) {
			values[node.first] = node.second->getValue() * degrees;
		}
	}
}

void KinematicTree::getMotorSpeeds(std::map<MotorID, RPM> & values) const {
	for (auto node : m_nodes) {
		if (!(node.second->isFixedNode())) {
			values[node.first] = node.second->getValueDerivative() * 60. / (2. * M_PI) * rounds_per_minute;
		}
	}
}

void KinematicTree::setMotorSpeed(MotorID id, RPM value) {
	m_nodes[id]->setValueDerivative((2 * M_PI * value / 60.).value());
}

void KinematicTree::setMotorSpeeds(std::map<MotorID, RPM> const& values) {
	for (auto e : values) {
		m_nodes[e.first]->setValueDerivative((2. * M_PI * e.second / 60.).value());
	}
}


arma::mat44 KinematicTree::getTransitionMatrixFromTo(MotorID from, MotorID to) const
{
	arma::mat44 ret = arma::eye(4, 4);

	{
		// check that nodes exist
		const KinematicNode *fromNode = getNode(from);
		const KinematicNode *backNode = getNode(to);
		if (nullptr == fromNode || nullptr == backNode)
			return ret;

		arma::mat44 forward  = arma::eye(4, 4);
		arma::mat44 backward = arma::eye(4, 4);

		while (fromNode != backNode)
		{
			/* traverse up */
			forward =  forward * fromNode->getBackwardMatrix();
			if (NULL != fromNode->getParent())
			{
				fromNode = fromNode->getParent();
			} else
			{
				break;
			}
		}
		while (backNode != fromNode)
		{
			/* traverse down */
			backward = backNode->getForwardMatrix() * backward;
			if (NULL != backNode->getParent())
			{
				backNode = backNode->getParent();
			} else
			{
				break;
			}
		}

		ret = forward * backward;
	}
	return ret;
}
/**
 * clip an angle to be applied to a motor (the values are defined in the robotdescription.xml)
 * @param id
 * @param angle
 * @return clipped angle
 */
Degree KinematicTree::clipAngleForMotor(MotorID id, Degree angle) const
{
	Degree clippedAngle = angle;
	if (services.getRobotModel().getRobotDescription()->getMotorIDs().count(id) > 0) {
		const KinematicNode *node = getNode(id);
		clippedAngle = node->clipValue(angle.value()) * degrees;
	}
	return clippedAngle;
}


KinematicPath KinematicTree::getPathFromNodeToNode(MotorID from, MotorID to) const
{
	const KinematicNode *fromNode = getNode(from);
	const KinematicNode *backNode = getNode(to);

	if (nullptr == fromNode || nullptr == backNode)
		return KinematicPath();

	KinematicPath fromNodes, backNodes;
	KinematicPathNode link;
	bool hasLink = false;

	/* traverse in the direction of root */
	while (NULL != fromNode)
	{
		KinematicPathNode node(fromNode, KinematicPathNode::Direction::FROM_PARENT);
		fromNodes.push_back(node);
		fromNode = fromNode->getParent();
	}
	while (NULL != backNode)
	{
		KinematicPathNode node(backNode, KinematicPathNode::Direction::FROM_CHILD);
		backNodes.push_back(node);
		backNode = backNode->getParent();
	}

	if (0 < fromNodes.size())
	{
		fromNodes[0].m_direction = KinematicPathNode::Direction::BEGINNING;
	}

	while (fromNodes.size() > 0 && backNodes.size() > 0)
	{
		if (fromNodes.back().m_node == backNodes.back().m_node)
		{
			hasLink = true;
			link = fromNodes.back();
			fromNodes.pop_back();
			backNodes.pop_back();
		} else
		{
			break;
		}
	}

	/* join the paths */

	if (false != hasLink)
	{
		if (KinematicPathNode::Direction::BEGINNING != link.m_direction)
		{
			link.m_direction = KinematicPathNode::Direction::LINK;
		}
		fromNodes.push_back(link);
	}

	for (int i = backNodes.size() - 1; i >= 0; --i)
	{
		fromNodes.push_back(backNodes[i]);
	}

	/* set the directions in which the nodes are connected */
	for (uint i = 1; i < fromNodes.size(); ++i)
	{
		KinematicPathNode *prevNode = &(fromNodes[i - 1]);
		KinematicPathNode *node = &(fromNodes[i]);

//		INFO("from %s to %s", prevNode->m_node->getName().c_str(), node->m_node->getName().c_str());
		if (KinematicPathNode::Direction::LINK != node->m_direction)
		{
			if (node->m_node == prevNode->m_node->getParent())
			{
				node->m_direction = KinematicPathNode::Direction::FROM_CHILD;
			} else if (prevNode->m_node == node->m_node->getParent())
			{
				node->m_direction = KinematicPathNode::Direction::FROM_PARENT;
			} else
			{
				/* this should not be executed anyway... */
				ERROR("I have built an impossible kinematic path! there must be a bug in KinematicTree::getPathFromNodeToNode");
			}
		}
	}

	return fromNodes;
}


void KinematicTree::drawPose(DebugStream3D &debug3d, bool includeMasses, bool includeEquivalentMasses, bool drawCOM, bool drawGroundPlane) const {
	const KinematicNode *rootNode = getNode(EFFECTOR_ID_ROOT);
	if (nullptr == rootNode)
		return;

	arma::mat44 base = rootNode->getForwardMatrix();
	arma::colvec3 position = base.submat(0, 3, 2, 3);
	SETCOLORINDEX3D(EFFECTOR_ID_ROOT);
	SPHERE3D(position(0), position(1), position(2), 0.003);

	if (drawCOM)
	{
		KinematicMass com = getCOM(EFFECTOR_ID_ROOT);
		const arma::colvec3 comPosition = com.m_position;
		const arma::colvec3 linearMomentumVector = getLinearMomentum(EFFECTOR_ID_ROOT) * 0.00001;
		const arma::colvec3 linearMomentumEnd = comPosition + linearMomentumVector;
//		linearMomentumVector.print(); printf("\n\n");

		SPHERE3D(comPosition(0), comPosition(1), comPosition(2), 0.00001 * com.m_massGrams);
		LINE3D(comPosition(0), comPosition(1), comPosition(2), linearMomentumEnd(0), linearMomentumEnd(1), linearMomentumEnd(2));
	}

	if (includeMasses)
	{
		for (KinematicMass const &mass : rootNode->getMasses())
		{
			arma::colvec4 helper;
			helper.rows(0, 2) = mass.m_position;
			helper(3) = 1.;
			arma::colvec4 position = base * helper;
			SPHERE3D(position(0), position(1), position(2), 0.0001 * mass.m_massGrams);
		}
	}

	if (includeEquivalentMasses)
	{
		arma::colvec4 helper;
		helper.rows(0, 2) = rootNode->getEquivalentMass().m_position;
		helper(3) = 1.;
		arma::colvec4 position = base * helper;
		SPHERE3D(position(0), position(1), position(2), 0.0001 * rootNode->getEquivalentMass().m_massGrams);
	}

	/* draw coordinate system */
	for (int i = 0; i < 3; ++i)
	{
		SETCOLORINDEX3D(i + 1);
		arma::colvec3 targetPos = base.col(i).rows(0, 2) * 0.05 + position;
		LINE3D(position(0), position(1), position(2),
				targetPos(0), targetPos(1), targetPos(2));
	}

	for (const KinematicNode *node : getNode(EFFECTOR_ID_ROOT)->getChildren())
	{
		drawPoseSub(debug3d, node, base, includeMasses, includeEquivalentMasses);
	}
}

MotorID KinematicTree::getNodeID(const std::string &effectorName) const {
	for (const auto &it : m_nodes) {
		if (it.second->getName() == effectorName)
			return it.first;
	}

	return MOTOR_NONE;
}

const KinematicNode* KinematicTree::getRootNode() const {
	static const std::string rootName = "root";
	static const KinematicNode* ret = nullptr;

	if (nullptr == ret) {
		const MotorID rootID = getNodeID(rootName);
		ret = getNode(rootID);
	}

	return ret;
}

const KinematicNode* KinematicTree::getNode(MotorID id) const {
	auto it = m_nodes.find(id);
	if (it != m_nodes.end()) {
		return it->second;
	} else {
		return nullptr;
	}
}


void KinematicTree::drawPoseSub(DebugStream3D &debug3d, const KinematicNode *baseNode, arma::mat44 prev, bool includeMasses, bool includeEquivalentMasses) const
{
	arma::mat44 base = prev * baseNode->getForwardMatrix();
	arma::colvec3 position = base.submat(0, 3, 2, 3);
	arma::colvec3 prevPosition = prev.submat(0, 3, 2, 3);


	SETCOLORINDEX3D(baseNode->getID());
	LINE3D(position(0), position(1), position(2),
			prevPosition(0), prevPosition(1), prevPosition(2));
//		SPHERE3D(position(0), position(1), position(2), 0.005);

	if (includeMasses)
	{
		for (KinematicMass const &mass : baseNode->getMasses())
		{
			arma::colvec4 helper;
			helper.rows(0, 2) = mass.m_position;
			helper(3) = 1.;
			arma::colvec4 position = base * helper;
			SPHERE3D(position(0), position(1), position(2), 0.0001 * mass.m_massGrams);
		}
	}

	if (includeEquivalentMasses)
	{
		arma::colvec4 helper;
		helper.rows(0, 2) = baseNode->getEquivalentMass().m_position;
		helper(3) = 1.;
		arma::colvec4 position = base * helper;
		SPHERE3D(position(0), position(1), position(2), 0.0001 * baseNode->getEquivalentMass().m_massGrams);
	}

	/* draw coordinate system */
	for (int i = 0; i < 3; ++i)
	{
		SETCOLORINDEX3D(i + 1);
		arma::colvec3 targetPos = base.col(i).rows(0, 2) * 0.05 + position;
		LINE3D(position(0), position(1), position(2),
				targetPos(0), targetPos(1), targetPos(2));
	}

	for (KinematicNode  const *node : baseNode->getChildren())
	{
		drawPoseSub(debug3d, node, base, includeMasses, includeEquivalentMasses);
	}
}

KinematicMass KinematicTree::getCOM() const
{
	const KinematicNode *node = getNode(EFFECTOR_ID_ROOT);
	if (nullptr == node) {
		return KinematicMass();
	}

	KinematicMass ret = node->getEquivalentMass();

	for (const KinematicNode *child : node->getChildren())
	{
		ret += getCOMSub(child, arma::eye(4, 4));
	}

	ret.m_position = ret.m_position * 1. / ret.m_massGrams;

	return ret;
}

KinematicMass KinematicTree::getCOMSub(const KinematicNode *node, arma::mat44 prev) const
{
	KinematicMass ret;
	arma::mat44 base = prev * node->getForwardMatrix();

	ret += node->getEquivalentMass();

	arma::colvec4 helper;
	helper.rows(0, 2) = ret.m_position;
	helper(3) = 1.;
	arma::colvec4 position = base * helper;
	ret.m_position = position.rows(0, 2) * ret.m_massGrams;

	for (const KinematicNode *child : node->getChildren())
	{
		ret += getCOMSub(child, base);
	}

	return ret;
}

KinematicMass KinematicTree::getCOM(MotorID node) const
{
	arma::mat44 nodeToRoot = getTransitionMatrixFromTo(node, EFFECTOR_ID_ROOT);
	arma::colvec4 comPos;
	KinematicMass com = getCOM();
	comPos.rows(0, 2) = com.m_position;
	comPos(3) = 1.;
	comPos = nodeToRoot * comPos;
	com.m_position = comPos.rows(0, 2);
	return com;
}


KinematicMass KinematicTree::getCOM(MotorID base, MotorID end) const
{
	KinematicMass ret;

	arma::mat44 forward = arma::eye(4, 4);
	KinematicPath path = getPathFromNodeToNode(base, end);

	if (path.empty())
		return KinematicMass();

	path = invertKinematicPath(path);

	ret = getNode(base)->getEquivalentMass();

	ret.m_position = ret.m_position * ret.m_massGrams;

	uint nodeCnt = 0;
	for (KinematicPathNode const &node : path)
	{
		if (0 == nodeCnt)
		{
			/* we already have that node */
			++nodeCnt;
			continue;
		}
		forward = forward * path[nodeCnt - 1].m_node->getMatrixToRelative(node.m_node->getID());
		KinematicMass equivMass = node.m_node->getEquivalentMass();
		arma::colvec4 posHelper = arma::ones(4);
		posHelper.rows(0, 2) = equivMass.m_position;
		posHelper = forward * posHelper;
		equivMass.m_position = posHelper.rows(0, 2) * equivMass.m_massGrams;

		ret += equivMass;
		++nodeCnt;
	}

	ret.m_position = ret.m_position * 1. / ret.m_massGrams;

	return ret;
}


arma::colvec3 KinematicTree::getLinearMomentum(MotorID referenceFrame) const
{
	const KinematicNode* rootNode = getNode(referenceFrame);
	KinematicMass massFromParent, massFromChildren;
	arma::colvec4 momentum = arma::zeros(4);

	for (const KinematicNode* child: rootNode->getChildren()) {
		momentum += child->getForwardMatrix() * getLinearMomentumSub(child, false, massFromChildren);
	}

	momentum = getLinearMomentumSub(rootNode, true, massFromParent);

	return momentum.rows(0, 2);
}

arma::colvec4 KinematicTree::getLinearMomentumSub(const KinematicNode *node, bool traversingUp, KinematicMass& o_massToPassBack) const
{
	KinematicMass attachedNodesMass;
	arma::colvec4 linMomentum = arma::zeros(4);

	if (traversingUp) {
		if (nullptr != node->getParent()) {
			// recur into the parent

			// at first linMomentum is in the parents coordinate frame
			linMomentum = getLinearMomentumSub(node->getParent(), true, attachedNodesMass);

			// the momentum comes from the parents mass and the siblings masses
			for (const KinematicNode *child : node->getParent()->getChildren()) {
				if (child != node) {
					linMomentum += child->getForwardMatrix() * getLinearMomentumSub(child, false, attachedNodesMass);
				}
			}
			// transform linMomentum into the nodes coordinate frame
			linMomentum = node->getBackwardMatrix() * linMomentum;

			// add parents mass
			attachedNodesMass += node->getParent()->getEquivalentMass();

			// tansform the mass into the nodes coordinate frame
			attachedNodesMass.applyTransformation(node->getBackwardMatrix());

			linMomentum -= node->getLinearSpeedOfPoint(attachedNodesMass.m_position) * attachedNodesMass.m_massGrams;

			o_massToPassBack += attachedNodesMass;
		}
	} else {
		for (const KinematicNode *child : node->getChildren()) {
			linMomentum += child->getForwardMatrix() * getLinearMomentumSub(child, false, attachedNodesMass);
		}

		KinematicMass curNodesMass = attachedNodesMass + node->getEquivalentMass();
		arma::colvec4 velocity = node->getLinearSpeedOfPoint(curNodesMass.m_position);
		linMomentum += velocity * curNodesMass.m_massGrams;
		curNodesMass.applyTransformation(node->getForwardMatrix());
		o_massToPassBack += curNodesMass;
	}

	return linMomentum;
}

void KinematicTree::calculateEffectorsPositions(
		std::map<MotorID, arma::colvec3> &positions,
		const KinematicNode *node,
		arma::mat44 const& prev) const
{
	const arma::mat44 base = prev * node->getForwardMatrix();
	positions[node->getID()] = base.col(3).rows(0, 2);

	for (const KinematicNode* const &child : node->getChildren()) {
		calculateEffectorsPositions(positions, child, base);
	}
}

void KinematicTree::setGyroscopeAngles(arma::mat33 _rotMat) {
	std::map<MotorID, KinematicNode*>::iterator gyroIter = m_nodes.find(EFFECTOR_ID_GYROSCOPE);
	if (gyroIter != m_nodes.end())
	{
		gyroIter->second->setAdditionalExtrinsicRotation(_rotMat);
	} else {
//		ERROR("Try setting extrinsic angles for gyroscope which is not present!");
	}

}

