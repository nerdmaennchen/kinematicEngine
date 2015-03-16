/*
 * kinematicTree.cpp
 *
 *  Created on: 25.11.2013
 *      Author: lutz
 */

#include "kinematicTree.h"

#include "node/kinematicNodeFactory.h"
#include "node/kinematicNode.h"

#include <iostream>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/optional/optional.hpp>

namespace kinematicEngine {

KinematicTree::KinematicTree()
	: m_nodes()
	, m_root(nullptr)
{
}

KinematicTree::~KinematicTree() {
}

void KinematicTree::setup(const RobotDescription &robotDescription) {
	// clone the nodes in a detached state (no parents, no children)
	m_MotorCnt = 0;
	for (const auto& in : robotDescription.getNodes()) {
		m_nodes[in.first] = in.second->cloneDetached();
		if (in.second->isServo()) {
			m_MotorCnt += in.second->getMotorCnt();
		}
	}

	// for each node, set the correct parent (this will auto-fill the children)
	for (const auto& in : robotDescription.getNodes()) {
		const KinematicNode* otherParentNode = robotDescription.getNodes().at(in.first)->getParent();
		if (otherParentNode) {
			m_nodes[in.first]->setParent( m_nodes[ otherParentNode->getID() ] );
		}
		else {
			m_root = m_nodes[in.first];
		}
	}
}

arma::mat44 KinematicTree::getTransitionMatrixFromTo(NodeID from, NodeID to) const
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


KinematicPath KinematicTree::getPathFromNodeToNode(NodeID from, NodeID to) const
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
				std::cerr << "I have built an impossible kinematic path! there must be a bug in KinematicTree::getPathFromNodeToNode" << std::endl;
			}
		}
	}

	return fromNodes;
}


KinematicTree::NodeID KinematicTree::getNodeID(const std::string &effectorName) const {
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
		const NodeID rootID = getNodeID(rootName);
		ret = getNode(rootID);
	}

	return ret;
}

const KinematicNode* KinematicTree::getNode(NodeID id) const {
	auto it = m_nodes.find(id);
	if (it != m_nodes.end()) {
		return it->second;
	} else {
		return nullptr;
	}
}

KinematicMass KinematicTree::getCOM() const
{
	const KinematicNode *node = m_root;
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

KinematicMass KinematicTree::getCOM(NodeID node) const
{
	arma::mat44 nodeToRoot = getTransitionMatrixFromTo(node, m_root->getID());
	arma::colvec4 comPos;
	KinematicMass com = getCOM();
	comPos.rows(0, 2) = com.m_position;
	comPos(3) = 1.;
	comPos = nodeToRoot * comPos;
	com.m_position = comPos.rows(0, 2);
	return com;
}


KinematicMass KinematicTree::getCOM(NodeID base, NodeID end) const
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
		forward = forward * path[nodeCnt - 1].m_node->getMatrixToRelative(node.m_node);
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


arma::colvec3 KinematicTree::getLinearMomentum(NodeID referenceFrame) const
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
		std::map<NodeID, arma::colvec3> &positions,
		const KinematicNode *node,
		arma::mat44 const& prev) const
{
	const arma::mat44 base = prev * node->getForwardMatrix();
	positions[node->getID()] = base.col(3).rows(0, 2);

	for (const KinematicNode* const &child : node->getChildren()) {
		calculateEffectorsPositions(positions, child, base);
	}
}

}
