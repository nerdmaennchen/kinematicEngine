/*
 * supportPolygon.cpp
 *
 *  Created on: 20.02.2014
 *      Author: lutz
 */

#include "kinematicTreeSupportPolygon.h"
#include <limits>

namespace kinematicEngine {

KinematicTreeSupportPolygon::KinematicTreeSupportPolygon() : m_edges()
{

}

KinematicTreeSupportPolygon::KinematicTreeSupportPolygon(std::vector<std::pair<MotorID, arma::colvec4>> nodes3, arma::mat vectorToplaneTransform)
	: m_edges()
	, m_vectorToplaneTransform(vectorToplaneTransform)
{
	std::vector<std::pair<MotorID, arma::colvec2>> nodes;
	for (std::pair<MotorID, arma::colvec4> const& node3 :nodes3) {
		nodes.push_back({node3.first, vectorToplaneTransform * node3.second});
	}

	if (nodes3.size() > 1)
	{
		std::sort(nodes.begin(), nodes.end(), [](std::pair<MotorID, arma::colvec2> a, std::pair<MotorID, arma::colvec2> b)
		{
			return a.second(0) < b.second(0);
		});

		const arma::colvec2 beginPoint = nodes.front().second;
		const arma::colvec2 endPoint = nodes.back().second;
		arma::colvec2 diff =  endPoint - beginPoint;
		std::vector<const std::pair<MotorID, arma::colvec2>* > leftNodes, rightNodes;

		for (std::pair<MotorID, arma::colvec2> const &node : nodes)
		{
			const arma::colvec2 dirNode = node.second - beginPoint;
			const double z = diff(0) * dirNode(1) - diff(1) * dirNode(0);
			if (z > 0)
			{
				leftNodes.push_back(&node);
			} else if (z < 0)
			{
				rightNodes.push_back(&node);
			}
		}

		std::reverse(rightNodes.begin(), rightNodes.end());

		std::vector<const std::pair<MotorID, arma::colvec2>*> leftPolygonParts = buildPolygonSub(leftNodes, beginPoint, endPoint);
		std::vector<const std::pair<MotorID, arma::colvec2>*> rightPolygonParts = buildPolygonSub(rightNodes, endPoint, beginPoint);

		for (const std::pair<MotorID, arma::colvec2>* const &node : rightPolygonParts)
		{
			m_edges.push_back(*node);
		}
		m_edges.push_back(nodes.front());
		for (const std::pair<MotorID, arma::colvec2>* const &node : leftPolygonParts)
		{
			m_edges.push_back(*node);
		}
		m_edges.push_back(nodes.back());

	} else
	{
		m_edges = nodes;
	}
}


std::vector<const std::pair<MotorID, arma::colvec2>*> KinematicTreeSupportPolygon::buildPolygonSub(std::vector<const std::pair<MotorID, arma::colvec2>*> nodes, arma::colvec2 lineBegin, arma::colvec2 lineEnd) const
{
	std::vector<const std::pair<MotorID, arma::colvec2>*> ret;
	const uint cnt = nodes.size();
	const arma::colvec2 lineDir = lineEnd - lineBegin;
	const double lineDirLen = arma::norm(lineDir, 2);
	if (cnt > 1)
	{
		/* find point with greatest distance left of us */
		const std::pair<MotorID, arma::colvec2>* leftMostPoint = nodes.front();
		double leftMostDist = 0.;

		std::vector<const std::pair<MotorID, arma::colvec2>*> relevantNodes;

		for (const std::pair<MotorID, arma::colvec2>* const &node : nodes)
		{
			const arma::colvec2 dirNode = node->second - lineBegin;
			const double z = lineDir(0) * dirNode(1) - lineDir(1) * dirNode(0);
			if (z > 0.)
			{
				arma::mat22 helperMat;
				helperMat.col(0) = lineDir;
				helperMat.col(1) = dirNode;
				const double dist = arma::det(helperMat) / lineDirLen;

				relevantNodes.push_back(node);
				if (dist > leftMostDist)
				{
					leftMostDist = dist;
					leftMostPoint = node;
				}
			}
		}


		const arma::colvec2 pivotPoint = leftMostPoint->second;
		std::vector<const std::pair<MotorID, arma::colvec2>* > leftNodes, rightNodes;

		const double leftMostDistSq = arma::norm(pivotPoint - lineBegin, 1);
		for (const std::pair<MotorID, arma::colvec2>* const &node : relevantNodes)
		{
			if (node != leftMostPoint)
			{
				const arma::colvec2 vecFromBeginOfLine = node->second - lineBegin;
				const double norm = arma::norm(vecFromBeginOfLine, 1);
				if (norm < leftMostDistSq)
				{
					leftNodes.push_back(node);
				} else
				{
					rightNodes.push_back(node);
				}
			}
		}

		std::vector<const std::pair<MotorID, arma::colvec2>*> leftPolygonParts = buildPolygonSub(leftNodes, lineBegin, pivotPoint);
		std::vector<const std::pair<MotorID, arma::colvec2>*> rightPolygonParts = buildPolygonSub(rightNodes, pivotPoint, lineEnd);

		for (const std::pair<MotorID, arma::colvec2>* const &node : leftPolygonParts)
		{
			ret.push_back(node);
		}

		const arma::colvec2 dirNode = pivotPoint - lineBegin;
		const double z = lineDir(0) * dirNode(1) - lineDir(1) * dirNode(0);
		if (z > 0)
		{
			ret.push_back(leftMostPoint);
		}
		for (const std::pair<MotorID, arma::colvec2>* const &node : rightPolygonParts)
		{
			ret.push_back(node);
		}
	} else if (1 == nodes.size())
	{
		const arma::colvec2 dirNode = nodes.front()->second - lineBegin;
		const double z = lineDir(0) * dirNode(1) - lineDir(1) * dirNode(0);
		if (z > 0)
		{
			ret.push_back(nodes.front());
		}
	}
	return ret;
}

bool KinematicTreeSupportPolygon::isInsidePolygon(arma::colvec4 vector) const {
	if (m_edges.size() < 2) {
		return false;
	}

	const arma::colvec2 vec2 = m_vectorToplaneTransform * vector;

	for (uint i = 1; i < m_edges.size(); ++i) {
		const arma::colvec2 dir = m_edges[i - 1].second - m_edges[i].second;
		const arma::colvec2 helper = vec2 - m_edges[i - 1].second;
		const double crossProductZ = helper(0) * dir(1) - helper(1) * dir(0);
		if (crossProductZ > 0) {
			return false;
		}
	}

	const arma::colvec2 dir = m_edges[m_edges.size() - 1].second - m_edges[0].second;
	const arma::colvec2 helper = vec2 - m_edges[m_edges.size() - 1].second;
	const double crossProductZ = helper(0) * dir(1) - helper(1) * dir(0);
	if (crossProductZ > 0) {
		return false;
	}


	return true;
}

KinematicTreeSupportPolygon::~KinematicTreeSupportPolygon() {
	// TODO Auto-generated destructor stub
}

}
