/*
 * supportPolygon.h
 *
 *  Created on: 20.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICTREESUPPORTPOLYGON_H_
#define KINEMATICTREESUPPORTPOLYGON_H_

#include <vector>
#include <armadillo>
#include "kinematicEngine/node/kinematicNode.h"

namespace kinematicEngine {

class KinematicNode;

class KinematicTreeSupportPolygon {
public:
	KinematicTreeSupportPolygon();
	KinematicTreeSupportPolygon(std::vector<std::pair<MotorID, arma::colvec4>> nodes, arma::mat vectorToplaneTransform);
	virtual ~KinematicTreeSupportPolygon();

	inline std::vector<std::pair<MotorID, arma::colvec2>> const &getEdges() const {
		return m_edges;
	}

	bool isInsidePolygon(arma::colvec4 vector) const;

	arma::mat const& getVectorToplaneTransform() const {
		return m_vectorToplaneTransform;
	}

private:
	// the actual support Polygon
	std::vector<std::pair<MotorID, arma::colvec2>> m_edges;

	// internal stuff
	std::vector<const std::pair<MotorID, arma::colvec2>*> buildPolygonSub(std::vector<const std::pair<MotorID, arma::colvec2>*> nodes, arma::colvec2 lineBegin, arma::colvec2 lineEnd) const;

	arma::mat m_vectorToplaneTransform;
};

}

#endif /* KINEMATICTREESUPPORTPOLYGON_H_ */
