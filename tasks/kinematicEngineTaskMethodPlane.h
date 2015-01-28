/*
 * kinematicEngineTaskMethodPlane.h
 *
 *  Created on: 24.11.2014
 *      Author: lutz
 */

#ifndef TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHODPLANE_H_
#define TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHODPLANE_H_

#include "kinematicEngineTaskMethod.h"

class KinematicEngineTaskMethodPlane : public KinematicEngineTaskMethod {
public:
	KinematicEngineTaskMethodPlane();
	virtual ~KinematicEngineTaskMethodPlane();

	virtual arma::mat const& getTransform() const override;

	virtual arma::colvec const& getTarget() const override;

	void setPlaneParams(arma::colvec3 normalVec, arma::colvec3 supportPoint);

protected:
	arma::mat m_transform;

	arma::colvec m_target;
};

#endif /* TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHODPLANE_H_ */
