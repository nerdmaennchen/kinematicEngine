/*
 * kinematicEngineTaskMethodPoint.h
 *
 *  Created on: 24.11.2014
 *      Author: lutz
 */

#ifndef TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHODPOINT_H_
#define TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHODPOINT_H_

#include "kinematicEngineTaskMethod.h"

class KinematicEngineTaskMethodPoint : public KinematicEngineTaskMethod {
public:
	KinematicEngineTaskMethodPoint();
	KinematicEngineTaskMethodPoint(arma::colvec3 target);
	virtual ~KinematicEngineTaskMethodPoint();

	virtual arma::mat const& getTransform() const override;

	virtual arma::colvec const& getTarget() const override;

	void setTarget(arma::colvec3 target);

private:

	arma::colvec3 m_target;
	arma::mat33 m_transform;
};

#endif /* TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHODPOINT_H_ */
