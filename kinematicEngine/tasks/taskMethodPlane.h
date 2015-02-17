/*
 *  Created on: 24.11.2014
 *      Author: lutz
 */

#ifndef TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHODPLANE_H_
#define TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHODPLANE_H_

#include "taskMethod.h"

namespace kinematicEngine {

class TaskMethodPlane : public TaskMethod {
public:
	TaskMethodPlane();
	virtual ~TaskMethodPlane();

	virtual arma::mat const& getTransform() const override;

	virtual arma::colvec const& getTarget() const override;

	void setPlaneParams(arma::colvec3 normalVec, arma::colvec3 supportPoint);

protected:
	arma::mat m_transform;

	arma::colvec m_target;
};

}
#endif /* TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHODPLANE_H_ */
