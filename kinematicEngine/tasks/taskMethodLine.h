/*
 *  Created on: 24.11.2014
 *      Author: lutz
 */

#ifndef TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHODLINE_H_
#define TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHODLINE_H_

#include "taskMethod.h"

namespace kinematicEngine {

class TaskMethodLine : public TaskMethod {
public:
	TaskMethodLine();
	virtual ~TaskMethodLine();

	virtual arma::mat const& getTransform() const override;

	virtual arma::colvec const& getTarget() const override;

	/**
	 * the direction of the line
	 * @param direction
	 */
	void setLineParams(arma::colvec3 direction, arma::colvec3 position);

protected:
	arma::mat m_transform;

	arma::colvec2 m_target;
};

}

#endif /* TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHODLINE_H_ */
