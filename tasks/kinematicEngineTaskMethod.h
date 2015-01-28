/*
 * kinematicEngineTaskMethod.h
 *
 *  Created on: 24.11.2014
 *      Author: lutz
 */

#ifndef TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHOD_H_
#define TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHOD_H_

#include <armadillo>

class KinematicEngineTaskMethod {
public:
	KinematicEngineTaskMethod() {}
	virtual ~KinematicEngineTaskMethod() {}

	/**
	 * get the transformation for this method.
	 * When this method corresponds to a line the transform projects a 3d point to a plane orthogonal to the line
	 * @return
	 */
	virtual arma::mat const& getTransform() const = 0;

	/**
	 * what is the target of this method.
	 * The target vector for a line method would be a 2d coordinate in the plane orthogonal to the line (transform * target)
	 * @return
	 */
	virtual arma::colvec const& getTarget() const = 0;
};

#endif /* TOOLS_KINEMATICENGINE_TASKS_KINEMATICENGINETASKMETHOD_H_ */
