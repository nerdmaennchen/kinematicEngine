/*
w * kinematicEngineTaskOrientation.cpp
 *
 *  Created on: 14.02.2014
 *      Author: lutz
 */

#include "kinematicEngineTaskOrientation.h"
#include "kinematicEngineTaskMethodPoint.h"

KinematicEngineTaskOrientation::KinematicEngineTaskOrientation() : KinematicEngineTask(), m_axis(AXIS_X), m_referenceCoordinateSystem(MOTOR_NONE)
{
}

KinematicEngineTaskOrientation::KinematicEngineTaskOrientation(std::string name, MotorID baseNode, MotorID effectorNode, const KinematicTree &tree, Axis axis) :
				KinematicEngineTask(name, baseNode, effectorNode, tree),
		m_axis(axis),
		m_referenceCoordinateSystem(baseNode),
		m_methodPoint()
{
	setMethod(&m_methodPoint);
}

KinematicEngineTaskOrientation::KinematicEngineTaskOrientation(std::string name, MotorID baseNode, MotorID effectorNode, MotorID referenceCoordinateSystem, const KinematicTree &tree, Axis axis) :
		KinematicEngineTask(name, baseNode, effectorNode, tree),
		m_axis(axis),
		m_referenceCoordinateSystem(referenceCoordinateSystem),
		m_methodPoint()
{
	setMethod(&m_methodPoint);
}

KinematicEngineTaskOrientation::KinematicEngineTaskOrientation(std::string name, MotorID baseNode, MotorID effectorNode, MotorID referenceCoordinateSystem, const KinematicTree &tree, arma::colvec3 target, Axis axis) :
		KinematicEngineTask(name, baseNode, effectorNode, tree),
		m_axis(axis),
		m_referenceCoordinateSystem(referenceCoordinateSystem),
		m_methodPoint(target)
{
	setMethod(&m_methodPoint);
}


KinematicEngineTaskOrientation::~KinematicEngineTaskOrientation() {
}

arma::mat KinematicEngineTaskOrientation::getJacobianForTask(const KinematicTree &kinematicTree, arma::mat &jacobianWithoutRemovedDOFs, bool normalizeJacobian) const
{
	arma::mat jacobian = arma::zeros(getDimensionCnt(), kinematicTree.getMotorCt());

	arma::mat33 forward = arma::eye(3, 3);
	arma::mat33 intermediateTransition = arma::eye(3, 3);

	for (uint32_t pathIndex = 0; pathIndex < m_invPath.size(); ++pathIndex)
	{
		const KinematicNode *node = m_invPath[pathIndex].m_node;
		arma::colvec3 orientationOfNode = forward.col(m_axis);

		if ((false == node->isFixedNode()) &&
			(KinematicPathNode::Direction::LINK != m_invPath[pathIndex].m_direction) &&
			(pathIndex < m_invPath.size() - 1))
		{
			arma::colvec3 partialDerivative = node->getPartialDerivativeOfOrientationToEffector(orientationOfNode);
			if (KinematicPathNode::Direction::FROM_PARENT == m_invPath[pathIndex].m_direction)
			{
				/* in this case the rotation axis of the joint is inverted... */
				partialDerivative = -partialDerivative;
			}
			jacobian.col(kinematicTree.toInt(node->getID())) = partialDerivative;
		}

		if (pathIndex < m_invPath.size() - 1)
		{
			const KinematicNode *nextNode = m_invPath[pathIndex + 1].m_node;
			intermediateTransition = node->getInvMatrixToRelative(nextNode->getID()).submat(0, 0, 2, 2);
			forward = intermediateTransition * forward;

			/* update the previously calculated entries in the jacobian according to our current orientation */
			jacobian = intermediateTransition * jacobian;
		}
	}

	/* transform the jacobian into the reference coordinate system */
	arma::mat33 transformToReferenceSystem = kinematicTree.getTransitionMatrixFromTo(m_referenceCoordinateSystem, m_baseNode).submat(0, 0, 2, 2);

	jacobian = transformToReferenceSystem * jacobian;

	jacobianWithoutRemovedDOFs = jacobian;
	removeDOFfromJacobian(jacobian, kinematicTree);

	return jacobian;
}


arma::colvec KinematicEngineTaskOrientation::getError(const KinematicTree &kinematicTree) const {
	/* calculate the orientation of the effector */
	const arma::mat44 transition = kinematicTree.getTransitionMatrixFromTo(m_referenceCoordinateSystem, m_effectorNode);
	arma::colvec3 value = m_method->getTransform() * transition.col(m_axis).rows(0, 2);
	double norm = arma::dot(m_method->getTarget(), value);


	if (0. < m_precision)
	{
		if (norm <= m_precision)
		{
			/* this means target reached */
			value = m_method->getTarget();
		}
	}
	return m_method->getTarget() - value;

}

void KinematicEngineTaskOrientation::setTarget(arma::colvec3 target)
{
	setMethod(&m_methodPoint);
	m_methodPoint.setTarget(target);
}
