/*
 *  Created on: 14.02.2014
 *      Author: lutz
 */

#include "taskLocation.h"
#include <armadillo>

namespace kinematicEngine {


TaskLocation::TaskLocation() : Task()
{
}

TaskLocation::TaskLocation(std::string name, NodeID baseNode, NodeID effectorNode, const KinematicTree &tree)
	: Task(name, baseNode, effectorNode, tree)
	, m_referenceCoordinateSystem(baseNode)
{
}

TaskLocation::TaskLocation(std::string name, NodeID baseNode, NodeID effectorNode, NodeID referenceNode, const KinematicTree &tree)
	: Task(name, baseNode, effectorNode, tree)
	, m_referenceCoordinateSystem(referenceNode)
{

}

TaskLocation::TaskLocation(std::string name, NodeID baseNode, NodeID effectorNode, const KinematicTree &tree, TaskMethod *method)
	: Task(name, baseNode, effectorNode, tree, method)
	, m_referenceCoordinateSystem(baseNode)
{
}

TaskLocation::~TaskLocation()
{
}

arma::mat TaskLocation::getJacobianForTask(const KinematicTree &kinematicTree, arma::mat &jacobianWithoutRemovedDOFs, bool normalizeJacobian) const
{
	arma::mat jacobian = arma::zeros(3, kinematicTree.getMotorCnt());
	arma::mat retJacobian = arma::zeros(getDimensionCnt(), kinematicTree.getMotorCnt());

	if (false != hasTarget())
	{
		arma::mat44 forward = arma::eye(4, 4);
		arma::mat44 intermediateTransition = arma::eye(4, 4);

		for (uint32_t pathIndex = 0; pathIndex < m_invPath.size(); ++pathIndex)
		{
			const KinematicNode *node = m_invPath[pathIndex].m_node;

			arma::colvec3 vecToEndeffector = forward.col(3).rows(0, 2);

			if ((false == node->isFixedNode()) &&
				(KinematicPathNode::Direction::LINK != m_invPath[pathIndex].m_direction) &&
				(pathIndex < m_invPath.size() - 1))
			{
				kinematics::JacobianValues partialDerivatives = node->getPartialDerivativeOfLocationToEffector(vecToEndeffector);

				if (KinematicPathNode::Direction::FROM_PARENT == m_invPath[pathIndex].m_direction)
				{
					/* in this case the rotation axis of the joint is inverted... */
					for (kinematics::JacobianValue value : partialDerivatives) {
						jacobian.col(value.first) = -value.second;
					}
				} else {
					for (kinematics::JacobianValue value : partialDerivatives) {
						jacobian.col(value.first) = value.second;
					}
				}
			}

			if (pathIndex < m_invPath.size() - 1)
			{
				const KinematicNode *nextNode = m_invPath[pathIndex + 1].m_node;
				intermediateTransition = node->getInvMatrixToRelative(nextNode);
				forward = intermediateTransition * forward;

				/* update the previously calculated entries in the jacobian according to our current orientation */
				jacobian = intermediateTransition.submat(0, 0, 2, 2) * jacobian;
			}
		}

		/* transform the jacobian into the reference coordinate system */
		arma::mat33 transformToReferenceSystem = kinematicTree.getTransitionMatrixFromTo(m_referenceCoordinateSystem, m_baseNode).submat(0, 0, 2, 2);
		jacobian = transformToReferenceSystem * jacobian;

		retJacobian = m_method->getTransform() * jacobian;
		jacobianWithoutRemovedDOFs = retJacobian;

		removeDOFfromJacobian(retJacobian, kinematicTree);
		if (normalizeJacobian) {
			normJacobian(retJacobian);
		}
	}


	return retJacobian;
}


arma::colvec TaskLocation::getError(const KinematicTree &kinematicTree) const {
	/* calculate the position of the effector */
	const arma::mat44 transition = kinematicTree.getTransitionMatrixFromTo(m_baseNode, m_effectorNode);
	const arma::mat44 transition2 = kinematicTree.getTransitionMatrixFromTo(m_referenceCoordinateSystem, m_baseNode);
	arma::colvec value = m_method->getTransform() * (transition2.submat(0, 0, 2, 2) * transition.col(3).rows(0, 2));

	arma::colvec target = m_method->getTarget();
	double norm = arma::norm((target - value), 2);

	if (0 < m_precision)
	{
		if (norm <= m_precision)
		{
			/* this means target reached */
			value = target;
		}
	}

	return target - value;
}

}
