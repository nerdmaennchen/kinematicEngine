/*
 * KinematicEngineTaskLocation.cpp
 *
 *  Created on: 14.02.2014
 *      Author: lutz
 */

#include <tools/kinematicEngine/tasks/kinematicEngineTaskLocation.h>
#include <armadillo>

KinematicEngineTaskLocation::KinematicEngineTaskLocation() : KinematicEngineTask()
{
}

KinematicEngineTaskLocation::KinematicEngineTaskLocation(std::string name, MotorID baseNode, MotorID effectorNode, const KinematicTree &tree)
	: KinematicEngineTask(name, baseNode, effectorNode, tree)
	, m_referenceCoordinateSystem(effectorNode)
{
}

KinematicEngineTaskLocation::KinematicEngineTaskLocation(std::string name, MotorID baseNode, MotorID effectorNode, MotorID referenceNode, const KinematicTree &tree)
	: KinematicEngineTask(name, baseNode, effectorNode, tree)
	, m_referenceCoordinateSystem(referenceNode)
{

}

KinematicEngineTaskLocation::KinematicEngineTaskLocation(std::string name, MotorID baseNode, MotorID effectorNode, const KinematicTree &tree, KinematicEngineTaskMethod *method)
	: KinematicEngineTask(name, baseNode, effectorNode, tree, method)
	, m_referenceCoordinateSystem(effectorNode)
{
}

KinematicEngineTaskLocation::~KinematicEngineTaskLocation()
{
}

arma::mat KinematicEngineTaskLocation::getJacobianForTask(const KinematicTree &kinematicTree, arma::mat &jacobianWithoutRemovedDOFs, bool normalizeJacobian) const
{
	arma::mat jacobian = arma::zeros(3, kinematicTree.getMotorCt());
	arma::mat retJacobian = arma::zeros(getDimensionCnt(), kinematicTree.getMotorCt());

	if (false != hasTarget())
	{
		arma::mat44 forward = arma::eye(4, 4);
		arma::mat44 intermediateTransition = arma::eye(4, 4);

		for (uint32_t pathIndex = 0; pathIndex < m_invPath.size(); ++pathIndex)
		{
			const KinematicNode *node = m_invPath[pathIndex].m_node;

			arma::colvec3 vecToEndeffector = forward.col(3).rows(0, 2);

			if ((false == node->isFixedNode()) &&
				(KinematicPathNode::Direction::LINK != m_invPath[pathIndex].m_direction))
			{
				arma::colvec3 partialDerivative = node->getPartialDerivativeOfLocationToEffector(vecToEndeffector);
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
				intermediateTransition = node->getInvMatrixToRelative(nextNode->getID());
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


arma::colvec KinematicEngineTaskLocation::getError(const KinematicTree &kinematicTree) const {
	/* calculate the position of the effector */
	const arma::mat44 transition = kinematicTree.getTransitionMatrixFromTo(m_baseNode, m_effectorNode);
	const arma::mat44 transition2 = kinematicTree.getTransitionMatrixFromTo(m_referenceCoordinateSystem, m_effectorNode);
	arma::colvec3 value = m_method->getTransform() * (transition2.submat(0, 0, 2, 2) * transition.col(3).rows(0, 2));

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

