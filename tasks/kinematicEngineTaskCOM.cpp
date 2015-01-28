/*
 * kinematicTaskCOM.cpp
 *
 *  Created on: 25.02.2014
 *      Author: lutz
 */

#include <tools/kinematicEngine/tasks/kinematicEngineTaskCOM.h>


KinematicEngineTaskCOM::KinematicEngineTaskCOM() : KinematicEngineTask(),
		m_referenceNode(EFFECTOR_ID_ROOT)
{

}

KinematicEngineTaskCOM::KinematicEngineTaskCOM(std::string name, MotorID baseNode, MotorID effectorNode, MotorID referenceCoordinate, const KinematicTree &tree)
		: KinematicEngineTask(name, baseNode, effectorNode, tree),
		m_referenceNode(referenceCoordinate)
{
}

KinematicEngineTaskCOM::~KinematicEngineTaskCOM() {
}



/**
 * get the jacobian for this task
 * @param kinematicTree
 * @return
 */
arma::mat KinematicEngineTaskCOM::getJacobianForTask(const KinematicTree &kinematicTree, arma::mat &jacobianWithoutRemovedDOFs, bool normalizeJacobian) const
{
	arma::mat jacobian = arma::zeros(3, kinematicTree.getMotorCt());
	arma::mat retJacobian = arma::zeros(getDimensionCnt(), kinematicTree.getMotorCt());

	if (false != hasTarget())
	{
		arma::mat44 forward = arma::eye(4, 4);
		arma::mat44 intermediateTransition = arma::eye(4, 4);

		KinematicMass com;

		for (uint32_t pathIndex = 0; pathIndex < m_invPath.size(); ++pathIndex)
		{
			const KinematicNode *node = m_invPath[pathIndex].m_node;
			KinematicMass equivalentMass = node->getEquivalentMass();

			/* transform com into the node's coordinate frame */
			arma::colvec4 helper;
			helper.rows(0, 2) = com.m_position;
			helper.row(3) = 1.;
			helper = forward * helper;


			/* weight the com to this node */
			com.m_position = helper.rows(0, 2) * com.m_massGrams;
			equivalentMass.m_position = equivalentMass.m_position * equivalentMass.m_massGrams;
			com += equivalentMass;
			if (com.m_massGrams > 0.)
			{
				com.m_position = com.m_position * 1. / com.m_massGrams;
			}

			arma::colvec3 vecToCom = com.m_position;


			if (false != node->isFixedNode())
			{
				arma::colvec3 partialDerivative = node->getPartialDerivativeOfLocationToEffector(vecToCom) * com.m_massGrams;
				jacobian.col(kinematicTree.toExt(node->getID())) = partialDerivative;
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


		com.m_position = com.m_position * 1. / com.m_massGrams;


		/* align in reference coordinate frame */
		const arma::mat44 baseToRefTransform = kinematicTree.getTransitionMatrixFromTo(m_referenceNode, m_baseNode);
		jacobian = baseToRefTransform.submat(0, 0, 2, 2) * jacobian;

		retJacobian = m_method->getTransform() * jacobian;
		jacobianWithoutRemovedDOFs = retJacobian;

		/* remove all motors from the jacobian which shall not be used */
		removeDOFfromJacobian(retJacobian, kinematicTree);

		if (normalizeJacobian) {
			normJacobian(retJacobian);
		}
	}


	return retJacobian;
}

arma::colvec KinematicEngineTaskCOM::getError(const KinematicTree &kinematicTree) const
{
	KinematicMass com = kinematicTree.getCOM(m_baseNode, m_effectorNode);

	arma::colvec value = com.m_position;
	arma::colvec target = m_method->getTarget();

	const arma::mat44 baseToRefTransform = kinematicTree.getTransitionMatrixFromTo(m_referenceNode, m_baseNode);
	value = m_method->getTransform() * baseToRefTransform.submat(0, 0, 2, 2) * value;

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
