/*
 * kinematicEngineTaskCOMWholeRobot.cpp
 *
 *  Created on: 28.02.2014
 *      Author: lutz
 */

#include "kinematicEngineTaskCOMWholeRobot.h"

KinematicEngineTaskCOMWholeRobot::KinematicEngineTaskCOMWholeRobot()
	: KinematicEngineTask()
{
}

KinematicEngineTaskCOMWholeRobot::KinematicEngineTaskCOMWholeRobot(std::string name, MotorID base, KinematicTree const &tree)
	: KinematicEngineTask(name, base, MOTOR_NONE, tree)
	, m_referenceCoordinateSystem(base)
{
}

KinematicEngineTaskCOMWholeRobot::KinematicEngineTaskCOMWholeRobot(std::string name, MotorID base, MotorID reference, KinematicTree const &tree)
	: KinematicEngineTask(name, base, MOTOR_NONE, tree)
	, m_referenceCoordinateSystem(reference)
{

}

KinematicEngineTaskCOMWholeRobot::~KinematicEngineTaskCOMWholeRobot()
{
}

/**
 * get the jacobian for this task
 * @param kinematicTree
 * @return
 */
arma::mat KinematicEngineTaskCOMWholeRobot::getJacobianForTask(const KinematicTree &kinematicTree, arma::mat &jacobianWithoutRemovedDOFs, bool normalizeJacobian) const
{
	arma::mat retJacobian = arma::zeros(getDimensionCnt(), kinematicTree.getMotorCt());

	if (false != hasTarget())
	{
		arma::mat jacobian = arma::zeros(3, kinematicTree.getMotorCt());

		const KinematicNode *node = kinematicTree.getNode(m_baseNode);

		KinematicMass massFromParent, massFromChildren;

		for (const KinematicNode* child: node->getChildren()) {
			jacobian += child->getForwardMatrix().submat(0, 0, 2, 2) * getJacobianForTaskSub(kinematicTree, node, false, massFromChildren);
		}

		jacobian += getJacobianForTaskSub(kinematicTree, node, true, massFromChildren);

		retJacobian = m_method->getTransform() * jacobian;
		jacobianWithoutRemovedDOFs = retJacobian;
		removeDOFfromJacobian(retJacobian, kinematicTree);
		if (normalizeJacobian) {
			normJacobian(retJacobian);
		}
	}

	return retJacobian;
}


arma::mat KinematicEngineTaskCOMWholeRobot::getJacobianForTaskSub(const KinematicTree &kinematicTree,
		const KinematicNode *node,
		bool traversingUp,
		KinematicMass &o_massFromSubTree) const
{
	KinematicMass attachedNodesMass;
	arma::mat retJacobian = arma::zeros(3, kinematicTree.getMotorCt());
	if (traversingUp) {
		const KinematicNode* parent = node->getParent();
		if (nullptr != parent) {
			const arma::mat33 parentToNodeRotMat = node->getBackwardMatrix().submat(0, 0, 2, 2);
			KinematicMass parentsMass = parent->getEquivalentMass();
			retJacobian = parentToNodeRotMat * getJacobianForTaskSub(kinematicTree, parent, true, parentsMass);

			for (const KinematicNode* sibling : parent->getChildren()) {
				if (sibling != node) {
					KinematicMass siblingsMass;
					retJacobian += parentToNodeRotMat * sibling->getForwardMatrix().submat(0, 0, 2, 2) * getJacobianForTaskSub(kinematicTree, sibling, false, siblingsMass);
					siblingsMass.applyTransformation(sibling->getForwardMatrix());
					attachedNodesMass += siblingsMass;
				}
			}

			attachedNodesMass += parent->getEquivalentMass() + parentsMass;
			attachedNodesMass.applyTransformation(node->getBackwardMatrix());
			o_massFromSubTree += attachedNodesMass;

			if (!node->isFixedNode()) {
				const int idx = kinematicTree.toInt(node->getID());
				arma::colvec3 momentumDerivative = -node->getPartialDerivativeOfLocationToEffector(attachedNodesMass.m_position) * attachedNodesMass.m_massGrams / 1000.;
				retJacobian.col(idx) = momentumDerivative;
			}
		}
	} else {
		KinematicMass massmovedByNode;
		for (const KinematicNode *child : node->getChildren()) {
			KinematicMass childMass;
			retJacobian += child->getForwardMatrix().submat(0, 0, 2, 2) * getJacobianForTaskSub(kinematicTree, child, false, childMass);
			childMass.applyTransformation(child->getForwardMatrix());
			massmovedByNode += childMass;
		}
		massmovedByNode += node->getEquivalentMass();

		if (!node->isFixedNode()) {
			const int idx = kinematicTree.toInt(node->getID());
			arma::colvec3 momentumDerivative = node->getPartialDerivativeOfLocationToEffector(massmovedByNode.m_position) * massmovedByNode.m_massGrams / 1000.;

			retJacobian.col(idx) = momentumDerivative;
		}

		o_massFromSubTree += massmovedByNode;
	}
	return retJacobian;
}


arma::colvec KinematicEngineTaskCOMWholeRobot::getError(const KinematicTree &kinematicTree) const
{
	arma::mat33 transformToReferenceSystem = kinematicTree.getTransitionMatrixFromTo(m_referenceCoordinateSystem, m_baseNode).submat(0, 0, 2, 2);
	arma::colvec3 actualCOM = transformToReferenceSystem * kinematicTree.getCOM(m_baseNode).m_position;

	/* get target */
	arma::colvec target = m_method->getTarget();
	arma::colvec value = m_method->getTransform() * actualCOM;

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

