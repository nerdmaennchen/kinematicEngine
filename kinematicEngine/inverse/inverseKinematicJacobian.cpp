/*
 * inverseKinematicJacobian.cpp
 *
 *  Created on: 13.06.2014
 *      Author: lutz
 */

#include "inverseKinematicJacobian.h"
#include "kinematicEngine/utils/utils.h"

#define DEFAULT_EPSILON 1.

namespace kinematicEngine {

InverseKinematicJacobian::InverseKinematicJacobian()
	: InverseKinematics()
	, m_epsilon(DEFAULT_EPSILON)
	, m_speedEpsilon(DEFAULT_EPSILON)
	, m_nullspaceEpsilon(DEFAULT_EPSILON)
{
}

InverseKinematicJacobian::~InverseKinematicJacobian()
{
}


double InverseKinematicJacobian::iterationStep(
		KinematicTree const& tree
		, MotorValuesMap& o_values
		, TasksContainer const& alltasks
		, Constraints const& constraints
		, const TaskDefaultPosition* idleTask
	) const
{
	double totalError = 0;
	const uint motorCnt = tree.getMotorCnt();

	arma::colvec motorValuesVec = arma::zeros(motorCnt);
	std::map<NodeID, KinematicNode*> const& nodes = tree.getNodes();

	for (std::pair<NodeID, KinematicNode*> const& node : nodes) {
		kinematics::Motor2IntMap const& motor2Int = node.second->getMotor2IntMap();
		MotorValuesMap const& values = node.second->getValues();

		for (std::pair<MotorID, double> const& value : values) {
			motorValuesVec(motor2Int.at(value.first)) = value.second;
		}
	}

	arma::colvec summedJointAngleDiffs = arma::zeros(motorCnt);
	arma::mat nullspaceEyeMat = arma::eye(motorCnt, motorCnt);
	arma::mat nullspaceMat = arma::zeros(motorCnt, motorCnt);

	arma::mat bigJacobian;
	arma::colvec bigErrorVector;
	for (std::pair<uint, std::vector<const Task*>> const& group : alltasks) {
		std::vector<const Task*> const& tasks = group.second;

		if (tasks.size() > 0) {
			const arma::mat dofMat = nullspaceEyeMat - nullspaceMat;
			arma::mat jacobianRAW;
			const arma::mat jacobian = getJacobianForTasks(tree, tasks, jacobianRAW, false);
			arma::mat jacobianDOF = jacobian * arma::diagmat(dofMat);

			arma::colvec errorVec = getErrorForTasks(tree, tasks);

			arma::mat pseudoInverseJacobian = buildPseudoInverse(jacobianDOF, m_epsilon);

			const arma::colvec jointDiffs = pseudoInverseJacobian * errorVec;
			const arma::colvec jointDiffsDecorr = dofMat * jointDiffs;

			const arma::colvec preliminaryJointDiffs = summedJointAngleDiffs + jointDiffsDecorr;
			const arma::colvec preliminaryJointAngles = motorValuesVec + preliminaryJointDiffs;
			arma::colvec jointDiffConstraintError = arma::zeros(motorCnt);
			arma::colvec correctedJointAngleDiffs = jointDiffsDecorr;

			for (const Constraint* constraint : constraints) {
				jointDiffConstraintError += constraint->getAngleErrorForConstraint(tree, preliminaryJointAngles);
			}

			// correct the constraint error
			for (uint i(0); i < motorCnt; ++i) {
				const double factor = dofMat(i, i);
				if (factor > m_nullspaceEpsilon) {
					jointDiffConstraintError(i) = jointDiffConstraintError(i) / factor;
				} else {
					jointDiffConstraintError(i) = 0;
				}
			}

			correctedJointAngleDiffs += dofMat * jointDiffConstraintError;

			summedJointAngleDiffs += correctedJointAngleDiffs;

			nullspaceMat += dofMat * (buildPseudoInverse(jacobianRAW, m_nullspaceEpsilon) * jacobianRAW);
			totalError += arma::dot(errorVec, errorVec);

//			std::cout.precision(3);
//			std::cout << std::fixed << std::setw(7);
//			jacobian.raw_print(std::cout); printf("\n");
//			summedJointAngleDiffs.t().raw_print(std::cout); printf("\n");
//			errorVec.t().raw_print(std::cout); printf("\n");
//			std::cout << arma::norm(errorVec, 2) << std::endl;
//			printf("\n");
		}
	}

	if (nullptr != idleTask) {
		arma::mat helper = arma::mat(nullspaceEyeMat - nullspaceMat);
		arma::colvec errorVec = idleTask->getErrors(tree);
		arma::colvec angleDiffVec = ((nullspaceEyeMat - nullspaceMat) * errorVec);

		summedJointAngleDiffs += angleDiffVec;
	}

	for (uint32_t i = 0; i < summedJointAngleDiffs.n_rows; ++i)
	{
		summedJointAngleDiffs(i) = utils::limited(summedJointAngleDiffs(i), -m_maxValueChange, m_maxValueChange);
	}

	motorValuesVec += summedJointAngleDiffs;

	for (std::pair<NodeID, KinematicNode*> const& node : nodes) {
		kinematics::Int2MotorMap const& int2MotorMap = node.second->getInt2MotorMap();
		kinematics::MotorIDs const& motors = node.second->getMotors();
		MotorValuesMap newValues;

		for (kinematics::Int2Motor const& i2m : int2MotorMap) {
			newValues[i2m.second] = motorValuesVec(i2m.first);
		}

		/* clip values */
		newValues = node.second->clipValues(newValues);

		/* export */
		for (MotorID const& motor : motors) {
			o_values[motor] = newValues[motor];
		}
	}

	return totalError;
}

double InverseKinematicJacobian::calculateSpeeds(
		KinematicTree const& tree
		, MotorValuesMap& o_values
		, TasksContainer const& tasks
		, Constraints const& constraints
		, const TaskDefaultPosition* idleTask
	) const
{
	double totalError = 0;
	uint motorCnt = tree.getMotorCnt();

	arma::colvec motorSpeedsVec = arma::zeros(motorCnt);
	arma::colvec motorValuesVec = arma::zeros(motorCnt);

	std::map<NodeID, KinematicNode*> const& nodes = tree.getNodes();

	for (std::pair<NodeID, KinematicNode*> const& node : nodes) {
		kinematics::Motor2IntMap const& motor2Int = node.second->getMotor2IntMap();
		MotorValuesMap const& values = node.second->getValues();
		MotorValuesMap const& valueDerivatives = node.second->getValueDerivatives();

		for (std::pair<MotorID, double> const& value : values) {
			motorValuesVec(motor2Int.at(value.first)) = value.second;
		}
		for (std::pair<MotorID, double> const& value : valueDerivatives) {
			motorSpeedsVec(motor2Int.at(value.first)) = value.second;
		}
	}

	arma::colvec jointDiffConstraintError = arma::zeros(motorCnt);
	for (const Constraint* constraint : constraints) {
		jointDiffConstraintError += constraint->getAngleErrorForConstraint(tree, motorValuesVec);
	}

	arma::colvec summedJointSpeedDiffs = arma::zeros(motorCnt);
	arma::mat nullspaceEyeMat = arma::eye(motorCnt, motorCnt);
	arma::mat nullspaceMat = jointDiffConstraintError * jointDiffConstraintError.t(); //arma::zeros(motorCnt, motorCnt);

	arma::mat bigJacobian;
	for (std::pair<uint, std::vector<const Task*>> const& group : tasks) {
		std::vector<const Task*> const& _tasks = group.second;

		if (_tasks.size() > 0) {
			arma::mat dofMat = nullspaceEyeMat - nullspaceMat;

			arma::mat jacobianRAW;
			arma::mat jacobian = getJacobianForTasks(tree, _tasks, jacobianRAW, false);
			arma::mat jacobianDOF = jacobian * arma::diagmat(dofMat);

			// how much "error" (mother function)
			arma::colvec speedTarget = getSpeedTargetForTasks(tree, _tasks);

			// how fast does the target move
			arma::colvec speedVec = jacobian * motorSpeedsVec;

			// the actual error
			arma::colvec speedError = speedTarget - speedVec;

//			const arma::colvec jointDiffs = arma::mat(jacobian * dofMat).t() * speedError;
			const arma::colvec jointDiffs = dofMat * buildPseudoInverse(jacobianDOF, m_speedEpsilon) * speedError;

			summedJointSpeedDiffs += jointDiffs;
			nullspaceMat += dofMat * (buildPseudoInverse(jacobianRAW, m_nullspaceEpsilon) * jacobianRAW);
			totalError += arma::dot(speedError, speedError);

//			std::cout.precision(3);
//			std::cout << std::fixed << std::setw(7);
//			jacobianDOF.raw_print(std::cout); printf("\n");
//			buildPseudoInverse(jacobianDOF, m_speedEpsilon).raw_print(std::cout); printf("\n");
//			summedJointSpeedDiffs.raw_print(std::cout); printf("\n");
//			speedError.raw_print(std::cout); printf("\n");
//			std::cout << arma::norm(speedError, 2) << std::endl;
//			printf("\n");
		}
	}

	if (nullptr != idleTask) {
		arma::mat helper = arma::mat(nullspaceEyeMat - nullspaceMat);
		arma::colvec errorVec = idleTask->getErrors(tree);
		summedJointSpeedDiffs += (helper * errorVec) * idleTask->getSpeed();
	}

	summedJointSpeedDiffs += motorSpeedsVec;

	for (std::pair<NodeID, KinematicNode*> const& node : nodes) {
		kinematics::Int2MotorMap const& int2MotorMap = node.second->getInt2MotorMap();
		kinematics::MotorIDs const& motors = node.second->getMotors();
		MotorValuesMap newValues;

		for (kinematics::Int2Motor const& i2m : int2MotorMap) {
			newValues[i2m.second] = summedJointSpeedDiffs(i2m.first);
		}

		/* export */
		for (MotorID const& motor : motors) {
			o_values[motor] = newValues[motor];
		}
	}

	return totalError;
}


arma::mat InverseKinematicJacobian::getJacobianForTasks(KinematicTree const& tree, std::vector<const Task*> const& tasks, arma::mat &jacobianRAW, bool normalize) const
{
	const uint32_t numTasks = tasks.size();
	uint numCols = tree.getMotorCnt();

	arma::mat jacobian = arma::zeros(1, numCols);
	if ((0 < numTasks) &&
		(0 < numCols)) /* at least one task and at least one motor */
	{
		/* calculate the size of the jacobian and the task vector */
		uint32_t numRows = 0;
		for (const Task* const &task : tasks)
		{
			if (task->hasTarget()) {
				numRows += task->getDimensionCnt();
			}
		}

		/* build the "big" jacobian */
		jacobian = arma::zeros(numRows, numCols);
		jacobianRAW = arma::zeros(numRows, numCols);

		uint32_t beginRow = 0;
		for (const Task *const &task : tasks)
		{
			if (task->hasTarget())
			{
				const uint32_t endRow = beginRow + task->getDimensionCnt() - 1;
				arma::mat jacobianRawSub;
				jacobian.submat(beginRow, 0, endRow, numCols - 1) = task->getJacobianForTask(tree, jacobianRawSub, normalize);
				jacobianRAW.submat(beginRow, 0, endRow, numCols - 1) = jacobianRawSub;

				beginRow = endRow + 1;
			}
		}
	}

	return jacobian;
}

arma::colvec InverseKinematicJacobian::getErrorForTasks(KinematicTree const& tree, std::vector<const Task*> const& tasks) const
{
	const uint32_t numTasks = tasks.size();

	arma::colvec errorVec = arma::zeros(1);
	if (0 < numTasks) /* at least one task */
	{
		uint32_t numRows = 0;
		for (const Task* const &task : tasks)
		{
			if (task->hasTarget()) {
				numRows += task->getDimensionCnt();
			}
		}
		errorVec = arma::zeros(numRows);

		uint32_t beginRow = 0;
		for (const Task *const &task : tasks)
		{
			if (task->hasTarget())
			{
				const uint32_t endRow = beginRow + task->getDimensionCnt() - 1;
				arma::colvec error = task->getError(tree);
				errorVec.rows(beginRow, endRow) = error * task->getWeight();
				beginRow = endRow + 1;
			}
		}
	}

	return errorVec;
}

arma::colvec InverseKinematicJacobian::getSpeedTargetForTasks(KinematicTree const& tree, std::vector<const Task*> const& tasks) const
{
	const uint32_t numTasks = tasks.size();

	arma::colvec errorVec = arma::zeros(1);
	if (0 < numTasks) /* at least one task */
	{
		uint32_t numRows = 0;
		for (const Task* const &task : tasks)
		{
			if (task->hasTarget()) {
				numRows += task->getDimensionCnt();
			}
		}
		errorVec = arma::zeros(numRows);

		uint32_t beginRow = 0;
		for (const Task *const &task : tasks)
		{
			if (task->hasTarget())
			{
				const uint32_t endRow = beginRow + task->getDimensionCnt() - 1;
				arma::colvec error = task->getError(tree) * task->getWeight();

				if (task->hasSpeed() && arma::norm(error, 2) > 0.000000001 ) {
					error *= task->getSpeedToReachTarget() / arma::norm(error, 2);
				}

				errorVec.rows(beginRow, endRow) = error;
				beginRow = endRow + 1;
			}
		}
	}

	return errorVec;
}

int InverseKinematicJacobian::calculateNumRows(std::vector<const Task*> const& tasks) const
{
	int numRowsConstraints = 0;
	for (const Task* const &task : tasks)
	{
		if (task->hasTarget()) {
			numRowsConstraints += task->getDimensionCnt();
		}
	}
	return numRowsConstraints;
}


arma::mat InverseKinematicJacobian::buildPseudoInverse(arma::mat matrix, double epsilon) const
{
	const arma::mat helper = matrix * matrix.t();
	const arma::mat I = arma::eye(helper.n_rows, helper.n_cols);
	const arma::mat pseudoInverseJacobian = matrix.t() * arma::inv((helper + epsilon * I));
	return pseudoInverseJacobian;
}

}
