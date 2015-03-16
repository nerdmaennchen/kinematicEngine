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
    std::cout << "\x1B[2J\x1B[H";
	double totalError = 0;
	const uint motorCnt = tree.getMotorCnt();

	arma::colvec motorValuesVec = arma::zeros(motorCnt);
	std::map<NodeID, KinematicNode*> const& nodes = tree.getNodes();

	for (std::pair<NodeID, KinematicNode*> const& node : nodes) {
		if (node.second->isServo())
		{
			kinematics::Motor2IntMap const& motor2Int = node.second->getMotor2IntMap();
			MotorValuesMap const& values = node.second->getValues();

			for (std::pair<MotorID, double> const& value : values) {
				motorValuesVec(motor2Int.at(value.first)) = value.second;
			}
		}
	}

	arma::colvec summedJointAngleDiffs = arma::zeros(motorCnt);
	arma::mat nullspaceEyeMat = arma::eye(motorCnt, motorCnt);
	arma::mat nullspaceMat = nullspaceEyeMat;

	arma::mat bigJacobian;
	for (std::pair<uint, std::vector<const Task*>> const& group : alltasks) {
		std::vector<const Task*> const& tasks = group.second;

		if (tasks.size() > 0) {
			arma::mat jacobianRAW;
			const arma::mat jacobian = getJacobianForTasks(tree, tasks, jacobianRAW, false);

			const arma::colvec errorVec = getErrorForTasks(tree, tasks);

			const arma::mat pseudoInverseJacobian = buildPseudoInverse(jacobian, m_speedEpsilon);

			const arma::colvec jointDiffs = pseudoInverseJacobian * errorVec;
			const arma::colvec jointDiffsDecorr = nullspaceMat * jointDiffs;

			const arma::colvec preliminaryJointAngles = motorValuesVec + summedJointAngleDiffs + jointDiffsDecorr;
			arma::colvec jointDiffConstraintError = arma::zeros(motorCnt);
			arma::colvec correctedJointAngleDiffs = jointDiffsDecorr;

			for (const Constraint* constraint : constraints) {
				jointDiffConstraintError += constraint->getAngleErrorForConstraint(tree, preliminaryJointAngles);
			}

			// correct the constraint error
			for (uint i(0); i < motorCnt; ++i) {
				const double factor = nullspaceMat(i, i);
				if (factor > m_nullspaceEpsilon) {
					jointDiffConstraintError(i) = jointDiffConstraintError(i) / factor;
				} else {
					jointDiffConstraintError(i) = 0;
				}
			}

			correctedJointAngleDiffs += nullspaceMat * jointDiffConstraintError;

			summedJointAngleDiffs += correctedJointAngleDiffs;

			bigJacobian = arma::join_cols(bigJacobian, jacobianRAW);
			nullspaceMat = nullspaceEyeMat - buildPseudoInverse(bigJacobian, m_nullspaceEpsilon) * bigJacobian;

			totalError += arma::dot(errorVec, errorVec);

//			std::cout << "level " << group.first << std::endl;
//			std::cout.precision(7);
//			std::cout << std::fixed << std::setw(12);
//			bigJacobian.raw_print(std::cout, "bigJacobian = ["); printf("]\n");
//			jacobian.raw_print(std::cout, std::string("jacobian") + std::to_string(group.first) + " = ["); printf("]\n");
//			bigJacobian.raw_print(std::cout, std::string("bigJacobian") + std::to_string(group.first) + " = ["); printf("]\n");
//			jacobianDOF.raw_print(std::cout, "jacobianDOF = ["); printf("]\n");
//			arma::mat(jacobian * dofMat).raw_print(std::cout, "jacobianDOF2 = ["); printf("]\n");
//			jointDiffs.t().raw_print(std::cout, std::string("jointDiffs") + std::to_string(group.first) + " = ["); printf("]\n");
//			jointDiffsDecorr.t().raw_print(std::cout, std::string("jointDiffsDecorr") + std::to_string(group.first) + " = ["); printf("]\n");
//			summedJointAngleDiffs.t().raw_print(std::cout, "summedJointAngleDiffs = ["); printf("]\n");
//			jointDiffConstraintError.t().raw_print(std::cout, "jointDiffConstraintError = ["); printf("]\n");
//			correctedJointAngleDiffs.t().raw_print(std::cout, "correctedJointAngleDiffs = ["); printf("]\n");
//			errorVec.t().raw_print(std::cout, std::string("errorVec") + std::to_string(group.first) + " = ["); printf("]'\n");
//			nullspaceMat.raw_print(std::cout, "nullspaceMat = ["); printf("]\n");
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
		if (node.second->isServo())
		{
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
		if (node.second->isServo())
		{
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
	}

	arma::colvec jointDiffConstraintError = arma::zeros(motorCnt);
	for (const Constraint* constraint : constraints) {
		jointDiffConstraintError += constraint->getAngleErrorForConstraint(tree, motorValuesVec);
	}

	arma::colvec summedJointSpeedDiffs = arma::zeros(motorCnt);
	arma::mat nullspaceEyeMat = arma::eye(motorCnt, motorCnt);
	arma::mat nullspaceMat = nullspaceEyeMat - jointDiffConstraintError * jointDiffConstraintError.t();

	arma::mat bigJacobian;
	for (std::pair<uint, std::vector<const Task*>> const& group : tasks) {
		std::vector<const Task*> const& _tasks = group.second;

		if (_tasks.size() > 0) {
			arma::mat jacobianRAW;
			arma::mat jacobian = getJacobianForTasks(tree, _tasks, jacobianRAW, false);

			const arma::mat pseudoInverseJacobian = buildPseudoInverse(jacobian, m_speedEpsilon);

			// how much "error" (mother function)
			arma::colvec speedTarget = getSpeedTargetForTasks(tree, _tasks);

			// how fast does the target move
			arma::colvec speedVec = jacobian * motorSpeedsVec;


			// the actual error
			arma::colvec speedError = speedTarget - speedVec;

			const arma::colvec jointDiffs = nullspaceMat * pseudoInverseJacobian * speedError;

			summedJointSpeedDiffs += jointDiffs;

			bigJacobian = arma::join_cols(bigJacobian, jacobianRAW);
//			nullspaceMat += dofMat * (buildPseudoInverse(jacobianRAW, m_nullspaceEpsilon) * jacobianRAW);
			nullspaceMat = nullspaceEyeMat - buildPseudoInverse(bigJacobian, m_nullspaceEpsilon) * bigJacobian;
			totalError += arma::dot(speedError, speedError);

//			printf("level %d\n", group.first);
//			std::cout.precision(3);
//			std::cout << std::fixed << std::setw(7);
//			jacobian.raw_print(std::cout, "jacobian"); printf("\n");
//			jacobianDOF.raw_print(std::cout, "jacobianDOF"); printf("\n");
//			pseudoInverseJacobian.raw_print(std::cout, "pseudoInverseJacobianDOF"); printf("\n");
//			buildPseudoInverse(jacobian, m_speedEpsilon).raw_print(std::cout, "pseudoInverseJacobian"); printf("\n");
//			jointDiffs.t().raw_print(std::cout, "jointDiffs"); printf("\n");
//			summedJointSpeedDiffs.t().raw_print(std::cout, "summedJointSpeedDiffs"); printf("\n");
//			speedError.t().raw_print(std::cout, "speedError"); printf("\n");
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
		if (node.second->isServo())
		{
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
	}

	return totalError;
}


double InverseKinematicJacobian::iterationStepGravitation(
		KinematicTree const& tree
		, std::map<MotorID, double>& torques
		, Task const& task
	) const
{
	arma::mat jacobianRaw;
	arma::mat jacobian = task.getJacobianForTask(tree, jacobianRaw, false);
	arma::colvec antiGravVec({0., 0., 9.86});

	arma::colvec torquesRaw = jacobian.t() * antiGravVec;


	std::map<NodeID, KinematicNode*> const& nodes = tree.getNodes();

	for (std::pair<NodeID, KinematicNode*> const& node : nodes) {
		kinematics::Int2MotorMap const& int2MotorMap = node.second->getInt2MotorMap();
		MotorValuesMap newValues;

		for (kinematics::Int2Motor const& i2m : int2MotorMap) {
			torques[i2m.second] = torquesRaw(i2m.first);
		}
	}

	return arma::norm(torquesRaw, 2);
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
				} else {
/*					error = arma::zeros(error.n_rows, 1);
					error(0) = 1;*/
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
