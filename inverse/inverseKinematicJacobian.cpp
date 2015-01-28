/*
 * inverseKinematicJacobian.cpp
 *
 *  Created on: 13.06.2014
 *      Author: lutz
 */

#include "inverseKinematicJacobian.h"

#define DEFAULT_EPSILON 1.


InverseKinematicJacobian::InverseKinematicJacobian()
	: InverseKinematics()
	, m_epsilon(DEFAULT_EPSILON)
{
}

InverseKinematicJacobian::~InverseKinematicJacobian()
{
}


double InverseKinematicJacobian::iterationStep(
		KinematicTree const& tree
		, std::map<MotorID, Degree>& o_angles
		, KinematicEngineTasksContainer const& alltasks
		, KinematicEngineConstraints const& constraints
		, const KinematicEngineTaskDefaultPosition* idleTask
		, bool printDebugInfos
	) const
{
	double totalError = 0;

	std::map<MotorID, Degree> curMotorValues;
	tree.getMotorValues(curMotorValues);
	uint motorCnt = tree.getMotorCt();

	arma::colvec motorValuesVec = arma::zeros(motorCnt);
	for (std::pair<MotorID, Degree> const& curValue : curMotorValues)
	{
		motorValuesVec(tree.toInt(curValue.first)) = Radian(curValue.second).value();
	}

	arma::colvec summedJointAngleDiffs = arma::zeros(motorCnt);
	arma::mat nullspaceEyeMat = arma::eye(motorCnt, motorCnt);
	arma::mat nullspaceMat = arma::zeros(motorCnt, motorCnt);

	arma::mat bigJacobian;
	arma::colvec bigErrorVector;
	for (uint l(0); l < KinematicEngineTasksTypes::level::NUM_TASK_LEVELS; ++l) {
		std::vector<const KinematicEngineTask*> const& tasks = alltasks[l];

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

			for (const KinematicEngineConstraint* constraint : constraints) {
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

			if (printDebugInfos) {
				arma::colvec motorIDsVec = arma::zeros(motorCnt);
				for (std::pair<MotorID, Degree> const& curValue : curMotorValues)
				{
					motorIDsVec(tree.toInt(curValue.first)) = curValue.first;
				}

				uint insertIdx = bigJacobian.n_rows;
				bigJacobian.resize(bigJacobian.n_rows + jacobian.n_rows, jacobian.n_cols);
				bigJacobian.submat(insertIdx, 0, insertIdx + jacobian.n_rows - 1, jacobian.n_cols - 1) = jacobian;

				bigErrorVector.resize(insertIdx + errorVec.n_rows);
				bigErrorVector.rows(insertIdx, insertIdx + errorVec.n_rows - 1) = errorVec;

				arma::rowvec dofNorms = arma::rowvec(dofMat.n_rows);
				for (uint col = 0; col < dofMat.n_cols; ++ col) {
					dofNorms(col) = arma::norm(dofMat.col(col), 2);
				}


//				std::cout.precision(3);
//				std::cout << std::fixed << std::setw(7);
//				motorIDsVec.t().raw_print(std::cout);
//				motorValuesVec.t().raw_print(std::cout); printf("\n");
//				nullspaceMat.raw_print(std::cout); printf("\n");
	//			dofNorms.raw_print(std::cout); printf("\n");
//				jacobianRAW.raw_print(std::cout); printf("\n");
//				jacobian.raw_print(std::cout); printf("\n");
	//			jacobianDOF.raw_print(std::cout); printf("\n");

//				bigErrorVector.t().raw_print(std::cout);
//				arma::mat(bigJacobian * (preliminaryJointDiffs)).t().raw_print(std::cout);
//				arma::mat(bigJacobian * (summedJointAngleDiffs)).t().raw_print(std::cout);
//				printf("\n");

//				jointDiffConstraintError.t().raw_print(std::cout);
//				printf("\n");

//				jointDiffs.t().raw_print(std::cout);
//				preliminaryJointAngles.t().raw_print(std::cout);
//				jointDiffsDecorr.t().raw_print(std::cout);
//				preliminaryJointDiffs.t().raw_print(std::cout);
//				correctedJointAngleDiffs.t().raw_print(std::cout);
//				arma::colvec(summedJointAngleDiffs + motorValuesVec).t().raw_print(std::cout);
//				printf("\n");
			}

		}
	}

//	std::cout.precision(3);
//	std::cout << std::fixed << std::setw(7);
//	nullspaceMat.raw_print(std::cout); printf("\n");


	if (nullptr != idleTask) {
		arma::mat helper = arma::mat(nullspaceEyeMat - nullspaceMat);
		arma::colvec errorVec = idleTask->getErrors(tree);
		arma::colvec angleDiffVec = ((nullspaceEyeMat - nullspaceMat) * errorVec);

//		std::cout.precision(3);
//		std::cout << std::fixed << std::setw(7);
//		errorVec.t().raw_print(std::cout);
//		angleDiffVec.t().raw_print(std::cout); printf("\n");

		summedJointAngleDiffs += angleDiffVec;
	}

	for (uint32_t i = 0; i < summedJointAngleDiffs.n_rows; ++i)
	{
		summedJointAngleDiffs(i) = Math::limited(summedJointAngleDiffs(i), -m_maxValueChange, m_maxValueChange);
	}

	motorValuesVec += summedJointAngleDiffs;

	for (uint i = 0; i < motorValuesVec.n_rows; ++i)
	{
		Radian newAngle = motorValuesVec(i) * radians;

		/* clip */
		MotorID id = tree.toExt(i);
		Degree newAngleDeg = tree.clipAngleForMotor(id, Degree(newAngle));
		o_angles[id] = newAngleDeg;
	}

	return totalError;

	return 0;
}

double InverseKinematicJacobian::calculateSpeeds(
		KinematicTree const& tree
		, std::map<MotorID, RPM>& o_speeds
		, KinematicEngineTasksContainer const& tasks
		, KinematicEngineConstraints const& constraints
		, const KinematicEngineTaskDefaultPosition* idleTask
	) const
{
	double totalError = 0;
	uint motorCnt = tree.getMotorCt();

	std::map<MotorID, RPM> curMotorSpeeds;
	std::map<MotorID, Degree> curMotorValues;
	tree.getMotorSpeeds(curMotorSpeeds);
	tree.getMotorValues(curMotorValues);

	arma::colvec motorSpeedsVec = arma::zeros(motorCnt);
	arma::colvec motorValuesVec = arma::zeros(motorCnt);
	for (std::pair<MotorID, RPM> const& curValue : curMotorSpeeds)
	{
		// convert rounds per minute to radian per second
		motorSpeedsVec(tree.toInt(curValue.first)) = (curValue.second).value() / 60. * (2. * M_PI);
	}
	for (std::pair<MotorID, Degree> const& curValue : curMotorValues)
	{
		motorValuesVec(tree.toInt(curValue.first)) = Radian(curValue.second).value();
	}

	arma::colvec jointDiffConstraintError = arma::zeros(motorCnt);
	for (const KinematicEngineConstraint* constraint : constraints) {
		jointDiffConstraintError += constraint->getAngleErrorForConstraint(tree, motorValuesVec);
	}

	arma::colvec summedJointSpeedDiffs = arma::zeros(motorCnt);
	arma::mat nullspaceEyeMat = arma::eye(motorCnt, motorCnt);
	arma::mat nullspaceMat = jointDiffConstraintError * jointDiffConstraintError.t(); //arma::zeros(motorCnt, motorCnt);

	arma::mat bigJacobian;
	arma::colvec bigTargetVector;
	for (uint l(0); l < KinematicEngineTasksTypes::level::NUM_TASK_LEVELS; ++l) {
		std::vector<const KinematicEngineTask*> const& _tasks = tasks[l];

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


//			uint insertIdx = bigJacobian.n_rows;
//			bigJacobian.resize(bigJacobian.n_rows + jacobian.n_rows, jacobian.n_cols);
//			bigJacobian.submat(insertIdx,0, insertIdx + jacobian.n_rows - 1, jacobian.n_cols - 1) = jacobian;
//
//			bigTargetVector.resize(insertIdx + speedTarget.n_rows);
//			bigTargetVector.rows(insertIdx, insertIdx + speedTarget.n_rows - 1) = speedTarget;
//
//			arma::rowvec dofNorms = arma::zeros(1, dofMat.n_cols);
//			for (uint i = 0; i < dofMat.n_cols; ++i) {
//				dofNorms(i) = arma::norm(dofMat.col(i), 2);
//			}

//			std::cout.precision(8);
//			std::cout << std::fixed << std::setw(15);
//			dofMat.raw_print(std::cout); printf("\n");
//			dofNorms.raw_print(std::cout); printf("\n");
//			jacobian.raw_print(std::cout); printf("\n");
//			speedVec.t().raw_print(std::cout);
//			speedError.t().raw_print(std::cout);
//			arma::mat(jacobian * jointDiffs).t().raw_print(std::cout);
//			arma::mat(jacobian * dofMat * jointDiffs).t().raw_print(std::cout);
//			printf("\n");
//			speedTarget.t().raw_print(std::cout);
//			arma::mat(jacobian * (motorSpeedsVec + summedJointSpeedDiffs)).t().raw_print(std::cout);
//			bigTargetVector.t().raw_print(std::cout);
//			arma::mat(bigJacobian * (motorSpeedsVec + summedJointSpeedDiffs)).t().raw_print(std::cout);
//			motorSpeedsVec.t().raw_print(std::cout);
//			summedJointSpeedDiffs.t().raw_print(std::cout);
//			printf("\n");

		}
	}


//	std::cout.precision(3);
//	std::cout << std::fixed << std::setw(10);
//	nullspaceMat.raw_print(std::cout);
//	printf("\n\n\n");

	if (nullptr != idleTask) {
		arma::mat helper = arma::mat(nullspaceEyeMat - nullspaceMat);
		arma::colvec errorVec = idleTask->getErrors(tree);
		summedJointSpeedDiffs += (helper * errorVec) * idleTask->getSpeed();
	}


//	motorSpeedsVec.t().raw_print(std::cout);
//	summedJointSpeedDiffs.t().raw_print(std::cout);

	summedJointSpeedDiffs += motorSpeedsVec;

	for (uint i = 0; i < summedJointSpeedDiffs.n_rows; ++i)
	{
		RPM newSpeed = summedJointSpeedDiffs(i) * 60. / (2. * M_PI) * rounds_per_minute;

		MotorID id = tree.toExt(i);
		o_speeds[id] = newSpeed;
	}

	return totalError;
}

double InverseKinematicJacobian::iterationStepGravitation(KinematicTree const& tree, std::map<MotorID, double>& o_torques, std::vector<const KinematicEngineTask*> const& tasks) const
{
	const uint32_t numTasks = tasks.size();

	if (numTasks > 0)
	{
		const uint32_t numCols = tree.getMotorCt();

		/* calculate the size of the jacobian and the task vector */
		uint32_t numRows = 0;
		for (const KinematicEngineTask* const &task : tasks)
		{
			if (task->hasTarget()) {
				numRows += task->getDimensionCnt();
			}
		}

		/* build the "big" jacobian */
		arma::mat jacobian = arma::zeros(numRows, numCols);

		/* and the "big" error vector */
		arma::colvec errorVec = arma::zeros(numRows);

		uint32_t beginRow = 0;
		for (const KinematicEngineTask *const &task : tasks)
		{
			if (task->hasTarget())
			{
				const uint32_t endRow = beginRow + task->getDimensionCnt() - 1;
				arma::mat jacobianRaw;
				arma::mat jacobianForTask = task->getJacobianForTask(tree, jacobianRaw, false);

				jacobian.submat(beginRow, 0, endRow, numCols - 1) = jacobianForTask;

				errorVec.rows(beginRow, endRow) = task->getTarget() * task->getWeight();

				beginRow = endRow + 1;
			}
		}

//			std::cout.precision(3);
//			std::cout << std::fixed << std::setw(7);
//			jacobian.raw_print(std::cout); printf("\n\n\n");
//			errorVec.t().print(); printf("\n");

//		arma::mat helper = jacobian * jacobian.t();
//		const arma::mat pseudoInverseJacobian = jacobian.t() * arma::inv((helper + m_epsilon * arma::eye(helper.n_rows, helper.n_cols)));
		const arma::mat pseudoInverseJacobian = jacobian.t();

		arma::colvec torques = pseudoInverseJacobian * errorVec;

		for (uint i = 0; i < torques.n_rows; ++i)
		{
			MotorID id = tree.toExt(i);
			o_torques[id] = torques(i);
		}

		return arma::dot(errorVec, errorVec);
	}

	return 0;
}

arma::mat InverseKinematicJacobian::getJacobianForTasks(KinematicTree const& tree, std::vector<const KinematicEngineTask*> const& tasks, arma::mat &jacobianRAW, bool normalize) const
{
	const uint32_t numTasks = tasks.size();
	uint numCols = tree.getMotorCt();

	arma::mat jacobian = arma::zeros(1, numCols);
	if ((0 < numTasks) &&
		(0 < numCols)) /* at least one task and at least one motor */
	{
		/* calculate the size of the jacobian and the task vector */
		uint32_t numRows = 0;
		for (const KinematicEngineTask* const &task : tasks)
		{
			if (task->hasTarget()) {
				numRows += task->getDimensionCnt();
			}
		}

		/* build the "big" jacobian */
		jacobian = arma::zeros(numRows, numCols);
		jacobianRAW = arma::zeros(numRows, numCols);

		uint32_t beginRow = 0;
		for (const KinematicEngineTask *const &task : tasks)
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

arma::colvec InverseKinematicJacobian::getErrorForTasks(KinematicTree const& tree, std::vector<const KinematicEngineTask*> const& tasks) const
{
	const uint32_t numTasks = tasks.size();

	arma::colvec errorVec = arma::zeros(1);
	if (0 < numTasks) /* at least one task */
	{
		uint32_t numRows = 0;
		for (const KinematicEngineTask* const &task : tasks)
		{
			if (task->hasTarget()) {
				numRows += task->getDimensionCnt();
			}
		}
		errorVec = arma::zeros(numRows);

		uint32_t beginRow = 0;
		for (const KinematicEngineTask *const &task : tasks)
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

arma::colvec InverseKinematicJacobian::getSpeedTargetForTasks(KinematicTree const& tree, std::vector<const KinematicEngineTask*> const& tasks) const
{
	const uint32_t numTasks = tasks.size();

	arma::colvec errorVec = arma::zeros(1);
	if (0 < numTasks) /* at least one task */
	{
		uint32_t numRows = 0;
		for (const KinematicEngineTask* const &task : tasks)
		{
			if (task->hasTarget()) {
				numRows += task->getDimensionCnt();
			}
		}
		errorVec = arma::zeros(numRows);

		uint32_t beginRow = 0;
		for (const KinematicEngineTask *const &task : tasks)
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

int InverseKinematicJacobian::calculateNumRows(std::vector<const KinematicEngineTask*> const& tasks) const
{
	int numRowsConstraints = 0;
	for (const KinematicEngineTask* const &task : tasks)
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

