/*
 * kinematicEngineTasks.h
 *
 *  Created on: 09.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICENGINETASKS_H_
#define KINEMATICENGINETASKS_H_

#include <tools/kinematicEngine/tasks/kinematicEngineTask.h>
#include <tools/kinematicEngine/tasks/kinematicEngineTaskDefaultPosition.h>
#include <array>

namespace KinematicEngineTasksTypes
{
	enum level : uint {
		TASK_LEVEL_GROUND_CONTACT_CONSTRAINT = 0,
		TASK_LEVEL_INTERNAL_CONSTRAINT = 1,
		TASK_LEVEL_DYNAMICS_CONSTRAINT = 2,
		TASK_LEVEL_REGULAR = 3,
		NUM_TASK_LEVELS = 4
	};
}

typedef std::array<std::vector<const KinematicEngineTask*>, KinematicEngineTasksTypes::NUM_TASK_LEVELS> KinematicEngineTasksContainer;

/**
 * @ingroup representations
 */
class KinematicEngineTasks {
public:


	KinematicEngineTasks();
	virtual ~KinematicEngineTasks();

	void addGroundContactConstraintTask(const KinematicEngineTask *task) {
		m_tasks[KinematicEngineTasksTypes::TASK_LEVEL_GROUND_CONTACT_CONSTRAINT].push_back(task);
	}

	void addInternalConstraintTask(const KinematicEngineTask *task) {
		m_tasks[KinematicEngineTasksTypes::TASK_LEVEL_INTERNAL_CONSTRAINT].push_back(task);
	}

	void addConstraintTask(const KinematicEngineTask *task) {
		m_tasks[KinematicEngineTasksTypes::TASK_LEVEL_DYNAMICS_CONSTRAINT].push_back(task);
	}

	void addTask(const KinematicEngineTask *task) {
		m_tasks[KinematicEngineTasksTypes::TASK_LEVEL_REGULAR].push_back(task);
	}

	void addGravityTask(const KinematicEngineTask *task) {
		m_gravityTasks.push_back(task);
	}

	std::vector<const KinematicEngineTask*> const& getGravityTasks() const {
		return m_gravityTasks;
	}

	KinematicEngineTasksContainer const& getTasks() const {
		return m_tasks;
	}

	const KinematicEngineTaskDefaultPosition* getIdleTask() const {
		return m_idleTask;
	}

	void setIdleTask(const KinematicEngineTaskDefaultPosition* task) {
		m_idleTask = task;
	}

	void clearTasks() {
		for (std::vector<const KinematicEngineTask*>& tasks : m_tasks) {
			tasks.clear();
		}
		m_gravityTasks.clear();
	}

	KinematicTree const& getTree() const {
		return m_tree;
	}

	KinematicTree &getTree() {
		return m_tree;
	}

private:

	KinematicEngineTasksContainer m_tasks;

	/**
	 * tasks that generate torques to apply to work against gravity
	 */
	std::vector<const KinematicEngineTask*> m_gravityTasks;

	const KinematicEngineTaskDefaultPosition* m_idleTask;
	KinematicTree m_tree;
};

#endif /* KINEMATICENGINETASKS_H_ */
