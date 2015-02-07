/*
 * kinematicEngineTasks.h
 *
 *  Created on: 09.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICENGINETASKS_H_
#define KINEMATICENGINETASKS_H_

#include "robot/robotDescription.h"
#include "kinematicEngineTask.h"
#include "kinematicEngineTaskDefaultPosition.h"
#include <array>

typedef std::map<uint, std::vector<const KinematicEngineTask*>> KinematicEngineTasksContainer;

class KinematicEngineTasks {
public:


	KinematicEngineTasks(RobotDescription const& description);
	virtual ~KinematicEngineTasks();

	void addTask(const KinematicEngineTask *task, uint level) {
		m_tasks[level].push_back(task);
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
		m_tasks.clear();
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
