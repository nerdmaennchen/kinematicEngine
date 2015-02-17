/*
 *  Created on: 09.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICENGINETASKS_H_
#define KINEMATICENGINETASKS_H_

#include "kinematicEngine/robotDescription.h"
#include "task.h"
#include "taskDefaultPosition.h"
#include <array>

namespace kinematicEngine {

typedef std::map<uint, std::vector<const Task*>> TasksContainer;

class Tasks {
public:


	Tasks(RobotDescription const& description);
	virtual ~Tasks();

	void addTask(const Task *task, uint level) {
		m_tasks[level].push_back(task);
	}

	void addGravityTask(const Task *task) {
		m_gravityTasks.push_back(task);
	}

	std::vector<const Task*> const& getGravityTasks() const {
		return m_gravityTasks;
	}

	TasksContainer const& getTasks() const {
		return m_tasks;
	}

	const TaskDefaultPosition* getIdleTask() const {
		return m_idleTask;
	}

	void setIdleTask(const TaskDefaultPosition* task) {
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

	TasksContainer m_tasks;

	/**
	 * tasks that generate torques to apply to work against gravity
	 */
	std::vector<const Task*> m_gravityTasks;

	const TaskDefaultPosition* m_idleTask;
	KinematicTree m_tree;
};

}

#endif /* KINEMATICENGINETASKS_H_ */
