#ifndef KINEMATICENGINE_ROBOTDESCRIPTION_H__
#define KINEMATICENGINE_ROBOTDESCRIPTION_H__

#include "node/kinematicNode.h"
#include "node/kinematicNodeFactory.h"

#include <boost/property_tree/ptree.hpp>

#include <string>
#include <map>
#include <set>

namespace kinematicEngine {

/**
 ** \class RobotDescription
 ** \brief Description of the current robot, based on the robotDescription.xml
 */

class RobotDescription {

	typedef kinematics::NodeID NodeID;
public:
	RobotDescription(std::string robotDescriptionPath);
	virtual ~RobotDescription();

	inline const std::map<NodeID, KinematicNode*> getNodes() const {
		return m_nodes;
	}

	inline const std::set<MotorID>& getMotorIDs() const {
		return motorIDs;
	}

	const std::string getMotorName(NodeID id) const {
		const auto &it = m_nodes.find(id);
		if (it != m_nodes.end()) {
			assert(it->second->isServo());
			return it->second->getName();
		} else {
			return "Unknown";
		}
	}

	NodeID getEffectorID(const std::string &effectorName) const {
		for (const auto &it : m_nodes) {
			if (it.second->getName() == effectorName) {
				return it.first;
			}
		}

		return MOTOR_NONE;
	}

private:
	std::map<NodeID, KinematicNode*> m_nodes;
	std::set<MotorID> motorIDs;

	/**
	 * generate the kinematic tree from a given xml file (robot description)
	 * @param path path to file
	 */
	void generateFromXML(std::string path);

	void buildFromPTree(boost::property_tree::ptree subTree, KinematicNode *parent, KinematicNodeFactory& nodeFactory);
};

}

#endif
