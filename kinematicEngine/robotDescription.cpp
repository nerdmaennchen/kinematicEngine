#include "robotDescription.h"
#include "kinematicEngine/kinematics.h"
#include "kinematicEngine/utils/utils.h"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/optional/optional.hpp>
#include <iostream>

namespace kinematicEngine {

/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

RobotDescription::RobotDescription(std::string robotDescriptionPath) {
	// build the kinematic tree
	generateFromXML(robotDescriptionPath);

	// collect motor IDs and create offset configuration options
	for (const auto &it : m_nodes) {
		if (it.second->isServo()) {
			for (auto motorID : it.second->getMotors()) {
				motorIDs.insert(motorID);
			}
		}
	}
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

RobotDescription::~RobotDescription() {
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

void RobotDescription::generateFromXML(std::string robotDescriptionPath)
{
	if (false == utils::fileExists(robotDescriptionPath)) {
		std::cerr << "Could not find robot description file ./" << robotDescriptionPath << ". Did you copy it to the robot?" << std::endl;
		return;
	}

	boost::property_tree::ptree tree;
	boost::property_tree::read_xml(robotDescriptionPath.c_str(), tree);

	int robotDescriptionNodeCnt = tree.count("robotdescription");


	if (0 < robotDescriptionNodeCnt) {
		BOOST_FOREACH(boost::property_tree::ptree::value_type const &child, tree.get_child("robotdescription") ) {
			boost::optional<std::string> name = child.second.get_optional<std::string>("<xmlattr>.name");
			if (name.is_initialized() && name.get() == "root") {
				KinematicNodeFactory nodeFactory;
				KinematicNode *rootNode = nodeFactory.createNodeFromPTree(child);
				m_nodes[rootNode->getID()] = rootNode;
				buildFromPTree(child.second, rootNode, nodeFactory);
			}
		}
	}
}


/*------------------------------------------------------------------------------------------------*/

/**
 **
 */

void RobotDescription::buildFromPTree(boost::property_tree::ptree subTree, KinematicNode *parent, KinematicNodeFactory& nodeFactory) {
	try {
		BOOST_FOREACH(boost::property_tree::ptree::value_type const &child, subTree) {
			if (child.first == "effector") {
				KinematicNode *node = nodeFactory.createNodeFromPTree(child);
				node->setParent(parent);
				m_nodes[node->getID()] = node;
				buildFromPTree(child.second, node, nodeFactory);
			}
		}
	} catch (const boost::property_tree::xml_parser::xml_parser_error& ex) {
		std::cerr << "Error in file " << ex.filename() << " at line " << ex.line() << ": " << ex.what() << std::endl;
	}
}

}
