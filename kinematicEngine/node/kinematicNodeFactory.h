/*
 * kinematicNodeFactory.h
 *
 *  Created on: 14.05.2014
 *      Author: lutz
 */

#ifndef KINEMATICNODEFACTORY_H_
#define KINEMATICNODEFACTORY_H_

#include "kinematicNode.h"
#include <boost/property_tree/ptree.hpp>

namespace kinematicEngine {

class KinematicNodeFactory {
public:
	KinematicNodeFactory();
	virtual ~KinematicNodeFactory();

	KinematicNode *createNodeFromPTree(boost::property_tree::ptree::value_type ptree);

private:
	kinematics::NodeID m_nodeIDCounter;
	kinematics::NodeID m_intIDCounter;
};

}

#endif /* KINEMATICNODEFACTORY_H_ */
