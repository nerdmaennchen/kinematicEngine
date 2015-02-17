/*
 *  Created on: 08.03.2014
 *      Author: lutz
 */

#ifndef KINEMATICPATH_H_
#define KINEMATICPATH_H_

#include "kinematicEngine/node/kinematicNode.h"
#include <vector>

namespace kinematicEngine {

class KinematicPathNode {
public:

	enum class Direction {
		FROM_PARENT, // previous element of path is the current elements parent
		FROM_CHILD,  // previous element of path is one of the current element's children
		BEGINNING,   // the first element in the chain
		LINK,        // element to ignore in the chain (this element's neighboring elements in the path are this element's children)
	};

	KinematicPathNode() : m_node(nullptr), m_direction(Direction::BEGINNING) {};

	KinematicPathNode(const KinematicNode *node, Direction direction) :
		m_node(node),
		m_direction(direction)
	{ }

	virtual ~KinematicPathNode()
	{ }

	const KinematicNode *m_node;

	Direction m_direction;
};



typedef std::vector<KinematicPathNode> KinematicPath;


KinematicPath invertKinematicPath(const KinematicPath& path);

}

#endif /* KINEMATICPATH_H_ */
