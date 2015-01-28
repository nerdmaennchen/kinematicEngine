/*
 * kinematicPath.cpp
 *
 *  Created on: 08.03.2014
 *      Author: lutz
 */

#include "kinematicPath.h"


KinematicPath invertKinematicPath(const KinematicPath& path)
{
	KinematicPath ret = path;
	std::reverse(ret.begin(), ret.end());

	/* build the associacion in the path */

	if (0 < ret.size())
	{
		ret[0].m_direction = KinematicPathNode::Direction::BEGINNING;
	}

	for (uint i = 1; i < ret.size(); ++i)
	{
		KinematicPathNode *prevNode = &(ret[i - 1]);
		KinematicPathNode *node = &(ret[i]);


//		INFO("from %s to %s", prevNode->m_node->getName().c_str(), node->m_node->getName().c_str());

		if (node->m_direction != KinematicPathNode::Direction::LINK)
		{
			if (node->m_node == prevNode->m_node->getParent())
			{
				node->m_direction = KinematicPathNode::Direction::FROM_CHILD;
			} else if (prevNode->m_node == node->m_node->getParent())
			{
				node->m_direction = KinematicPathNode::Direction::FROM_PARENT;
			} else
			{
				/* this should not be executed anyway... */
				ERROR("I have built an impossible kinematic path! there must be a bug in invertKinematicPath");
			}
		}
	}

	return ret;
}

