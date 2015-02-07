/*
 * kinematicNode.h
 *
 *  Created on: 12.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICNODE_H_
#define KINEMATICNODE_H_

#include "kinematicVisual.h"
#include "kinematicMass.h"

#include "utils/motorIDs.h"

#include "utils/units.h"
#include "utils/utils.h"

#include <string>

#include "../physics/physicsEnvironment.h"

#include <ode/ode.h>

#include <armadillo>


/**
 * this represents something movable
 */

class KinematicNode {
public:

	KinematicNode();

	KinematicNode(MotorID id,
			KinematicNode *parent,
			std::string name,
			double minValue,
			double maxValue,
			double preferredValue,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Degree alphaX,
			Degree alphaY,
			Degree alphaZ);

	virtual ~KinematicNode();

	/// create a copy of this node without parent or children
	virtual KinematicNode* cloneDetached() const = 0;

	/// define whether this node is a servo
	virtual bool isServo() const {
		return false;
	}

	virtual void setValue(double newValue) = 0;

	virtual double getValue() const = 0;

	virtual void setValueDerivative(double newValueDerivative)
	{
		m_valueDerivative = newValueDerivative;
	}

	virtual double getValueDerivative() const
	{
		return m_valueDerivative;
	}

	/**
	 * returns the partial derivative of the endeffector of this joint.
	 * @param effector an effector in this joint's coordinate system
	 * @return partial derivative (locally linearized) of the endeffectors movement
	 */
	virtual arma::colvec3 getPartialDerivativeOfLocationToEffector(arma::colvec3 effector) const = 0;

	virtual arma::colvec3 getPartialDerivativeOfOrientationToEffector(arma::colvec3 effector) const = 0;

	/**
	 * what is the linear velocity of the point position in this nodes coordinates given the nodes speed (value derivative)
	 * @param position
	 * @return
	 */
	virtual arma::colvec4 getLinearSpeedOfPoint(arma::colvec3 position) const = 0;

	virtual double clipValue(double newValue) const
	{
		return utils::limited(newValue, m_minValue, m_maxValue);
	}

	virtual double getMinValue() const {
		return m_minValue;
	}

	virtual double getMaxValue() const {
		return m_maxValue;
	}


	/**
	 * adds a child to this node (does NOT take ownership of pointer)
	 * @param child
	 */
	inline void addChild(KinematicNode *child)
	{
		m_children.push_back(child);
	}

	/**
	 * adds a visual to this node and takes ownership of pointer
	 * @param visual the visual to add
	 */
	inline void addVisual(KinematicVisual *visual)
	{
		m_visuals.push_back(visual);
	}

	/**
	 * returns the affine transformation to a neighbor node (parent or child)
	 * @param relative parent or child of this node
	 * @return affine transformation
	 */
	arma::mat44 getMatrixToRelative(MotorID relative) const;

	arma::mat44 getInvMatrixToRelative(MotorID relative) const;

	inline arma::mat44 getForwardMatrix() const
	{
		return forwardMatrix;
	}

	inline arma::mat44 getBackwardMatrix() const
	{
		return backwardMatrix;
	}

	inline MotorID getID() const
	{
		return id;
	}

	/**
	 * should only be called at construction time an never touched again since this node does not remove itself from its parent
	 * @param parent
	 */
	inline void setParent(KinematicNode *parent)
	{
		m_parent = parent;

		if (parent != nullptr)
			parent->addChild(this);
	}

	inline const KinematicNode *getParent() const
	{
		return m_parent;
	}

	inline std::vector<KinematicNode*> &getChildren()
	{
		return m_children;
	}

	// FIXME: this is not really const, if the vector contains non-const pointers
	inline const std::vector<KinematicNode*> &getChildren() const
	{
		return m_children;
	}

	// FIXME: this is not really const, if the vector contains non-const pointers
	inline const std::vector<KinematicVisual*> &getVisuals() const
	{
		return m_visuals;
	}

	inline const std::string &getName() const
	{
		return m_name;
	}

	inline bool hasMass() const
	{
		return m_hasMass;
	}

	inline const std::vector<KinematicMass> &getMasses() const
	{
		return m_masses;
	}

	inline std::vector<KinematicMass> &getMasses()
	{
		return m_masses;
	}

	/**
	 * can this node move?
	 * @return true iff this node can move
	 */
	virtual bool isFixedNode() const
	{
		return true;
	}

	virtual void addMass(double massGrams, arma::colvec3 position, std::string name)
	{
		if (0 != massGrams)
		{
			m_masses.push_back(KinematicMass(position, massGrams, name));
			m_hasMass = true;

			/* calculate new equivalent mass */
			m_equivalentMass = KinematicMass();
			for (KinematicMass const & mass : m_masses)
			{
				m_equivalentMass = m_equivalentMass.mean(mass);
			}
		}
	}

	virtual KinematicMass getEquivalentMass() const
	{
		return m_equivalentMass;
	}

	virtual void setAdditionalExtrinsicRotation(arma::mat33 _rotMat);

	virtual dBodyID attachToODE(PhysicsEnvironment *environment, dSpaceID visualSpaceID, dSpaceID collisionSpaceID, arma::mat44 coordinateFrame) = 0;

	virtual dBodyID getODEBody() const {
		return m_odeBody;
	}

	virtual dMass getODEMass(arma::mat44 coordinateFrame);

	/**
	 * a dummy node must not possess a body and thus a mass.
	 * This function shall be overridden by any dummy node to pass its attached masses to its parent
	 * @param coordinateFrame parents coordinate frame
	 * @return
	 */
	virtual dMass getODEMassToAttachToParent(arma::mat44 coordinateFrame) {
		dMass dummyMass;
		dMassSetZero(&dummyMass);
		return dummyMass;
	}

	virtual void attatchODEVisuals(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space);

	/**
	 * let the node apply a torque in the ode world
	 * @param torque
	 */
	virtual void setTorqueForODE(double torque) = 0;

protected:

	MotorID id;
	KinematicNode *m_parent;

	std::string m_name;

	double m_value;
	double m_valueDerivative;
	double m_minValue;
	double m_maxValue;
	double m_preferredValue;

	Millimeter m_translationX;
	Millimeter m_translationY;
	Millimeter m_translationZ;

	Degree m_alphaX;
	Degree m_alphaY;
	Degree m_alphaZ;

	bool m_hasMass = false;

	std::vector<KinematicNode*> m_children;
	std::vector<KinematicVisual*> m_visuals;

	arma::mat44 forwardMatrix, backwardMatrix, preRotationMatrix;

	std::vector<KinematicMass> m_masses;

	KinematicMass m_equivalentMass;

	arma::mat44 m_additionalExtrinsicRotation;

	dBodyID m_odeBody;
	arma::mat44 m_odeNodeBodyOffset; // the offset you have to add to the nodes coordinate frame to get the center of the body
};

#endif /* KINEMATICNODE_H_ */
