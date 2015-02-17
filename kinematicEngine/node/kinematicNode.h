/*
 * kinematicNode.h
 *
 *  Created on: 12.02.2014
 *      Author: lutz
 */

#ifndef KINEMATICNODE_H_
#define KINEMATICNODE_H_

#include "visuals/kinematicVisual.h"
#include "kinematicMass.h"

#include "kinematicEngine/utils/motorIDs.h"

#include "utils/units.h"
#include "kinematicEngine/utils/utils.h"

#include <string>

#include "kinematicEngine/physics/physicsEnvironment.h"

#include <ode/ode.h>

#include <armadillo>

namespace kinematicEngine {

/**
 * this represents something movable
 */

namespace kinematics {
	typedef uint NodeID;

	typedef std::map<MotorID, double> MotorValuesMap;
	typedef std::pair<MotorID, double> MotorValuesMapEntry;

	typedef std::vector<MotorID> MotorIDs;

	typedef std::pair<MotorID, uint> Motor2Int;
	typedef std::map<MotorID, uint> Motor2IntMap;

	typedef std::pair<uint, MotorID> Int2Motor;
	typedef std::map<uint, MotorID> Int2MotorMap;

	typedef std::pair<uint, arma::colvec3> JacobianValue;
	typedef std::vector<std::pair<uint, arma::colvec3> > JacobianValues;
}

class KinematicNode {
protected:
	typedef kinematics::NodeID NodeID;
	typedef kinematics::MotorValuesMap MotorValuesMap;
	typedef kinematics::MotorValuesMapEntry MotorValuesMapEntry;
	typedef kinematics::Motor2Int Motor2Int;
	typedef kinematics::Int2Motor Int2Motor;
	typedef kinematics::Motor2IntMap Motor2IntMap;
	typedef kinematics::Int2MotorMap Int2MotorMap;
	typedef kinematics::JacobianValues JacobianValues;
	typedef kinematics::MotorIDs MotorIDs;

	KinematicNode(KinematicNode *parent,
			std::string name,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Degree alphaX,
			Degree alphaY,
			Degree alphaZ);

public:
	KinematicNode();

public:
	virtual ~KinematicNode();

	/// create a copy of this node without parent or children
	virtual KinematicNode* cloneDetached() const = 0;

	NodeID getID() const {
		return m_nodeID;
	}

	void setID(NodeID newID) {
		m_nodeID = newID;
	}

	/// define whether this node is a servo
	virtual bool isServo() const {
		return false;
	}

	/**
	 * how many motors contribute to this joint
	 * this is usually 1 but more in case several motors contribute to a joint
	 */
	virtual uint getMotorCnt() const {
		return m_minValues.size();
	}

	virtual void setValues(MotorValuesMap newValues) {
		m_values = newValues;
		forwardMatrix = preRotationMatrix;
		// the handy way of inversion
		backwardMatrix = homogeneousTransform::invertHomogeneous(forwardMatrix);
	}

	virtual MotorValuesMap getValues() const {
		return m_values;
	}

	virtual void setValueDerivatives(MotorValuesMap newValueDerivatives)
	{
		m_valueDerivatives = newValueDerivatives;
	}

	virtual MotorValuesMap getValueDerivatives() const
	{
		return m_valueDerivatives;
	}

	virtual MotorValuesMap clipValues(MotorValuesMap newValues) const
	{
		MotorValuesMap returnValues;
		for (MotorValuesMapEntry entry : m_minValues) {
			MotorID id = entry.first;
			returnValues[id] = utils::limited(newValues.at(id), m_minValues.at(id), m_maxValues.at(id));
		}
		return returnValues;
	}

	MotorValuesMap const& getMinValues() const {
		return m_minValues;
	}

	MotorValuesMap const& getMaxValues() const {
		return m_maxValues;
	}

	MotorValuesMap const& getPreferedValues() const {
		return m_preferredValues;
	}

	void setMinValues(MotorValuesMap const& newMinVals) {
		m_minValues = newMinVals;
	}

	void setMaxValues(MotorValuesMap const& newMaxVals) {
		m_maxValues = newMaxVals;
	}

	void setPreferedValues(MotorValuesMap const& newPreferedVals) {
		m_preferredValues = newPreferedVals;
	}

	/**
	 * returns the partial derivative of the endeffector of this joint.
	 * @param effector an effector in this joint's coordinate system
	 * @return partial derivative (locally linearized) of the endeffectors movement
	 */
	virtual JacobianValues getPartialDerivativeOfLocationToEffector(arma::colvec3 effector) const = 0;

	virtual JacobianValues getPartialDerivativeOfOrientationToEffector(arma::colvec3 effector) const = 0;

	/**
	 * what is the linear velocity of the point position in this nodes coordinates given the nodes speed (value derivative)
	 * @param position
	 * @return
	 */
	virtual arma::colvec4 getLinearSpeedOfPoint(arma::colvec3 position) const = 0;


	/**
	 * returns the affine transformation to a neighbor node (parent or child)
	 * @param relative parent or child of this node
	 * @return affine transformation
	 */
	arma::mat44 getMatrixToRelative(KinematicNode const* relative) const;

	arma::mat44 getInvMatrixToRelative(KinematicNode const* relative) const;

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

	inline arma::mat44 getForwardMatrix() const
	{
		return forwardMatrix;
	}

	inline arma::mat44 getBackwardMatrix() const
	{
		return backwardMatrix;
	}

	inline Motor2IntMap const& getMotor2IntMap() const {
		return m_motor2IntMap;
	}

	inline Int2MotorMap const& getInt2MotorMap() const {
		return m_int2MotorMap;
	}

	void setMotor2IntMap(Motor2IntMap const& newIDs) {
		m_motor2IntMap = newIDs;
		m_int2MotorMap.clear();
		m_motors.clear();

		for (Motor2Int const& m2i : m_motor2IntMap) {
			m_motors.push_back(m2i.first);
			m_int2MotorMap[m2i.second] = m2i.first;
		}
	}

	MotorIDs const& getMotors() const {
		return m_motors;
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
	virtual dMass getODEMassToAttachToParent(arma::mat44 coordinateFrame) const;

	virtual void attatchODEVisuals(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space);

	/**
	 * let the node apply a torque in the ode world
	 * @param torque
	 */
	virtual void setTorqueForODE(double torque) = 0;

protected:
	NodeID m_nodeID;

	MotorIDs m_motors;
	Motor2IntMap m_motor2IntMap;
	Int2MotorMap m_int2MotorMap;

	KinematicNode *m_parent;

	std::string m_name;

	MotorValuesMap m_values;
	MotorValuesMap m_valueDerivatives;
	MotorValuesMap m_minValues;
	MotorValuesMap m_maxValues;
	MotorValuesMap m_preferredValues;

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

}

#endif /* KINEMATICNODE_H_ */
