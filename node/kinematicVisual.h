#ifndef KINEMATICVISUAL_H__
#define KINEMATICVISUAL_H__

#include "utils/units.h"

#include <armadillo>
#include <string>
#include <vector>
#include <array>

#include <ode/ode.h>

/*------------------------------------------------------------------------------------------------*/

class KinematicFace {
public:
	KinematicFace(const std::string &label)
		: label(label)
	{}

	virtual ~KinematicFace() {}

	void addVertex(const arma::colvec4 &vertex) {
		vertices.push_back(vertex);
	}

	size_t size() const {
		return vertices.size();
	}

	const std::vector<arma::colvec4> getVertices() const {
		return vertices;
	}

	const std::string& getLabel() const {
		return label;
	}

	const arma::colvec4& operator[](unsigned int index) const {
		return vertices[index];
	}

	arma::colvec4& operator[](unsigned int index) {
		return vertices[index];
	}

private:
	std::string label;
	std::vector<arma::colvec4> vertices;
};


KinematicFace operator* (arma::mat44 transitionMatrix, const KinematicFace& face);


/*------------------------------------------------------------------------------------------------*/

class KinematicVisual {
public:
	typedef std::array<float, 4> ColorVec;

	KinematicVisual(std::string name,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Degree alphaX,
			Degree alphaY,
			Degree alphaZ,
			ColorVec colors,
			int textureNum,
			bool visible = true,
			bool canCollide = true)
		: m_colorVec(colors)
		, m_textureNum(textureNum)
		, visible(visible)
		, canCollide(canCollide)
		, name(name)
	{
		arma::mat44 translation, rotationX, rotationY, rotationZ;

		translation    << 1. << 0. << 0. << Meter(translationX).value() << arma::endr
		               << 0. << 1. << 0. << Meter(translationY).value() << arma::endr
		               << 0. << 0. << 1. << Meter(translationZ).value() << arma::endr
		               << 0. << 0. << 0. << 1. << arma::endr;

		{
			const double cX = cos(alphaX);
			const double sX = sin(alphaX);

			rotationX  << 1. << 0. <<  0. << 0. << arma::endr
			           << 0. << cX << -sX << 0. << arma::endr
			           << 0. << sX <<  cX << 0. << arma::endr
			           << 0. << 0. <<  0. << 1. << arma::endr;
		}

		{
			const double cY = cos(alphaY);
			const double sY = sin(alphaY);

			rotationY  << cY  << 0. << sY << 0. << arma::endr
			           << 0.  << 1. << 0. << 0. << arma::endr
			           << -sY << 0. << cY << 0. << arma::endr
			           << 0.  << 0. << 0. << 1. << arma::endr;
		}

		{
			const double cZ = cos(alphaZ);
			const double sZ = sin(alphaZ);

			rotationZ  << cZ << -sZ << 0. << 0. << arma::endr
			           << sZ <<  cZ << 0. << 0. << arma::endr
			           << 0. <<  0. << 1. << 0. << arma::endr
			           << 0. <<  0. << 0. << 1. << arma::endr;
		}

		transitionMatrix= translation * rotationX * rotationY * rotationZ;
	}

	virtual ~KinematicVisual() {}

	void addFace(const KinematicFace& face) {
		faces.push_back(face);
	}

	const std::string& getName() const { return name; };

	bool isVisible() const { return visible; }
	void setVisibility(bool isVisible) { visible = isVisible; }

	const std::vector<KinematicFace>& getFaces() const { return faces; }


	virtual void attatchToODE(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space) = 0;

protected:
	arma::mat44 transitionMatrix;

	ColorVec m_colorVec;
	int m_textureNum;
	bool visible;
	bool canCollide;

private:
	std::string name;

	std::vector<KinematicFace> faces;
};


/*------------------------------------------------------------------------------------------------*/

class KinematicVisualBox : public KinematicVisual {
public:
	KinematicVisualBox(
			std::string name,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Millimeter sizeX,
			Millimeter sizeY,
			Millimeter sizeZ,
			Degree alphaX,
			Degree alphaY,
			Degree alphaZ,
			KinematicVisual::ColorVec colors,
			int textureNum,
			bool isVisible = true,
			bool canCollide = true);

	virtual void attatchToODE(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space);

private:
	Millimeter sizeX, sizeY, sizeZ;
};


class KinematicVisualCylinder : public KinematicVisual {
public:
	KinematicVisualCylinder(
			std::string name,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Millimeter radius,
			Millimeter length,
			Degree alphaX,
			Degree alphaY,
			Degree alphaZ,
			KinematicVisual::ColorVec colors,
			int textureNum,
			bool isVisible = true,
			bool canCollide = true);

	virtual void attatchToODE(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space);

private:
	Millimeter radius, length;
};



class KinematicVisualSphere : public KinematicVisual {
public:
	KinematicVisualSphere(
			std::string name,
			Millimeter translationX,
			Millimeter translationY,
			Millimeter translationZ,
			Millimeter radius,
			Degree alphaX,
			Degree alphaY,
			Degree alphaZ,
			KinematicVisual::ColorVec colors,
			int textureNum,
			bool isVisible = true,
			bool canCollide = true);

	virtual void attatchToODE(arma::mat44 coordinateFrame, dBodyID body, dSpaceID space);

private:
	Millimeter radius, length;
};

#endif
