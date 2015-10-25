/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"

// Returns the farthest point in shape in direction d
Util::Vector SteerLib::GJK_EPA::getFarthestPoint(std::vector<Util::Vector>& shape, Util::Vector& d)
{
	float highest = -FLT_MAX;
	Util::Vector farthestPoint = Util::Vector(0, 0, 0);

	for (int i = 0; i < shape.size(); i++)
	{
		Util::Vector v = shape[i];
		float dot = v.x * d.x + v.y * d.y;

		if (dot > highest)
		{
			highest = dot;
			farthestPoint = v;
		}
	}

	return farthestPoint;
}

// Returns farthest point in Minkowski Difference between shapeA and shapeB in direction d
Util::Vector SteerLib::GJK_EPA::getSupport(std::vector<Util::Vector>& shapeA, std::vector<Util::Vector>& shapeB, Util::Vector& d)
{
	return getFarthestPoint(shapeA, d) - getFarthestPoint(shapeB, -d);
}

SteerLib::GJK_EPA::GJK_EPA(std::vector<Util::Vector>& simplex, bool& is_colliding)
{
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
    return false; // There is no collision
}
