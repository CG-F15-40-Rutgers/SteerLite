/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"

SteerLib::GJK_EPA::GJK_EPA()
{
}

// Returns farthest point in shape in direction d
Util::Vector SteerLib::GJK_EPA::getFarthestPoint(const std::vector<Util::Vector>& shape, Util::Vector& d)
{
	float highest = -FLT_MAX;
	Util::Vector farthestPoint = Util::Vector(0, 0, 0);

	for (int i = 0; i < shape.size(); i++)
	{
		Util::Vector v = shape[i];
		float dot = v * d;

		if (dot > highest)
		{
			highest = dot;
			farthestPoint = v;
		}
	}

	return farthestPoint;
}

// Gets closest edge of shape to origin
// Sets distance to distance from origin to closest edge
// Sets normal to normalized vector perpendicular to edge
// Sets index of where to add a new point to the simplex
void SteerLib::GJK_EPA::getClosestEdge(float& distance, Util::Vector& normal, int& index, std::vector<Util::Vector>& shape)
{
	distance = FLT_MAX;
	normal = Util::Vector(0, 0, 0);
	index = 0;

	for (int i = 0; i < shape.size(); i++)
	{
		int j = i + 1 == shape.size() ? 0 : i + 1;

		// Get points of the edge we're checking
		Util::Vector a = shape[i], b = shape[j];

		// Get the edge
		Util::Vector ab = b - a;

		// Get vector perpendicular to the edge and normalize it
		Util::Vector abPerp = Util::Vector(-ab.z, ab.y, ab.x);
		Util::Vector abPerpNorm = Util::normalize(abPerp);

		// Calculate distance
		float dot = abPerpNorm * a;

		if (dot < distance)
		{
			// Update values if current edge is closer
			distance = dot;
			normal = abPerpNorm;
			index = j;
		}
	}
}

// Returns farthest point in Minkowski Difference between shapeA and shapeB in direction d
Util::Vector SteerLib::GJK_EPA::getSupport(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, Util::Vector& d)
{
	return getFarthestPoint(_shapeA, d) - getFarthestPoint(_shapeB, -d);
}

// Returns true if simplex contains the origin
// Modifies d to new direction to check
bool SteerLib::GJK_EPA::containsOrigin(std::vector<Util::Vector>& simplex, Util::Vector& d)
{
	if (simplex.size() == 2)
	{
		// Simplex is a line segment
		Util::Vector a = simplex[1], b = simplex[0];

		// Get vector ab
		Util::Vector ab = b - a;

		// Calculate vector perpendicular to ab
		Util::Vector abPerp = Util::cross(Util::cross(ab, -a), ab);

		// Set new direction perpendicular to ab
		d = abPerp;
	}
	else if (simplex.size() == 3)
	{
		// Simplex is a triangle
		Util::Vector a = simplex[2], b = simplex[1], c = simplex[0];

		// Get vector ab and ac
		Util::Vector ab = b - a;
		Util::Vector ac = c - a;

		// Calculate vectors perpendicular to ab and ac
		Util::Vector abPerp = Util::cross(Util::cross(ac, ab), ab);
		Util::Vector acPerp = Util::cross(Util::cross(ab, ac), ac);

		if (abPerp * -a > 0)
		{
			// Remove c from simplex
			simplex.erase(simplex.begin());

			// Set new direction perpendicular to ab
			d = abPerp;
		}
		else if (acPerp * -a > 0)
		{
			// Remove b from simplex
			simplex.erase(simplex.begin() + 1);

			// Set new direction perpendicular to ac
			d = acPerp;
		}
		else
		{
			return true;
		}
	}

	return false;
}

// Uses GJK to check for collision between _shapeA and _shapeB
// Sets simplex shape if there is a collision
// Sets is_colliding to true if there is a collision
void SteerLib::GJK_EPA::GJK(std::vector<Util::Vector>& simplex, bool& is_colliding, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	// Pick a direction to start in
	Util::Vector d = Util::Vector(1, 0, 0);

	// Add first point to simplex
	Util::Vector s = getSupport(_shapeA, _shapeB, d);
	simplex.push_back(s);

	// Switch to opposite direction for next point
	d = -d;

	while (true)
	{
		// Add new point to simplex
		Util::Vector a = getSupport(_shapeA, _shapeB, d);

		float dot = a * d;

		if (dot < 0)
		{
			// The new point doesn't pass the origin
			simplex.clear();
			is_colliding = false;
			break;
		}
		else
		{
			simplex.push_back(a);

			if (containsOrigin(simplex, d))
			{
				// Simplex contains origin
				is_colliding = true;
				break;
			}
		}
	}
}

// Uses EPA to calculate penetration depth and MTV
void SteerLib::GJK_EPA::EPA(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& simplex)
{
	while (true)
	{
		float distance;
		Util::Vector normal;
		int index;

		// Find closest edge of simplex to origin
		getClosestEdge(distance, normal, index, simplex);

		// Find farthest point in Minkowski Difference in direction normal to closest edge
		Util::Vector s = getSupport(_shapeA, _shapeB, normal);

		// Calculate distance of s along the normal
		float dot = s * normal;

		if (dot - distance < 0.0001)
		{
			// EPA has converged
			return_penetration_depth = distance;
			return_penetration_vector = normal;
			break;
		}
		else
		{
			// Add new point to simplex and continue expanding
			simplex.insert(simplex.begin() + index, s);
		}
	}
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simplex;
	bool is_colliding;

	GJK(simplex, is_colliding, _shapeA, _shapeB);

	if (is_colliding)
	{
		EPA(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB, simplex);
		return true;
	}
	else
	{
		return_penetration_depth = 0;
		return_penetration_vector.zero();
		return false;
	}

}
