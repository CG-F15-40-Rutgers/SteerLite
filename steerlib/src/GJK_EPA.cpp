/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

std::vector<Util::Vector> minkowskiDiff(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB) {
	std::vector<Util::Vector> diffPoints = std::vector<Util::Vector>();
	for each(Util::Vector v in _shapeA) {
		for each(Util::Vector w in _shapeB) {
			diffPoints.push_back(v - w);
		}
	}
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
    return false; // There is no collision
}
