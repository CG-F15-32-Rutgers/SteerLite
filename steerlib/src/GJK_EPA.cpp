/*!
*
* \author VaHiD AzIzI
*
*/


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

Util::Vector getFarthestPointInDirection(const Util::Vector dir, const std::vector<Util::Vector>& _shape)
{
	float max = dot(*_shape.begin(), dir);
	Util::Vector point;

	for (std::vector<Util::Vector>::const_iterator i = _shape.begin(); i != _shape.end(); ++i)
	{
		float dProduct = dot(*i, dir);

		if (dProduct > max)
		{
			max = dProduct;
			point = *i;
		}
	}
	return point;
}

void getClosestEdge(float& dist, Util::Vector& normal, int& index, std::vector<Util::Vector>& simplex)
{
	dist = FLT_MAX;
	index = 0;

	for (int i = 0; i < simplex.size(); ++i) {
		int j = i + 1 == simplex.size() ? 0 : i + 1;

		Util::Vector A = simplex[i];
		Util::Vector B = simplex[j];

		Util::Vector C = B - A;

		Util::Vector CPerp = Util::Vector(-C.z, C.y, C.x);
		Util::Vector CPerpNorm = Util::normalize(CPerp);

		float dt = dot(CPerpNorm, A);

		if (dt < dist) {
			dist = dt;
			normal = CPerpNorm;
			index = j;
		}
	}
}

Util::Vector support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, const Util::Vector& dir)
{
	Util::Vector point1;
	Util::Vector point2;

	point1 = getFarthestPointInDirection(dir, _shapeA);
	point2 = getFarthestPointInDirection(-dir, _shapeB);
	return point1 - point2;

}

bool inOrigin(std::vector<Util::Vector> simplex, Util::Vector D)
{
	Util::Vector CB = simplex[1] - simplex[2];
	Util::Vector CA = simplex[0] - simplex[2];
	Util::Vector CO = -simplex[2];

	Util::Vector CANorm = cross(cross(CB, CA), CA);
	Util::Vector CBNorm = cross(cross(CA, CB), CB);

	if (dot(CANorm, CO) > 0)
	{
		simplex.erase(simplex.begin() + 1);
		D = CANorm;
		return false;
	}

	if (dot(CBNorm, CO) > 0)
	{
		simplex.erase(simplex.begin());
		D = CBNorm;
		return false;
	}

	return true;

}
bool gjk(std::vector<Util::Vector> simplex, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	Util::Vector dir1(1, 0, 0);
	Util::Vector dir2(0, 0, 1);

	Util::Vector A = support(_shapeA, _shapeB, dir1);
	Util::Vector B = support(_shapeA, _shapeB, dir2);

	Util::Vector AB = B - A;
	Util::Vector A0 = -A;
	Util::Vector D = cross(cross(AB, A0), AB);

	simplex.clear();
	simplex.push_back(A);
	simplex.push_back(B);

	while (true) {
		Util::Vector C = support(_shapeA, _shapeB, D);

		if (dot(C, D) < 0)
		{
			return false;
		}
		else
		{
			simplex.push_back(C);
		}

		if (inOrigin(simplex, D))
		{
			return true;
		}
	}
}

bool epa(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector>& simplex)
{
	while (true) {
		float dist;
		Util::Vector normal = Util::Vector(0, 0, 0);
		int index;

		getClosestEdge(dist, normal, index, simplex);

		Util::Vector supp = support(_shapeA, _shapeB, normal);

		float dt = dot(supp, normal);

		if (dt - dist <= 0) {
			return_penetration_depth = dist;
			return_penetration_vector = normal;
			return  true;
		}
		else {
			simplex.insert(simplex.begin() + index, support);
		}
	}
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> simplex;

	if (gjk(simplex, _shapeA, _shapeB))
	{
		epa(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB, simplex);
		return true;
	}

	return false;
}