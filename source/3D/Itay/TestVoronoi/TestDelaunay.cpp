#include "gtest/gtest.h"
#include "Voronoi/TetGenDelaunay.hpp"
#include "GeometryCommon/Subcube.hpp"
#include "GeometryCommon/OuterBoundary3D.hpp"
#include "Utilities/assert.hpp"

#include <unordered_set>
#include <set>

using namespace std;

void BadSubcube()
{
	Subcube s2("--Z");
}

TEST(Subcube, Construction)
{
	Subcube s1("--+");
	ASSERT_THROW(BadSubcube(), invalid_argument);
}

TEST(Subcube, Sets)
{
	set<Subcube> set;
	set.insert(Subcube("---"));
	set.insert(Subcube("--+"));
	set.insert(Subcube("   "));
	set.insert(Subcube("--+"));
	set.insert(Subcube("   "));
	ASSERT_EQ(set.size(), 3);

	unordered_set<Subcube> hash;
	hash.insert(set.begin(), set.end());
	hash.insert(Subcube("+++"));
	hash.insert(Subcube("   "));
	ASSERT_EQ(hash.size(), 4); 

	ASSERT_EQ(Subcube::all().size(), 26);
}

OuterBoundary3D InitBoundary()
{
	return OuterBoundary3D(Vector3D(1, 1, 1), Vector3D(0, 0, 0));
}

TEST(SubCube, DistanceToFace)
{
	OuterBoundary3D boundary = InitBoundary();
	Vector3D pt(0.2, 0.5, 0.9);

	ASSERT_NEAR(boundary.distance(pt, "-  "), 0.2, 1e-8);
	ASSERT_NEAR(boundary.distance(pt, "+  "), 0.8, 1e-8);
	ASSERT_NEAR(boundary.distance(pt, " - "), 0.5, 1e-8);
	ASSERT_NEAR(boundary.distance(pt, " + "), 0.5, 1e-8);
	ASSERT_NEAR(boundary.distance(pt, "  -"), 0.9, 1e-8);
	ASSERT_NEAR(boundary.distance(pt, "  +"), 0.1, 1e-8);
}

TEST(SubCube, DistanceToPoint)
{
	OuterBoundary3D boundary = InitBoundary();
	Vector3D pt(0.2, 0.5, 0.9);

	ASSERT_NEAR(boundary.distance(pt, "---"), 1.0488, 1e-4);
	ASSERT_NEAR(boundary.distance(pt, "+++"), 0.94868, 1e-5);
	ASSERT_NEAR(boundary.distance(pt, "-+-"), 1.0488, 1e-4);  // Same distance because y is 0.5
	ASSERT_NEAR(boundary.distance(pt, "+-+"), 0.94868, 1e-4); // Ditto
	ASSERT_NEAR(boundary.distance(pt, "--+"), 0.54772, 1e-5);
}

TEST(SubCube, DistanceEdge)
{
	OuterBoundary3D boundary = InitBoundary();
	Vector3D pt(0.2, 0.5, 0.9);

	ASSERT_NEAR(boundary.distance(pt, "-- "), 0.53851, 1e-5);
	ASSERT_NEAR(boundary.distance(pt, "+ +"), 0.80623, 1e-5);
	ASSERT_NEAR(boundary.distance(pt, " -+"), 0.50990, 1e-5);
}

#define ASSERT_VECTOR_NEAR(vec1, vec2, epsilon) \
	ASSERT_NEAR(abs(vec2-vec1), 0, epsilon);

TEST(SubCube, PointReflection)
{
	OuterBoundary3D boundary = InitBoundary();
	Vector3D pt(0.2, 0.5, 0.9);

	ASSERT_VECTOR_NEAR(boundary.reflect(pt, "-  "), Vector3D(-0.2, 0.5, 0.9), 1e-5);
	ASSERT_VECTOR_NEAR(boundary.reflect(pt, "+ +"), Vector3D(1.8, 0.5, 1.1), 1e-5);
	ASSERT_VECTOR_NEAR(boundary.reflect(pt, "-+-"), Vector3D(-0.2, 1.5, -0.9), 1e-5);
}

TEST(Subcube, PointCopy)
{
	OuterBoundary3D boundary = InitBoundary();
	Vector3D pt(0.2, 0.5, 0.9);
	ASSERT_VECTOR_NEAR(boundary.copy(pt, "   "), pt, 1e-5);
	ASSERT_VECTOR_NEAR(boundary.copy(pt, "+++"), Vector3D(1.2, 1.5, 1.9), 1e-5);
	ASSERT_VECTOR_NEAR(boundary.copy(pt, "---"), Vector3D(-0.8, -0.5, -0.1), 1e-5);
	ASSERT_VECTOR_NEAR(boundary.copy(pt, "+ -"), Vector3D(1.2, 0.5, -0.1), 1e-5);
	ASSERT_VECTOR_NEAR(boundary.copy(pt, "-+ "), Vector3D(-0.8, 1.5, 0.9), 1e-5);
}