#include "gtest/gtest.h"
#include "GeometryCommon/Face.hpp"
#include "GeometryCommon/Vector3D.hpp"
#include "Utilities/assert.hpp"
#include "GeometryCommon/OuterBoundary3D.hpp"
#include "GeometryCommon/Tetrahedron.hpp"
#include "../misc/universal_error.hpp"

TEST(Geometry3D, Vector3D_Construction)
{
	Vector3D vec0;
	ASSERT_EQ(vec0.x, 0.0);
	ASSERT_EQ(vec0.y, 0.0);
	ASSERT_EQ(vec0.z, 0.0);

	Vector3D vec1(1, 2, 3);
	ASSERT_EQ(vec1.x, 1);
	ASSERT_EQ(vec1.y, 2);
	ASSERT_EQ(vec1.z, 3);

	Vector3D vec2(vec1);
	ASSERT_EQ(vec1.x, vec2.x);
	ASSERT_EQ(vec1.y, vec2.y);
	ASSERT_EQ(vec1.z, vec2.z);

	Vector3D vec3 = vec2;
	ASSERT_EQ(vec3, vec2);
}

TEST(Geometry3D, VectorRef)
{
	Vector3D vec0(2, 3, 2);
	Vector3D vec1(1, 1, 1);
	Vector3D vec1_2(1, 1, 1);
	Vector3D vec0_2(vec0);

	VectorRef ref0(vec0), ref1(vec1), ref1_2(vec1_2), ref0_2(vec0_2);

	ASSERT_EQ(ref1->x, 1.0);
	ASSERT_EQ(ref1_2->y, 1.0);
	ASSERT_EQ(ref0_2->z, 2.0);

	ASSERT_EQ(&(*ref0), &(*ref0_2)); // Make sure vec0 and vec0_2 return references to the same vector in memory
	ASSERT_NE(&vec0, &vec0_2);		// But make sure they are not the same vector
	ASSERT_EQ(&(*ref1), &(*ref1_2)); // Same for vec1 and vec1_2
	ASSERT_NE(&vec1, &vec1_2);		// But make sure they are not the same vector
	ASSERT_NE(&(*ref0), &(*ref1)); // Make sure different vectors have different references

	ASSERT_EQ(ref0, ref0_2);		   // Compare references
	ASSERT_EQ(ref1, ref1_2);
	ASSERT_NE(ref0, ref1);

	// Back to vectors
	Vector3D back0 = *ref0;
	Vector3D back1 = *ref1;
	Vector3D back0_2 = *ref0_2;
	Vector3D back1_2 = *ref1_2;
	ASSERT_EQ(back0, vec0);
	ASSERT_EQ(back0_2, vec0_2);
	ASSERT_EQ(back1, vec1);
	ASSERT_EQ(back1_2, vec1_2);

	ASSERT_EQ(*ref0 + *ref1, vec0 + vec1);
}

TEST(Geometry3D, Vector3D_Operations)
{
	Vector3D v1(1, 2, 3);
	Vector3D v2(6, 5, 4);

	auto v3 = v1 + v2;
	ASSERT_EQ(v3.x, 7);
	ASSERT_EQ(v3.y, 7);
	ASSERT_EQ(v3.z, 7);

	auto v4 = v3 - v2;
	ASSERT_EQ(v4, v1);
}

TEST(Geometry3D, Face_Basic)
{
	Vector3D v1(0, 0, 0), v2(1, 0, 0), v3(1, 1, 0), v4(0, 1, 0);
	vector<VectorRef> vertices{ v1, v2, v3, v4 };
	Face face(vertices);

	ASSERT_EQ(vertices, face.vertices);
}

TEST(Geometry3D, Face_Area)
{
	Vector3D v1(0, 0, 0), v2(1, 0, 0), v3(1, 1, 0), v4(0, 1, 0);
	vector<VectorRef> vertices{ v1, v2, v3, v4 };
	Face square(vertices);
	ASSERT_EQ(square.GetArea(), 1);

	Vector3D v5(0, 0, 0), v6(0, 2, 0), v7(2, 0, 0);
	vector<VectorRef> vertices2{ v5, v6, v7 };
	Face triangle(vertices2);
	ASSERT_EQ(triangle.GetArea(), 2);
}


void CreateFaultyBoundary1()
{
	OuterBoundary3D(Vector3D(1, 1, 1), Vector3D(0, 2, 1));
}

TEST(Geometry3D, OuterBoundry3D)
{
	OuterBoundary3D b1(Vector3D(0, 0, 0), Vector3D(-1, -1, -1));
	ASSERT_THROW(CreateFaultyBoundary1(), invalid_argument);
}

TEST(Geometry3D, Tetrahedron)
{
	Tetrahedron t(Vector3D(0, 0, 0), Vector3D(0, 1, 0), Vector3D(1, 1, 0), Vector3D(0, 0, 6));
	stringstream strm;

	strm << t;
	ASSERT_EQ(strm.str(), "{ (0, 0, 0) - (0, 1, 0) - (1, 1, 0) - (0, 0, 6) }");
	ASSERT_EQ(t.volume(), 1.0);
	ASSERT_EQ(t.centerOfMass(), Vector3D(0.25, 0.5, 1.5));
}