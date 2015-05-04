// TestVoroPlusPlus.cpp : Defines the entry point for the console application.
//

#include "gtest/gtest.h"
#include "GeometryCommon/Face.hpp"
#include "GeometryCommon/Vector3D.hpp"
#include "Voronoi/TessellationBase.hpp"
#include "Utilities/assert.hpp"
#include "GeometryCommon/OuterBoundary3D.hpp"
#include "Voronoi/TetGenTessellation.hpp"
#include "Voronoi/GhostBusters.hpp"
#include "Voronoi/TetGenDelaunay.hpp"

#include <vector>
#include <cstdlib>
#include <cassert>

#ifdef _DEBUG
#include <iostream>
#include <string>
#endif

using namespace std;

TEST(VoroPlusPlus, FaceStore)
{
	TessellationBase::FaceStore store;
	vector<Face> faces;

	for (int i = 0; i < 100; i++)
	{
		vector<VectorRef> vertices;
		int numVertices = rand() % 6 + 3;
		for (int j = 0; j < numVertices; j++)
			vertices.push_back(Vector3D(rand(), rand(), rand()));
		size_t index = store.StoreFace(vertices, 1, 2);
		ASSERT_EQ(index, i);
		faces.push_back(store.GetFace(index));
	}
}


double RandomDouble(double min, double max)
{
	double f = (double)rand() / RAND_MAX;  // Between 0 and 1,inclusive
	return (max - min) * f + min;
}

Vector3D RandomVector(const OuterBoundary3D &boundary)
{
	Vector3D v;

	v.x = RandomDouble(boundary.BackLowerLeft().x, boundary.FrontUpperRight().x);
	v.y = RandomDouble(boundary.BackLowerLeft().y, boundary.FrontUpperRight().y);
	v.z = RandomDouble(boundary.BackLowerLeft().z, boundary.FrontUpperRight().z);

	return v;
}

vector<Vector3D> CreateRandom(int numPoints, const OuterBoundary3D &boundary)
{
	srand(17);   // Make sure we generate the same sequence over and over
	vector<Vector3D> points;

	for (int i = 0; i < numPoints; i++)
	{
		Vector3D v = RandomVector(boundary);
		points.push_back(v);
	}

	return points;
}

vector<Vector3D> CreateCubeMesh(int perSide)
{
	vector<Vector3D> mesh;
	for (int x = 0; x < perSide; x++)
		for (int y = 0; y < perSide; y++)
			for (int z = 0; z < perSide; z++)
				mesh.push_back(Vector3D(x, y, z));

	return mesh;
}

void EnsureCubeVoronoi(const vector<Vector3D> &mesh, const Tessellation3D &tes)
{
	ASSERT_EQ(tes.GetPointNo(), mesh.size());
	for (size_t pt = 0; pt < mesh.size(); pt++)
	{
		auto faces = tes.GetCellFaces(pt);
		EXPECT_EQ(faces.size(), 6);
		for (size_t fc = 0; fc < faces.size(); fc++)
		{
			auto face = tes.GetFace(fc);
			EXPECT_EQ(face.vertices.size(), 4);
			EXPECT_NEAR(face.GetArea(), 1.0, 1e-12);
		}
		auto CoM = tes.GetCellCM(pt);
		double dist = abs(CoM - mesh[pt]);
		EXPECT_NEAR(dist, 0.0, 1e-12);
		EXPECT_NEAR(tes.GetVolume(pt), 1.0, 1e-12);
	}
}

TEST(TetGenDelaunay, Cube)
{
	vector<Vector3D> mesh = CreateCubeMesh(5);

	TetGenTessellation<RigidWallGhostBuster> tes;
	OuterBoundary3D boundary(Vector3D(4.5, 4.5, 4.5),
		Vector3D(-0.5, -0.5, -0.5));
	tes.Initialise(mesh, boundary);
	EnsureCubeVoronoi(mesh, tes);
}

TEST(TetGenDelaunay, OnBoundary)
{
	vector<Vector3D> mesh = CreateCubeMesh(5);

	TetGenTessellation<RigidWallGhostBuster> tes;
	OuterBoundary3D boundary(Vector3D(4.5, 4.5, 4.5),
		Vector3D(-0.5, -0.5, -0.5));

	mesh.push_back(Vector3D(-0.49999999999, -0.2, 2));
	EXPECT_THROW(tes.Initialise(mesh, boundary), invalid_argument);
}

TEST(CellCalculations, PyramidDimensions)
{
	// Pyramid with a square 1x1 base and a height of 1 right in the middle
	Vector3D b00(0, 0, 0), b01(0, 1, 0), b10(1, 0, 0), b11(1, 1, 0), h(.5, .5, 1);
	VectorRef rb00(b00), rb01(b01), rb10(b10), rb11(b11), rh(h);

	std::vector<VectorRef> base{ rb00, rb10, rb11, rb01 };
	std::vector<VectorRef> side1{ rb00, rb10, rh };
	std::vector<VectorRef> side2{ rb00, rb01, rh };
	std::vector<VectorRef> side3{ rb01, rb11, rh };
	std::vector<VectorRef> side4{ rb10, rb11, rh };
	Face fbase(base), f1(side1), f2(side2), f3(side3), f4(side4);
	std::vector<const Face *> faces{ &fbase, &f1, &f2, &f3, &f4 };

	double volume;
	Vector3D CoM;
	CalculateCellDimensions(faces, volume, CoM);

	// Volume is base * height / 3 
	EXPECT_NEAR(volume, 1.0/3.0, 1e-5);
}

TEST(CellCalculations, SquareDimensions)
{
	// A 2x2x2 cube
	VectorRef r000(Vector3D(0, 0, 0)), r200(Vector3D(2, 0, 0)), r220(Vector3D(2, 2, 0)), r020(Vector3D(0, 2, 0));
	VectorRef r002(Vector3D(0, 0, 2)), r202(Vector3D(2, 0, 2)), r222(Vector3D(2, 2, 2)), r022(Vector3D(0, 2, 2));

	std::vector<VectorRef> vfront{ r000, r200, r220, r020 };
	std::vector<VectorRef> vback{ r002, r202, r222, r022 };
	std::vector<VectorRef> vbottom{ r000, r200, r202, r002 };
	std::vector<VectorRef> vtop{ r020, r220, r222, r022 };
	std::vector<VectorRef> vright{ r200, r220, r222, r202 };
	std::vector<VectorRef> vleft{ r000, r020, r022, r002 };
	Face ffront(vfront), fback(vback), fbottom(vbottom), ftop(vtop), fright(vright), fleft(vleft);
	std::vector<const Face *> faces{ &ffront, &fback, &fbottom, &ftop, &fright, &fleft };

	double volume;
	Vector3D CoM;
	CalculateCellDimensions(faces, volume, CoM);

	EXPECT_NEAR(volume, 8, 1e-5);
	EXPECT_NEAR(abs(CoM - Vector3D(1, 1, 1)), 0, 1e-5);
}

void assertion_gtest_bridge(const char *expr, const char *function, const char *file, long line)
{
	EXPECT_FALSE(expr);
}

int main(int argc, char *argv[])
{
	BOOST_ASSERT_HANDLER = assertion_gtest_bridge;
	testing::InitGoogleTest(&argc, argv);
	int rc = RUN_ALL_TESTS();

#ifdef _DEBUG
	string s;
	getline(cin, s);
#endif
}