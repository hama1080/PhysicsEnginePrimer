#pragma once
#include "PxPhysicsAPI.h"
#include <vector>
#include <fstream>

using namespace std;
using namespace physx;


class Triangle {
public:
	Triangle(PxVec3 normal, PxVec3* vertices)
	{
		this->normal_ = normal;
		this->vertices_ptr_ = vertices;
	}
	PxVec3 normal_;
	PxVec3 *vertices_ptr_;
};

class StlOutput {
public:
	void outputStl(string output_path, PxActor** actor_buffer, PxU32 actor_cnt, bool divide_file);

private:
	ofstream write_stream;

	size_t writeRigidActor(PxRigidActor* actor);
	size_t writeBox(const PxTransform &transform, const PxVec3 &scale);
	size_t writeSphere(const PxTransform &transform, const PxReal radius);
	size_t writeSolid(const vector<Triangle> &triangles);
	void createSphereVerticesAndNormals(vector<PxVec3> &sphere_vertices, vector<PxVec3> &sphere_normals, vector<short> &sphere_indices, int rings, int sectors);

	void writeFacetNormal(PxVec3 normal, PxVec3 *vertices);
	void writeOuterLoop(PxVec3 *vertices);
	void writeVertex(PxVec3 vertex);
};