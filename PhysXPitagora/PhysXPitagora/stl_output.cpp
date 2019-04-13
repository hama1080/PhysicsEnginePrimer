#include "stl_output.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <iomanip>
#include <sstream>


using namespace std;

static const PxVec3 g_box_vertex_data[] = {
	PxVec3(-1.0f, -1.0f, -1.0f),// 三角形1:開始
	PxVec3(-1.0f, -1.0f, 1.0f),
	PxVec3(-1.0f, 1.0f, 1.0f),// 三角形1:終了

	PxVec3(1.0f, 1.0f, -1.0f),// 三角形2:開始
	PxVec3(-1.0f, -1.0f, -1.0f),
	PxVec3(-1.0f, 1.0f, -1.0f),// 三角形2:終了

	PxVec3(1.0f, -1.0f, 1.0f),
	PxVec3(-1.0f, -1.0f, -1.0f),
	PxVec3(1.0f, -1.0f, -1.0f),

	PxVec3(1.0f, 1.0f, -1.0f),
	PxVec3(1.0f, -1.0f, -1.0f),
	PxVec3(-1.0f, -1.0f, -1.0f),

	PxVec3(-1.0f, -1.0f, -1.0f),
	PxVec3(-1.0f, 1.0f, 1.0f),
	PxVec3(-1.0f, 1.0f, -1.0f),

	PxVec3(1.0f, -1.0f, 1.0f),
	PxVec3(-1.0f, -1.0f, 1.0f),
	PxVec3(-1.0f, -1.0f, -1.0f),

	PxVec3(-1.0f, 1.0f, 1.0f),
	PxVec3(-1.0f, -1.0f, 1.0f),
	PxVec3(1.0f, -1.0f, 1.0f),

	PxVec3(1.0f, 1.0f, 1.0f),
	PxVec3(1.0f, -1.0f, -1.0f),
	PxVec3(1.0f, 1.0f, -1.0f),

	PxVec3(1.0f, -1.0f, -1.0f),
	PxVec3(1.0f, 1.0f, 1.0f),
	PxVec3(1.0f, -1.0f, 1.0f),

	PxVec3(1.0f, 1.0f, 1.0f),
	PxVec3(1.0f, 1.0f, -1.0f),
	PxVec3(-1.0f, 1.0f, -1.0f),

	PxVec3(1.0f, 1.0f, 1.0f),
	PxVec3(-1.0f, 1.0f, -1.0f),
	PxVec3(-1.0f, 1.0f, 1.0f),

	PxVec3(1.0f, 1.0f, 1.0f),
	PxVec3(-1.0f, 1.0f, 1.0f),
	PxVec3(1.0f, -1.0f, 1.0f)
};

static const PxVec3 g_box_normal_data[] = {
	PxVec3(-1.0f, 0.0f, 0.0f),
	PxVec3(0.0f, 0.0f, -1.0f),
	PxVec3(0.0f, -1.0f, 0.0f),
	PxVec3(0.0f, 0.0f, -1.0f),
	PxVec3(-1.0f, 0.0f, 0.0f),
	PxVec3(0.0f, -1.0f, 0.0f),
	PxVec3(0.0f, 0.0f, 1.0f),
	PxVec3(1.0f, 0.0f, 0.0f),
	PxVec3(1.0f, 0.0f, 0.0f),
	PxVec3(0.0f, 1.0f, 0.0f),
	PxVec3(0.0f, 1.0f, 0.0f),
	PxVec3(0.0f, 0.0f, 1.0f)
};


// stlファイルを書き出す
// output_path: 出力先ディレクトリ
// actor_buffer: アクター情報を格納したバッファ
// actor_cnt: バッファに含まれるアクター数
// divide_file:
//  falseにすると全てのactor情報を1つのSTLファイルにして出力
//  trueにすると1つのactorにつき1つのSTLファイルを出力
void StlOutput::outputStl(string output_path, PxActor** actor_buffer, PxU32 actor_cnt, bool divide_file)
{
	cout << "STLファイル生成中…" << endl;
	if (divide_file) {
		cout << "各剛体を個別のSTLファイルとして出力" << endl;
	}
	else {
		cout << "全剛体をまとめたSTLファイルとして出力" << endl;
		write_stream.open(output_path + "output.stl", ios::out);
	}

	size_t triangle_mesh_cnt = 0;

	for (PxU32 i = 0; i < actor_cnt; i++)
	{
		if (divide_file) {
			stringstream fileName;
			fileName << i << ".stl";
			write_stream.open(output_path + fileName.str(), ios::out);
			write_stream << "solid" << endl;
		}

		PxRigidActor* rigid_actor = (PxRigidActor*)actor_buffer[i];	// アクターの取得
		triangle_mesh_cnt += writeRigidActor(rigid_actor);

		if (divide_file)
		{
			write_stream << "endsolid" << endl;
			write_stream.close();
		}
	}

	cout << "\t書き出しアクター数:\t " << actor_cnt << endl;
	cout << "\t三角形メッシュ数:\t " << triangle_mesh_cnt << endl;
	cout << "書き出し完了" << endl;
}

size_t StlOutput::writeRigidActor(PxRigidActor* actor)
{
	// shapeを取得
	PxShape** shapes;
	shapes = new PxShape*[actor->getNbShapes()];
	actor->getShapes(shapes, actor->getNbShapes());
	PxShape* shape = shapes[0];

	// transformを取得
	PxTransform transform = PxShapeExt::getGlobalPose(*shape, *actor);

	// geometry_typeを取得
	PxGeometryType::Enum geometry_type = shape->getGeometryType();


	if (geometry_type == PxGeometryType::eBOX) {
		PxBoxGeometry box;
		shape->getBoxGeometry(box);
		return writeBox(transform, box.halfExtents);
	}
	else if (geometry_type == PxGeometryType::eSPHERE)
	{
		PxSphereGeometry sphere;
		shape->getSphereGeometry(sphere);
		return writeSphere(transform, sphere.radius);
	}
	else {
		cout << "未対応の形状" << endl;
		return 0;
	}
}


size_t StlOutput::writeBox(const PxTransform &transform, const PxVec3 &scale)
{
	PxMat44 matS = PxMat44(PxVec4(scale, 1.0f));//拡大縮小行列
	PxMat44 matR = PxMat44(transform); //並進・回転行列
	PxMat44 model_matrix = matR * matS;//拡大縮小→回転→平行移動の順番

	vector<Triangle> triangles;

	for (size_t i = 0; i != 12; i++)
	{
		PxVec3* triangle_vertices = new PxVec3[3];
		// g_box_vertex_dataからboxを構成する三角形のうち1つを取り出し、アフィン変換をかけた後にXYZ座標を取り出す。
		triangle_vertices[0] = model_matrix.transform(PxVec4(g_box_vertex_data[i * 3], 1)).getXYZ();
		triangle_vertices[1] = model_matrix.transform(PxVec4(g_box_vertex_data[i * 3 + 1], 1)).getXYZ();
		triangle_vertices[2] = model_matrix.transform(PxVec4(g_box_vertex_data[i * 3 + 2], 1)).getXYZ();

		triangles.push_back(Triangle(g_box_normal_data[i], triangle_vertices));
	}

	return writeSolid(triangles);
}


size_t StlOutput::writeSphere(const PxTransform &transform, const PxReal radius)
{
	PxMat44 matS = PxMat44(PxVec4(PxVec3(radius), 1.0f));//拡大縮小行列
	PxMat44 matR = PxMat44(transform); //並進・回転行列
	PxMat44 model_matrix = matR * matS;//拡大縮小→回転→平行移動の順番

	vector<Triangle> triangles;

	//半径1の球の頂点・法線を取得
	vector<PxVec3> vertices;
	vector<PxVec3> normals;
	vector<short> indices;
	const int kRings = 20;
	const int kSectors = 20;
	createSphereVerticesAndNormals(vertices, normals, indices, kRings, kSectors);
	for (size_t i = 0; i != indices.size() - 4; i += 4)
	{
		// triangle0: 2-1-0
		PxVec3* triangle_vertices_0 = new PxVec3[3];
		triangle_vertices_0[0] = model_matrix.transform(vertices[indices[i + 2]]);
		triangle_vertices_0[1] = model_matrix.transform(vertices[indices[i + 1]]);
		triangle_vertices_0[2] = model_matrix.transform(vertices[indices[i + 0]]);
		PxVec3 normal0 = 
			  (normals[indices[i + 0]] + normals[indices[i + 1]] + normals[indices[i + 2]])/3.0f;
		triangles.push_back(Triangle(normal0, triangle_vertices_0));

		// triangle1: 3-2-0
		PxVec3* triangle_vertices_1 = new PxVec3[3];
		triangle_vertices_1[0] = model_matrix.transform(vertices[indices[i + 3]]);
		triangle_vertices_1[1] = model_matrix.transform(vertices[indices[i + 2]]);
		triangle_vertices_1[2] = model_matrix.transform(vertices[indices[i + 0]]);
		PxVec3 normal1 =
			(normals[indices[i + 0]] + normals[indices[i + 2]] + normals[indices[i + 2]]) / 3.0f;

		triangles.push_back(Triangle(normal1, triangle_vertices_1));
	}
	return writeSolid(triangles);
}

size_t StlOutput::writeSolid(const vector<Triangle> &triangles)
{
	//書き出し
	for (size_t i = 0; i != triangles.size(); i++)
	{
		writeFacetNormal(triangles[i].normal_, triangles[i].vertices_ptr_);
		delete triangles[i].vertices_ptr_;
	}
	return triangles.size();
}

// 球の頂点，法線を得る関数
// http://stackoverflow.com/questions/7946770/calculating-a-sphere-in-opengl
void StlOutput::createSphereVerticesAndNormals(vector<PxVec3> &sphere_vertices, vector<PxVec3> &sphere_normals, vector<short> &sphere_indices, int rings, int sectors)
{
	float radius = 1.0; //半径1
	float const kR = 1. / (float)(rings - 1);
	float const kS = 1. / (float)(sectors - 1);
	int r, s;

	sphere_vertices.resize(rings * sectors * 3);
	sphere_normals.resize(rings * sectors * 3);

	vector<PxVec3>::iterator v = sphere_vertices.begin();
	vector<PxVec3>::iterator n = sphere_normals.begin();

	for (r = 0; r < rings; r++) for (s = 0; s < sectors; s++) {
		float const ky = sin(-M_PI_2 + M_PI * r * kR);
		float const kx = cos(2 * M_PI * s * kS) * sin(M_PI * r * kR);
		float const kz = sin(2 * M_PI * s * kS) * sin(M_PI * r * kR);

		*v++ = PxVec3(kx, ky, kz) * radius;

		*n++ = PxVec3(kx, ky, kz);
	}

	vector<short>::iterator i = sphere_indices.begin();
	for (r = 0; r < rings; r++) for (s = 0; s < sectors; s++) {
		sphere_indices.push_back(r * sectors + s);
		sphere_indices.push_back(r * sectors + (s + 1));
		sphere_indices.push_back((r + 1) * sectors + (s + 1));
		sphere_indices.push_back((r + 1) * sectors + s);

		if (((r + 1) * sectors + (s + 1)) == sphere_vertices.size() / 3) {
			return;
		}
	}
}

void StlOutput::writeFacetNormal(PxVec3 normal, PxVec3 *vertices)
{
	write_stream << "facet normal " << normal.x << " " << normal.y << " " << normal.z << endl;
	writeOuterLoop(vertices);
	write_stream << "endfacet" << endl;
}

void StlOutput::writeOuterLoop(PxVec3 *vertices)
{
	write_stream << "outer loop" << endl;
	for (size_t i = 0; i != 3; i++)
		writeVertex(vertices[i]);
	write_stream << "endloop" << endl;
}

void StlOutput::writeVertex(PxVec3 vertex)
{
	write_stream << "vertex " << fixed << setprecision(3) << vertex.x << " " << vertex.y << " " << vertex.z << endl;
}
