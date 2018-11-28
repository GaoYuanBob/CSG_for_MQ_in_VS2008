#include "files.h"
#include "cork.h"
#include <iostream>
#include <sstream>
#include <assert.h>
#include "vec.h"
#include <vcg/complex/trimesh/create/platonic.h>
#include <vector>
using namespace std;

#ifndef uint
typedef unsigned int uint;
#endif

// 不需要move_matrix的
//void readMeshModel(MeshModel* mm, FileMesh *data)
//{
//	data->vertices.resize(mm->cm.vert.size());
//	data->triangles.resize(mm->cm.face.size());
//	for(CMeshO::VertexIterator vi = mm->cm.vert.begin(); vi != mm->cm.vert.end(); ++vi)
//	{
//		int i = vi - mm->cm.vert.begin();
//		data->vertices[i].pos.x = vi->P().X();
//		data->vertices[i].pos.y = vi->P().Y();
//		data->vertices[i].pos.z = vi->P().Z();
//	}
//	for(CMeshO::FaceIterator fi = mm->cm.face.begin(); fi != mm->cm.face.end(); ++fi)
//	{
//		int i = fi - mm->cm.face.begin();
//		data->triangles[i].a = (int)(fi->V(0) - &*mm->cm.vert.begin());
//		data->triangles[i].b = (int)(fi->V(1) - &*mm->cm.vert.begin());
//		data->triangles[i].c = (int)(fi->V(2) - &*mm->cm.vert.begin());
//	}
//}

//// 需要move_state_matrix的
void readMeshModel_matrix(MeshModel* mm, FileMesh *data, vcg::Matrix44f move_matrix)
{
	data->vertices.resize(mm->cm.vert.size());
	data->triangles.resize(mm->cm.face.size());
	for(CMeshO::VertexIterator vi = mm->cm.vert.begin(); vi != mm->cm.vert.end(); ++vi)
	{
		int i = vi - mm->cm.vert.begin();
		Point3f tmpP =  move_matrix * vi->P();
		data->vertices[i].pos.x = tmpP.X();
		data->vertices[i].pos.y = tmpP.Y();
		data->vertices[i].pos.z = tmpP.Z();
	}
	for(CMeshO::FaceIterator fi = mm->cm.face.begin(); fi != mm->cm.face.end(); ++fi)
	{
		int i = fi - mm->cm.face.begin();
		data->triangles[i].a = (int)(fi->V(0) - &*mm->cm.vert.begin());
		data->triangles[i].b = (int)(fi->V(1) - &*mm->cm.vert.begin());
		data->triangles[i].c = (int)(fi->V(2) - &*mm->cm.vert.begin());
	}
}

void file2corktrimesh(const FileMesh &in, CorkTriMesh *out)	// old version - 2018.11.28 注释掉 GY
{
	out->n_vertices = in.vertices.size();
	out->n_triangles = in.triangles.size();

	out->triangles = new uint[out->n_triangles * 3];
	out->vertices = new float[out->n_vertices * 3];

	for (uint i = 0; i < out->n_triangles; i++) {
		out->triangles[3 * i + 0] = in.triangles[i].a;
		out->triangles[3 * i + 1] = in.triangles[i].b;
		out->triangles[3 * i + 2] = in.triangles[i].c;
	}

	for (uint i = 0; i < out->n_vertices; i++) {
		out->vertices[3 * i + 0] = in.vertices[i].pos.x;
		out->vertices[3 * i + 1] = in.vertices[i].pos.y;
		out->vertices[3 * i + 2] = in.vertices[i].pos.z;
	}
}

void MeshModel_to_CorkTriMesh(MeshModel* mm, CorkTriMesh *out)	// new version - 2018.11.28 修改 GY (合并 readMeshModel 和 file2corktrimesh)
{
	out->n_vertices = mm->cm.vert.size();
	out->n_triangles = mm->cm.face.size();
	out->triangles = new uint[(out->n_triangles) * 3];
	out->vertices = new float[(out->n_vertices) * 3];
	for(CMeshO::FaceIterator fi = mm->cm.face.begin(); fi != mm->cm.face.end(); ++fi)
	{
		int i = fi - mm->cm.face.begin();
		out->triangles[3 * i]     = (int)(fi->V(0) - &*mm->cm.vert.begin());
		out->triangles[3 * i + 1] = (int)(fi->V(1) - &*mm->cm.vert.begin());
		out->triangles[3 * i + 2] = (int)(fi->V(2) - &*mm->cm.vert.begin());
	}
	for(CMeshO::VertexIterator vi = mm->cm.vert.begin(); vi != mm->cm.vert.end(); ++vi)
	{
		int i = vi - mm->cm.vert.begin();
		(out->vertices)[3 * i + 0] = vi->P().X();
		(out->vertices)[3 * i + 1] = vi->P().Y();
		(out->vertices)[3 * i + 2] = vi->P().Z();
	}
}

void corktrimesh2file(CorkTriMesh in, FileMesh &out)
{
	out.vertices.resize(in.n_vertices);
	out.triangles.resize(in.n_triangles);
	for (uint i = 0; i < in.n_triangles; i++) {
		out.triangles[i].a = in.triangles[3 * i + 0];
		out.triangles[i].b = in.triangles[3 * i + 1];
		out.triangles[i].c = in.triangles[3 * i + 2];
	}
	for (uint i = 0; i < in.n_vertices; i++) {
		out.vertices[i].pos.x = in.vertices[3 * i + 0];
		out.vertices[i].pos.y = in.vertices[3 * i + 1];
		out.vertices[i].pos.z = in.vertices[3 * i + 2];
	}
}

void CSG_Union(MeshModel* m1, MeshModel* m2, MeshDocument* md, const int& which_jaw)
{
	FileMesh fm_res;
	//FileMesh *fm1 = new FileMesh(), *fm2 = new FileMesh();
	CorkTriMesh *ctm1 = new CorkTriMesh(), *ctm2 = new CorkTriMesh(), *ctm_res = new CorkTriMesh();

	//readMeshModel(m1, fm1);					// 把 MeshModel* 转成 FileMesh*
	//readMeshModel(m2, fm2);

	//file2corktrimesh(*fm1, ctm1);				// 把 const FileMesh & 转成 CorkTriMesh*
	//file2corktrimesh(*fm2, ctm2);

	MeshModel_to_CorkTriMesh(m1, ctm1);			// 直接转了，不需要转成 FileMesh 过渡
	MeshModel_to_CorkTriMesh(m2, ctm2);

	if(isSolid(*ctm1) == false)
		printf("第一个模型不是 Solid 的!\n");
	if(isSolid(*ctm2) == false)
		printf("第二个模型不是 Solid 的!\n");

	computeUnion(*ctm1, *ctm2, ctm_res);

	corktrimesh2file(*ctm_res, fm_res);

	//delete fm1;
	//delete fm2;
	delete ctm1;
	delete ctm2;
	delete ctm_res;
////////////////////////////////// 将模型放入meshlist
	int s = 0;
	vector<Point3f> vertices;
	vector<Point3i> faces;

	for(int i = 0; i < fm_res.vertices.size(); ++i)
	{
		Point3f p;
		p.X() = fm_res.vertices[i].pos.x;
		p.Y() = fm_res.vertices[i].pos.y;
		p.Z() = fm_res.vertices[i].pos.z;
		vertices.push_back(p);
	}
	for(int i = 0; i < fm_res.triangles.size(); ++i)
	{
		Point3i aa;
		aa.X() = fm_res.triangles[i].a;
		aa.Y() = fm_res.triangles[i].b;
		aa.Z() = fm_res.triangles[i].c;
		faces.push_back(aa);
	}
	QString qlabel;
	if(which_jaw == 0)	qlabel = "U_aligner_tmp.obj";
	else				qlabel = "L_aligner_tmp.obj";

	MeshModel* new_mm;
	new_mm = md->addNewMesh("", qlabel, false);
	vcg::tri::ConcaveCover<CMeshO>(new_mm->cm, vertices, faces);

	new_mm->updateDataMask(MeshModel::MM_ALL);
	tri::UpdateBounding<CMeshO>::Box(new_mm->cm);
	tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFaceNormalized(new_mm->cm);

	// 将CSG结果分解多个连通区域，选择最大的
	CMeshO &cm = new_mm->cm;
	vector<pair<int, CMeshO::FacePointer> > connectedCompVec;
	int numCC = tri::Clean<CMeshO>::ConnectedComponents(cm, connectedCompVec);
	printf("Found %d Connected Components\n", numCC);
	if(numCC == 1)
	{
		if(which_jaw == 0)
		{
			new_mm->setLabel(QString("U_aligner.obj"));
			printf("完全连通，U_aligner.obj 有 %d 个点，%d 个面\n", new_mm->cm.vert.size(), new_mm->cm.face.size());
		}
		else
		{
			new_mm->setLabel(QString("L_aligner.obj"));
			printf("完全连通，L_aligner.obj 有 %d 个点，%d 个面\n", new_mm->cm.vert.size(), new_mm->cm.face.size());
		}
		return;
	}
	uint flag_for_seperate = 0;
	for(size_t i = 0; i < numCC; ++i)
	{
		tri::UpdateSelection<CMeshO>::ClearFace(cm);
		connectedCompVec[i].second->SetS();
		tri::UpdateSelection<CMeshO>::FaceConnectedFF(cm);
		tri::UpdateSelection<CMeshO>::ClearVertex(cm);
		tri::UpdateSelection<CMeshO>::VertexFromFaceLoose(cm);

		// MeshModel *destMesh= md->addNewMesh("",QString("CC %1").arg(i));
		MeshModel *destMesh = new MeshModel(md, "", QString("CC %1").arg(i));

		// 需要头文件 #include <vcg/complex/trimesh/append.h>
		tri::Append<CMeshO, CMeshO>::Mesh(destMesh->cm, cm, true);
		if(destMesh->cm.vert.size() < 10000)
		{
			delete destMesh;
			continue;
		}
		if(which_jaw == 0)	destMesh->setLabel(QString("U_aligner.obj"));
		else				destMesh->setLabel(QString("L_aligner.obj"));

		++flag_for_seperate;
		md->meshList.push_back(destMesh);
		printf("\t删除不连通区域后 %s 剩余 %d 个点，%d 个面\n", (const char*)destMesh->label().toLocal8Bit(), destMesh->cm.vert.size(), destMesh->cm.face.size());
		destMesh->updateDataMask(destMesh);
		tri::UpdateBounding<CMeshO>::Box(destMesh->cm);
		tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFaceNormalized(destMesh->cm);
	}
	if(flag_for_seperate != 1)
		printf("\n\nError seperating!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");

	// 如果选择了最大的连通区域，原来的meshmodel就不要了
	md->delMesh(new_mm);
	printf("\n");
	return;
}

// 融合牙齿和对应的附件，最后保存在m1中
void CSG_Union_Teeth_and_Att_matrix(MeshModel* m1, MeshModel* m2, vcg::Matrix44f move_matrix, MeshModel* res, const int& which_jaw)
{
	FileMesh fm_res;
	FileMesh *fm1 = new FileMesh(), *fm2 = new FileMesh();

	CorkTriMesh *ctm1 = new CorkTriMesh(), *ctm2 = new CorkTriMesh(), *ctm_res = new CorkTriMesh();	// 不写这个new，生成的是临时变量？

	readMeshModel_matrix(m1, fm1, move_matrix);
	readMeshModel_matrix(m2, fm2, move_matrix);

	file2corktrimesh(*fm1, ctm1);
	file2corktrimesh(*fm2, ctm2);

	computeUnion(*ctm1, *ctm2, ctm_res);

	corktrimesh2file(*ctm_res, fm_res);

	delete fm1;
	delete fm2;
	delete ctm1;
	delete ctm2;
	delete ctm_res;

	int s = 0;
	vector<Point3f> vertices;
	vector<Point3i> faces;
	for(int i = 0; i < fm_res.vertices.size(); ++i)
	{
		Point3f p;
		p.X() = fm_res.vertices[i].pos.x;
		p.Y() = fm_res.vertices[i].pos.y;
		p.Z() = fm_res.vertices[i].pos.z;
		vertices.push_back(p);
	}
	for(int i = 0; i < fm_res.triangles.size(); ++i)
	{
		Point3i aa;
		aa.X() = fm_res.triangles[i].a;
		aa.Y() = fm_res.triangles[i].b;
		aa.Z() = fm_res.triangles[i].c;
		faces.push_back(aa);
	}
	vcg::tri::ConcaveCover<CMeshO>(res->cm, vertices, faces);
}


void CSG_Union_Teeth_and_Att(MeshModel* m1, MeshModel* m2, MeshModel* res, const int& which_jaw)
{
	FileMesh fm_res;

	CorkTriMesh *ctm1 = new CorkTriMesh(), *ctm2 = new CorkTriMesh(), *ctm_res = new CorkTriMesh();	// 不写这个new，生成的是临时变量？

	MeshModel_to_CorkTriMesh(m1, ctm1);
	MeshModel_to_CorkTriMesh(m2, ctm2);

	computeUnion(*ctm1, *ctm2, ctm_res);

	corktrimesh2file(*ctm_res, fm_res);

	delete ctm1;
	delete ctm2;
	delete ctm_res;

	int s = 0;
	vector<Point3f> vertices;
	vector<Point3i> faces;
	for(int i = 0; i < fm_res.vertices.size(); ++i)
	{
		Point3f p;
		p.X() = fm_res.vertices[i].pos.x;
		p.Y() = fm_res.vertices[i].pos.y;
		p.Z() = fm_res.vertices[i].pos.z;
		vertices.push_back(p);
	}
	for(int i = 0; i < fm_res.triangles.size(); ++i)
	{
		Point3i aa;
		aa.X() = fm_res.triangles[i].a;
		aa.Y() = fm_res.triangles[i].b;
		aa.Z() = fm_res.triangles[i].c;
		faces.push_back(aa);
	}
	vcg::tri::ConcaveCover<CMeshO>(res->cm, vertices, faces);
}