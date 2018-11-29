#include "files.h"
#include "cork.h"
#include <iostream>
#include "vec.h"
#include <vcg/complex/trimesh/create/platonic.h>
#include <vector>
using namespace std;

#ifndef uint
typedef unsigned int uint;
#endif

// new version - 2018.11.28 �޸� GY (�ϲ�ԭ���� readMeshModel �� file2corktrimesh)
void MeshModel_to_CorkTriMesh(const MeshModel& mm, CorkTriMesh *out)
{
	out->n_vertices  = mm.cm.vert.size();
	out->n_triangles = mm.cm.face.size();
	out->triangles = new uint[out->n_triangles * 3];
	out->vertices = new float[out->n_vertices  * 3];
	for(CMeshO::ConstFaceIterator fi = mm.cm.face.begin(); fi != mm.cm.face.end(); ++fi)
	{
		int i = fi - mm.cm.face.begin();
		out->triangles[3 * i]     = (int)(fi->V(0) - &*mm.cm.vert.begin());
		out->triangles[3 * i + 1] = (int)(fi->V(1) - &*mm.cm.vert.begin());
		out->triangles[3 * i + 2] = (int)(fi->V(2) - &*mm.cm.vert.begin());
	}
	for(CMeshO::ConstVertexIterator vi = mm.cm.vert.begin(); vi != mm.cm.vert.end(); ++vi)
	{
		int i = vi - mm.cm.vert.begin();
		out->vertices[3 * i]     = vi->P().X();
		out->vertices[3 * i + 1] = vi->P().Y();
		out->vertices[3 * i + 2] = vi->P().Z();
	}
}

// new version - 2018.11.28 GY ���ϱߵĲ���Ҫ�任����ľͶ���һ���任������������
void MeshModel_to_CorkTriMesh_withMoveMat(const MeshModel& mm, CorkTriMesh *out, const vcg::Matrix44f& move_matrix)
{
	out->n_vertices  = mm.cm.vert.size();
	out->n_triangles = mm.cm.face.size();
	out->triangles = new uint[out->n_triangles * 3];
	out->vertices = new float[out->n_vertices  * 3];
	for(CMeshO::ConstFaceIterator fi = mm.cm.face.begin(); fi != mm.cm.face.end(); ++fi)
	{
		int i = fi - mm.cm.face.begin();
		out->triangles[3 * i]     = (int)(fi->V(0) - &*mm.cm.vert.begin());
		out->triangles[3 * i + 1] = (int)(fi->V(1) - &*mm.cm.vert.begin());
		out->triangles[3 * i + 2] = (int)(fi->V(2) - &*mm.cm.vert.begin());
	}
	for(CMeshO::ConstVertexIterator vi = mm.cm.vert.begin(); vi != mm.cm.vert.end(); ++vi)
	{
		int i = vi - mm.cm.vert.begin();
		Point3f tmpP =  move_matrix * vi->P();
		out->vertices[3 * i]     = tmpP.X();
		out->vertices[3 * i + 1] = tmpP.Y();
		out->vertices[3 * i + 2] = tmpP.Z();
	}
}

// GY �ں�����ģ�ͣ�Ŀǰ�����ںϣ��������ݣ��ͣ������뵹���������Բ���Ҫ�任����
void CSG_Union(const MeshModel& m1, const MeshModel& m2, MeshDocument* md, const int& which_jaw)
{
	CorkTriMesh *ctm1 = new CorkTriMesh(), *ctm2 = new CorkTriMesh(), *ctm_res = new CorkTriMesh();
	MeshModel_to_CorkTriMesh(m1, ctm1);			// ֱ��ת�ˣ�����Ҫת�� FileMesh ����
	MeshModel_to_CorkTriMesh(m2, ctm2);
	if(isSolid(*ctm1) == false)	printf("��һ��ģ�Ͳ��� Solid ��!\n");
	if(isSolid(*ctm2) == false)	printf("�ڶ���ģ�Ͳ��� Solid ��!\n");

	computeUnion(*ctm1, *ctm2, ctm_res);

	delete ctm1;
	delete ctm2;

	vector<Point3f> vertices;
	vector<Point3i> faces;
	for(int i = 0; i < ctm_res->n_vertices; i++)
	{
		Point3f p;
		p.X() = ctm_res->vertices[3 * i];
		p.Y() = ctm_res->vertices[3 * i + 1];
		p.Z() = ctm_res->vertices[3 * i + 2];
		vertices.push_back(p);
	}
	for(int i = 0; i < ctm_res->n_triangles; i++)
	{
		Point3i f;	// һ�������ε��������������
		f.X() = ctm_res->triangles[3 * i];
		f.Y() = ctm_res->triangles[3 * i + 1];
		f.Z() = ctm_res->triangles[3 * i + 2];
		faces.push_back(f);
	}
	delete ctm_res;

	QString qlabel;
	if(which_jaw == 0)	qlabel = "U_aligner_tmp.obj";
	else				qlabel = "L_aligner_tmp.obj";

	MeshModel* new_mm;
	new_mm = md->addNewMesh("", qlabel, false);
	vcg::tri::ConcaveCover<CMeshO>(new_mm->cm, vertices, faces);

	new_mm->updateDataMask(MeshModel::MM_ALL);
	tri::UpdateBounding<CMeshO>::Box(new_mm->cm);
	tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFaceNormalized(new_mm->cm);

	// ��CSG����ֽ�����ͨ����ѡ������
	CMeshO &cm = new_mm->cm;
	vector<pair<int, CMeshO::FacePointer> > connectedCompVec;
	int numCC = tri::Clean<CMeshO>::ConnectedComponents(cm, connectedCompVec);
	printf("Found %d Connected Components\n", numCC);
	if(numCC == 1)
	{
		if(which_jaw == 0)
		{
			new_mm->setLabel(QString("U_aligner.obj"));
			printf("��ȫ��ͨ��U_aligner.obj �� %d ���㣬%d ����\n", new_mm->cm.vert.size(), new_mm->cm.face.size());
		}
		else
		{
			new_mm->setLabel(QString("L_aligner.obj"));
			printf("��ȫ��ͨ��L_aligner.obj �� %d ���㣬%d ����\n", new_mm->cm.vert.size(), new_mm->cm.face.size());
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

		// ��Ҫͷ�ļ� #include <vcg/complex/trimesh/append.h>
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
		printf("\tɾ������ͨ����� %s ʣ�� %d ���㣬%d ����\n", (const char*)destMesh->label().toLocal8Bit(), destMesh->cm.vert.size(), destMesh->cm.face.size());
		destMesh->updateDataMask(destMesh);
		tri::UpdateBounding<CMeshO>::Box(destMesh->cm);
		tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFaceNormalized(destMesh->cm);
	}
	if(flag_for_seperate != 1)
		printf("\n\nError seperating!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n\n");

	// ���ѡ����������ͨ����ԭ����meshmodel�Ͳ�Ҫ��
	md->delMesh(new_mm);
	printf("\n");
	return;
}

// �ں� ��������m1 �� ��Ӧ�ĸ���m2����󱣴���m1��(Ŀǰ��Ϊһ���������ֻ��һ������)
// ��Ϊ���ݺ͸���ʵ��������û�б�ģ�ֻ�Ǳ任����䣬������Ҫ����任����
void CSG_Union_Teeth_and_Att_matrix(const MeshModel& m1, const MeshModel& m2, vcg::Matrix44f move_matrix, MeshModel* res, const int& which_jaw)
{
	CorkTriMesh *ctm1 = new CorkTriMesh(), *ctm2 = new CorkTriMesh(), *ctm_res = new CorkTriMesh();	// ��д���new�����ɵ�����ʱ������

	MeshModel_to_CorkTriMesh_withMoveMat(m1, ctm1, move_matrix);
	MeshModel_to_CorkTriMesh_withMoveMat(m2, ctm2, move_matrix);

	computeUnion(*ctm1, *ctm2, ctm_res);

	delete ctm1;
	delete ctm2;

	vector<Point3f> vertices;
	vector<Point3i> faces;
	for(int i = 0; i < ctm_res->n_vertices; ++i)
	{
		Point3f p;
		p.X() = ctm_res->vertices[3 * i];
		p.Y() = ctm_res->vertices[3 * i + 1];
		p.Z() = ctm_res->vertices[3 * i + 2];
		vertices.push_back(p);
	}
	for(int i = 0; i < ctm_res->n_triangles; ++i)
	{
		Point3i aa;
		aa.X() = ctm_res->triangles[3 * i];
		aa.Y() = ctm_res->triangles[3 * i + 1];
		aa.Z() = ctm_res->triangles[3 * i + 2];
		faces.push_back(aa);
	}
	delete ctm_res;

	vcg::tri::ConcaveCover<CMeshO>(res->cm, vertices, faces);
}