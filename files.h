#pragma once

#include "rawMesh.h"
#include "common/interfaces.h"
#include <vcg/complex/trimesh/clean.h>
#include <vcg/complex/trimesh/append.h>
#include <vcg/complex/trimesh/update/topology.h>
#include <vcg/complex/trimesh/update/normal.h>
#include <vcg/complex/trimesh/update/flag.h>
#include <vcg/complex/trimesh/update/selection.h>
#include <vcg/complex/trimesh/update/bounding.h>
#include <vcg/complex/trimesh/smooth.h>

/* Files provides a wrapper for different file types and a common data view for the rest of the program.
This wrapper was introduced to make it easier to support multiple file types using other people's file importer/exporter code */

// all functions with integer return values here are intended to return an error count as if they were a main f_unction
struct FileVertex : public MinimalVertexData {};
struct FileTriangle : public MinimalTriangleData {};
typedef RawMesh<FileVertex, FileTriangle> FileMesh;

void CSG_Union(MeshModel* m1, MeshModel* m2, MeshDocument* md, const int& which_jaw);
void CSG_Union_Teeth_and_Att_matrix(MeshModel* m1, MeshModel* m2, vcg::Matrix44f move_matrix, MeshModel* res, const int& which_jaw);