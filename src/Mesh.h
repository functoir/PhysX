//#####################################################################
// Mesh
// Dartmouth COSC 89.18/189.02: Computational Methods for Physical Systems, Assignment starter code
// Contact: Bo Zhu (bo.zhu@dartmouth.edu)
//#####################################################################
#ifndef __Mesh_h__
#define __Mesh_h__
#include <fstream>
#include "Common.h"

////Simplicial mesh
////template parameter d specifies the dimension of a vertex, e.g., d=2 -- 2d vertex; d=3 -- 3d vertex
////template parameter e_d specifies the dimension of an element (the number of vertices of a simplex), e.g., e_d=2 -- segment; e_d=3, triangle; e_d=4, tetrahedra
template<int d,int e_d> class SimplicialMesh
{using VectorD=Vector<double,d>;using VectorDi=Vector<int,d>;using VectorEi=Vector<int,e_d>;
public:
	////Basic attributes
	ArrayPtr<VectorD> vertices=nullptr;
	std::vector<VectorEi> elements;

	////Constructors
	SimplicialMesh(const ArrayPtr<VectorD> _vertices=nullptr)
	{
		if(_vertices==nullptr)vertices=std::make_shared<std::vector<VectorD> >();
		else vertices=_vertices;
	}

	////Copy constructor
	SimplicialMesh(const SimplicialMesh<d,e_d>& copy){*this=copy;}
	SimplicialMesh<d,e_d>& operator=(const SimplicialMesh<d,e_d>& copy)
	{
		if(vertices==nullptr)vertices=std::make_shared<std::vector<VectorD> >();
		*vertices=*(copy.vertices);
		elements=copy.elements;
		return *this;
	}

	////Access attributes
	static constexpr int Dim() {return d;}
	static constexpr int Element_Dim() {return e_d;}
	virtual std::vector<VectorD>& Vertices(){return *vertices.get();}
	virtual const std::vector<VectorD>& Vertices() const {return *vertices.get();}
	virtual std::vector<VectorEi>& Elements(){return elements;}
	virtual const std::vector<VectorEi>& Elements() const {return elements;}

	virtual void Clear()
	{
		if(vertices)vertices->clear();
		elements.clear();
	}
};


template<int d> class TetrahedronMesh : public SimplicialMesh<d,4>
{using VectorD=Vector<double,d>;using VectorDi=Vector<int,d>;using VectorEi=Vector4i;using Base=SimplicialMesh<d,4>;
public:
	TetrahedronMesh(const ArrayPtr<VectorD> _vertices=nullptr):Base(_vertices){}
};

template<int d> class TriangleMesh : public SimplicialMesh<d,3>
{using VectorD=Vector<double,d>;using VectorDi=Vector<int,d>;using VectorEi=Vector3i;using Base=SimplicialMesh<d,3>;
public:
	TriangleMesh(const ArrayPtr<VectorD> _vertices=nullptr):Base(_vertices){}
};

template<int d> class SegmentMesh : public SimplicialMesh<d,2>
{using VectorD=Vector<double,d>;using VectorDi=Vector<int,d>;using VectorEi=Vector2i;using Base=SimplicialMesh<d,2>;
public:
	SegmentMesh(const ArrayPtr<VectorD> _vertices=nullptr):Base(_vertices){}
};

////Helper functions

////Dimension and type conversion
////The reason to do dimension conversion on meshes is to guarantee the data can be visualized by our OpenGL viewer.
////Our current OpenGL viewer reads 3D data only. So if you want to visualize a 2D mesh, you need to convert it to 3D before writing to files.
template<class MESH_T1,class MESH_T2> void Dim_Conversion(const MESH_T1& mesh2,/*result*/MESH_T2& mesh3)
{
	mesh3.vertices->resize((int)(mesh2.vertices->size()));
	for(size_type i=0;i<(*mesh3.vertices).size();i++)
		Dim_Conversion<double,MESH_T1::Dim(),MESH_T2::Dim()>((*mesh2.vertices)[i],(*mesh3.vertices)[i],(double)0);
	mesh3.elements.resize((int)mesh2.elements.size());
	for(size_type i=0;i<mesh2.elements.size();i++)
		Dim_Conversion<int,MESH_T1::Element_Dim(),MESH_T2::Element_Dim()>(mesh2.elements[i],mesh3.elements[i],(int)-1);
}

inline Vector3 Normal(const Vector3& p1,const Vector3& p2,const Vector3& p3){return (p2-p1).cross(p3-p1).normalized();}

inline void Update_Normals(const TriangleMesh<3>& mesh,std::vector<Vector3>& normals)
{
    normals.resize(mesh.Vertices().size(),Vector3::Zero());
    for(const auto& v:mesh.elements){Vector3 n=Normal(mesh.Vertices()[v[0]],mesh.Vertices()[v[1]],mesh.Vertices()[v[2]]);for(int j=0;j<3;j++){normals[v[j]]+=n;}}
    for(auto& n:normals){n.normalize();}
}

inline int Element_Edges(const Vector3i& v,std::vector<Vector2i>& edges)
{edges[0]=Vector2i(v[0],v[1]);edges[1]=Vector2i(v[1],v[2]);edges[2]=Vector2i(v[2],v[0]);return 3;}

inline Vector2i Sorted(const Vector2i& v){return v[0]>v[1]?v:Vector2i(v[1],v[0]);}

template<int d> void Get_Edges(const TriangleMesh<d>& mesh,std::vector<Vector2i>& edges)
{
	Hashset<Vector2i> edge_hashset;std::vector<Vector2i> element_edges(3);
	for(const auto& vtx:mesh.elements){
		int n=Element_Edges(vtx,element_edges);
		for(int i=0;i<n;i++)edge_hashset.insert(Sorted(element_edges[i]));}
	for(const auto& edge:edge_hashset)edges.push_back(edge);
}

template<int d> void Subdivide(TriangleMesh<d>* mesh)
{
	std::vector<Vector2i> edges;Get_Edges(*mesh,edges);
	std::unordered_map<Vector2i,int> edge_vtx_hashtable;
	for(const auto& e:edges){
		Vector<double,d> pos=(double).5*(mesh->Vertices()[e[0]]+mesh->Vertices()[e[1]]);
		mesh->Vertices().push_back(pos);
		int i=(int)mesh->Vertices().size()-1;
		edge_vtx_hashtable.insert(std::make_pair(e,i));}

	auto n=mesh->elements.size();
	for(auto i=0;i<n;i++){const Vector3i v=mesh->elements[i];int v3,v4,v5;
		{auto search=edge_vtx_hashtable.find(Sorted(Vector2i(v[0],v[1])));if(search==edge_vtx_hashtable.end())continue;v3=search->second;}
		{auto search=edge_vtx_hashtable.find(Sorted(Vector2i(v[1],v[2])));if(search==edge_vtx_hashtable.end())continue;v4=search->second;}
		{auto search=edge_vtx_hashtable.find(Sorted(Vector2i(v[2],v[0])));if(search==edge_vtx_hashtable.end())continue;v5=search->second;}
		mesh->elements.push_back(Vector3i(v[0],v3,v5));
		mesh->elements.push_back(Vector3i(v3,v[1],v4));
		mesh->elements.push_back(Vector3i(v5,v4,v[2]));
		mesh->elements[i]=Vector3i(v3,v4,v5);}
}

inline void Initialize_Icosahedron_Mesh(const double scale,TriangleMesh<3>* mesh)
{
	////http://donhavey.com/blog/tutorials/tutorial-3-the-icosahedron-sphere/
	const double tao=1.61803399f;
	double vtx_pos[12][3]={{1,tao,0},{-1,tao,0},{1,-tao,0},{-1,-tao,0},{0,1,tao},{0,-1,tao},{0,1,-tao},{0,-1,-tao},{tao,0,1},{-tao,0,1},{tao,0,-1},{-tao,0,-1}};
	int ele[20][3]={{0,1,4},{1,9,4},{4,9,5},{5,9,3},{2,3,7},{3,2,5},{7,10,2},{0,8,10},{0,4,8},{8,2,10},{8,4,5},{8,5,2},{1,0,6},{11,1,6},{3,9,11},{6,10,7},{3,11,7},{11,6,7},{6,0,10},{9,1,11}};		

	mesh->Clear();
	int vtx_num=12;mesh->Vertices().resize(vtx_num);for(int i=0;i<vtx_num;i++){mesh->Vertices()[i]=Vector3(vtx_pos[i][0],vtx_pos[i][1],vtx_pos[i][2])*scale;}
	int ele_num=20;mesh->elements.resize(ele_num);for(int i=0;i<ele_num;i++)mesh->elements[i]=Vector3i(ele[i][0],ele[i][1],ele[i][2]);
}

inline void Initialize_Sphere_Mesh(const double r,TriangleMesh<3>* mesh,const int sub=2)
{
	Initialize_Icosahedron_Mesh(r,mesh);for(int i=0;i<sub;i++)Subdivide(mesh);
	for(auto& v:mesh->Vertices()){double length=v.norm();double rs=r/length;v*=rs;}
}

#endif

