//#####################################################################
// Mass spring driver
// Dartmouth COSC 89.18/189.02: Computational Methods for Physical Systems, Assignment starter code
// Contact: Bo Zhu (bo.zhu@dartmouth.edu)
//#####################################################################
#ifndef __MassSpringInteractiveDriver_h__
#define __MassSpringInteractiveDriver_h__
#include <memory>
#include "Common.h"
#include "Mesh.h"
#include "BaseDriver.h"
#include "SoftBodyConstraintDynamics.h"

class MassSpringInteractivDriver : public BaseDriver
{
	using Base=Driver;
public:
	SoftBodyConstraintDynamics soft_body;
	const double dt=(double).02;

	////visualization data
	Segments segments;
	std::vector<Point> points;
	
	////initialize simulation data and its visualizations
	virtual void Initialize_Data()
	{
		Initialize_Simulation_Data();
		
		segments.Initialize(this);
		segments.Sync_Data(soft_body.particles.XRef(),soft_body.visualizer_springs);

		int n=soft_body.particles.Size();
		points.resize(n);
		for(int i=0;i<n;i++){
			points[i].Set_Radius(.02);
			points[i].Initialize(this);
			points[i].Sync_Data(soft_body.particles.X(i));
		}
	}

	////advance simulation timesteps
	virtual void Advance(const double dt)
	{
		soft_body.Advance(dt);
	}

	////update simulation data to its visualization counterparts
	virtual void Sync_Simulation_And_Visualization_Data()
	{
		segments.Sync_Data(soft_body.particles.XRef());
		int n=soft_body.particles.Size();
		for(int i=0;i<n;i++){
			points[i].Sync_Data(soft_body.particles.X(i));
		}
	}

	////here we initialize three tests for rod, cloth, and beam
	virtual void Initialize_Simulation_Data()
	{
        
        std::cout << "test : " << test << std::endl;
        
		switch(test){
		case 1:{	////1d rod
			////initialize spring vertices
			auto length=(double)1;
            int n=4;
            double dx=length/(double)n;
			soft_body.particles.Resize(n);
			for(int i=0;i<n;i++){
				soft_body.particles.X(i)=Vector3::Unit(0)*(double)i * dx;
				soft_body.particles.M(i)=(double)1;
                soft_body.particles.R(i) = 0.01;
            }
            soft_body.particles.InitializeWeights();
            
            for (int i = 0; i < soft_body.particles.Size(); i++) {
                std:: cout << "M = " << soft_body.particles.M(i) << "W = " << soft_body.particles.W(i) << std::endl;
            }
            
            // initialize constraints
            for (int i = 0; i < n-1; i++) {
                soft_body.innate_constraints.emplace_back( i, i+1 );
                soft_body.innate_constraint_strengths.emplace_back(1);
                soft_body.visualizer_springs.emplace_back(i, i+1);
            }
            
			////set boundary conditions
			soft_body.Set_Boundary_Node(0);
            std::cout << "test 1 init done" << std::endl;
		} break;
        
        case 2: { // colliding pendulums
            auto length=(double)1;
            int n = 6;
            double dx=length/(double)n;
            soft_body.particles.Resize(3*n + 1);
            double step = length / (double) n;
            
            Vector3 second_start(-2 * step, 0, 0);
            Vector3 third_start = Vector3(-step, 0, 0);
            
            for(int i=0;i<n;i++){
                soft_body.particles.X(i)=Vector3::Unit(0)*(double)i * dx;
                soft_body.particles.M(i)=(double)1;
                soft_body.particles.R(i) = 0.01;
                
                soft_body.particles.X(i+n)= second_start + Vector3::Unit(0)*(double)i * -dx;
                soft_body.particles.M(i+n)=(double)1;
                soft_body.particles.R(i+n) = 0.01;
                
                soft_body.particles.X(i+2*n)= third_start + Vector3::Unit(1)*(double)i * dx;
                soft_body.particles.M(i+2*n)=(double)2;
                soft_body.particles.R(i+2*n) = 0.01;
                
                //// TODO: add a random free particle.
                if (i == n-1) {
                    soft_body.particles.X(3 * n) = Vector3::Unit(1) * (double)(i + 1.) * dx;
                    soft_body.particles.M(3 * n) = (double)1;
                    soft_body.particles.R(3 * n) = 0.01;
                }
            }
            soft_body.particles.InitializeWeights();
    
            // initialize constraints
            for (int i = 0; i < n-1; i++) {
                soft_body.innate_constraints.emplace_back( i, i+1 );
                soft_body.innate_constraint_strengths.emplace_back(1);
                soft_body.visualizer_springs.emplace_back(i, i+1);
                
                soft_body.innate_constraints.emplace_back( i+n, i+n+1 );
                soft_body.innate_constraint_strengths.emplace_back(1);
                soft_body.visualizer_springs.emplace_back(i+n, i+n+1);
                
                soft_body.innate_constraints.emplace_back( i+2*n, i+2*n+1 );
                soft_body.innate_constraint_strengths.emplace_back(1);
                soft_body.visualizer_springs.emplace_back(i+2*n, i+2*n+1);
            }
    
            ////set boundary conditions
            soft_body.Set_Boundary_Node(0);
            soft_body.Set_Boundary_Node(n);
            soft_body.Set_Boundary_Node(2*n);
            std::cout << "test 2 init done" << std::endl;
        } break;
		case 3:{	////2d cloth
			////create a cloth mesh
			auto length=(double)1;int width=4*scale;int height=6*scale;double step=length/(double)width;
			TriangleMesh<3> cloth_mesh;
			Build_Cloth_Mesh(width,height,step,&cloth_mesh,0,2);
			int n=(int)cloth_mesh.Vertices().size();
			std::vector<Vector2i> edges;Get_Mesh_Edges(cloth_mesh,edges);

			////copy cloth mesh vertices to particles
			soft_body.particles.Resize(n);
			for (int i = 0; i < n; i++) {
				soft_body.particles.X(i)=cloth_mesh.Vertices()[i];
				soft_body.particles.M(i)=(double)1;
                soft_body.particles.R(i) = 0.01;
            }
            
            soft_body.particles.InitializeWeights();
            
			////copy cloth mesh edges to constraints
			for (auto &edge : edges) {
                soft_body.innate_constraints.emplace_back(edge[0],edge[1]);
                soft_body.innate_constraint_strengths.emplace_back(1);
                soft_body.visualizer_springs.emplace_back(edge[0],edge[1]);
            }

			////set boundary conditions
			soft_body.Set_Boundary_Node(0);
			soft_body.Set_Boundary_Node(width-1);
		}break;
		case 4:{	////3d volumetric beam
			int n = 4 * scale;
            double dx = (double)1/(double)n;
            BuildBeamParticlesAndConstraints(n, dx);
			for (int i=0; i < 4; i++) {
                soft_body.Set_Boundary_Node(i);
            }
            
            soft_body.visualizer_springs.clear();
            for (auto& edge : soft_body.innate_constraints) {
                soft_body.innate_constraint_strengths.emplace_back(0.7);
                soft_body.visualizer_springs.emplace_back(edge.first, edge.second);
            }
		}break;

		//////////////////////////////////////////////////////////////////////////
		////YOUR IMPLEMENTATION (TASK 2: OPTION 1): simulate a single hair strand
		case 5:{
			soft_body.Initialize_Hair_Strand();
		}break;
        
        case 6: { // falling particles
            auto length=(double)20;
            int n = 500;
            double dx=length/(double)n;
            soft_body.particles.Resize(750);
    
            Vector3 first_start(-10, 0, 0);
            Vector3 second_start(-10, 1.0, 0);
            
            
    
            for(int i=0;i<n;i++){
                soft_body.particles.X(i)= first_start + Vector3::Unit(0)*(double)i * dx;
                soft_body.particles.M(i)=(double)1;
                soft_body.particles.R(i) = 0.01;
            }
            
            for (int i = 0; i < n-1; i++) {
                soft_body.Set_Boundary_Node(i);
            }
            
            for (int i = n; i < 750; i++) {
                soft_body.particles.X(i)= second_start + Vector3::Unit(0)*(double) (i - n) * 2 * dx;
                soft_body.particles.M(i)=(double)1;
                soft_body.particles.R(i) = 0.01;
            }
            soft_body.particles.InitializeWeights();
            std::cout << "test 6 init done" << std::endl;
        } break;
		}

		soft_body.Initialize();
	}

	//////////////////////////////////////////////////////////////////////////
	////These helper functions are all for creating meshes
protected:
	////Helper functions
	void Build_Cloth_Mesh(const int cell_num_0,const int cell_num_1,const double dx,TriangleMesh<3>* mesh,int axis_0=0,int axis_1=1)
	{
		mesh->elements.resize(2*(cell_num_0-1)*(cell_num_1-1));int t=0;
		for(int i=1;i<=cell_num_0-1;i++)for(int j=1;j<=cell_num_1-1;j++){ // counterclockwise node ordering
			if(i%2){mesh->elements[t++]=Vector3i(i+cell_num_0*(j-1),i+1+cell_num_0*(j-1),i+cell_num_0*j);mesh->elements[t++]=Vector3i(i+1+cell_num_0*(j-1),i+1+cell_num_0*j,i+cell_num_0*j);}
			else{mesh->elements[t++]=Vector3i(i+cell_num_0*(j-1),i+1+cell_num_0*(j-1),i+1+cell_num_0*j);mesh->elements[t++]=Vector3i(i+cell_num_0*(j-1),i+1+cell_num_0*j,i+cell_num_0*j);}}
		for(size_type i=0;i<mesh->elements.size();i++){mesh->elements[i]-=Vector3i::Ones();
		/*swap y and z*/int tmp=mesh->elements[i][1];mesh->elements[i][1]=mesh->elements[i][2];mesh->elements[i][2]=tmp;}
		for(int j=0;j<cell_num_1;j++)for(int i=0;i<cell_num_0;i++){Vector3 pos=Vector3::Zero();pos[axis_0]=(double)i*dx;pos[axis_1]=(double)j*dx;mesh->Vertices().push_back(pos);}
	}

	void Get_Mesh_Edges(const TriangleMesh<3>& mesh,std::vector<Vector2i>& edges)
	{
		Hashset<Vector2i> edge_hashset;ArrayF<Vector2i,6> element_edges;
		for(const auto& vtx:mesh.elements){
			edge_hashset.insert(Sorted(Vector2i(vtx[0],vtx[1])));
			edge_hashset.insert(Sorted(Vector2i(vtx[1],vtx[2])));
			edge_hashset.insert(Sorted(Vector2i(vtx[2],vtx[0])));}
		for(const auto& edge:edge_hashset)edges.push_back(edge);
	}
    
    void BuildBeamParticlesAndConstraints(int n, double dx, Vector3 pos=Vector3::Zero())
    {
        auto& particles = soft_body.particles;
        auto& edges = soft_body.innate_constraints;
        particles.Resize(n*4);
        for(int i=0;i<particles.Size();i++) {
            particles.M(i)=(double)1;
        }
        
        particles.InitializeWeights();
        for(int i=0; i < n; i++) {
            particles.X(i*4)=pos+Vector3(dx*(double)i,(double)0,(double)0);
            particles.X(i*4+1)=pos+Vector3(dx*(double)i,(double)0,(double)dx);
            particles.X(i*4+2)=pos+Vector3(dx*(double)i,(double)dx,(double)0);
            particles.X(i*4+3)=pos+Vector3(dx*(double)i,(double)dx,(double)dx);
            edges.emplace_back( i*4, i*4+1);
            edges.emplace_back( i*4+1, i*4+3);
            edges.emplace_back( i*4+3, i*4+2);
            edges.emplace_back( i*4+2, i*4 );
            if(i < n-1) {
                edges.emplace_back(i*4,i*4+4 );
                edges.emplace_back(i*4+1,i*4+5 );
                edges.emplace_back(i*4+2,i*4+6 );
                edges.emplace_back(i*4+3,i*4+7 );
                
                edges.emplace_back(i*4, i*4+7);
                edges.emplace_back(i*4+1,i*4+6);
                edges.emplace_back(i*4+2,i*4+5);
                edges.emplace_back(i*4+3,i*4+4);
            }
        }
    }

	Vector2i Sorted(const Vector2i& v){return v[0]>v[1]?v:Vector2i(v[1],v[0]);}
};
#endif