#ifndef __GridFluid_h__
#define __GridFluid_h__
#include "Common.h"
#include "Grid.h"
#include "Particles.h"

//////////////////////////////////////////////////////////////////////////
////Grid fluid simulator
template<int d> class GridFluid
{using VectorD=Vector<real,d>;using VectorDi=Vector<int,d>;
public:
	Grid<d> grid;
	Array<VectorD> u;		////velocity on grid nodes
	Array<real> div_u;		////velocity divergence on grid nodes (right hand side of the Poisson equation)
	Array<real> p;			////pressure
	Array<real> vor;		////vorticity
	Array<real> smoke_den;	////smoke density

	int node_num=0;
	VectorD src_pos=VectorD::Ones()*(real).5;
	VectorD src_vel=VectorD::Unit(0)*(real)1.5;
	real src_radius=(real).1;

	virtual void Initialize()
	{
		int n=64;
		VectorDi cell_counts=VectorDi::Ones()*n;
		cell_counts[1]/=2;
		real dx=(real)2./(real)n;
		VectorD domain_min=VectorD::Zero();
		grid.Initialize(cell_counts,dx,domain_min);
		node_num=grid.node_counts.prod();

		u.resize(node_num,VectorD::Unit(0)*(real).01);
		div_u.resize(node_num,(real)0);
		p.resize(node_num,(real)0);
		vor.resize(node_num,(real)0);
		smoke_den.resize(node_num,(real)0);
	}

	////Timestep update
	void Advance(const real dt)
	{
		Source();
		Advection(dt);
		Vorticity_Confinement(dt);
		Projection();
	}

	////TASK: Advection step: advect BOTH velocity and density on the grid using the semi-Lagrangian method
	////Hint: read the helper functions between Line 158-192 and use (some of) them in your implementation
	virtual void Advection(real dt)
	{
		Array<VectorD> u_copy=u;
		Array<real> den_copy=smoke_den;

		for(int i=0;i<node_num;i++){
			u[i]=VectorD::Zero();

			/*Your implementation starts*/
			/*Your implementation ends*/
		}
	}

	virtual void Projection()
	{
		real dx=grid.dx;
		real dx2=grid.dx*grid.dx;

		////Projection step 1: calculate the velocity divergence on each node
		////Read this sample code to learn how to access data with the node index and coordinate
		std::fill(div_u.begin(),div_u.end(),(real)0);
		for(int i=0;i<node_num;i++){
			if(Bnd(i))continue;		////ignore the nodes on the boundary
			VectorDi node=Coord(i);
			div_u[i]=(real)0;

			for(int j=0;j<d;j++){
				VectorD u_1=u[Idx(node-VectorDi::Unit(j))];
				VectorD u_2=u[Idx(node+VectorDi::Unit(j))];
				div_u[i]+=(u_2[j]-u_1[j])/(2*dx);}
		}

		////TASK: Projection step 2: solve the Poisson's equation -lap p= div u 
		////using the Gauss-Seidel iterations
		std::fill(p.begin(),p.end(),(real)0);
		for(int iter=0;iter<40;iter++){
			for(int i=0;i<node_num;i++){
				if(Bnd(i))continue;		////ignore the nodes on the boundary
				VectorDi node=Coord(i);

				/*Your implementation starts*/
				/*Your implementation ends*/
			}
		}
		
		////TASK: Projection step 3: correct velocity with the pressure gradient
		for(int i=0;i<node_num;i++){
			if(Bnd(i))continue;		////ignore boundary nodes
			VectorDi node=Coord(i);
			VectorD grad_p=VectorD::Zero();

			/*Your implementation starts*/
			/*Your implementation ends*/
		}
	}

	////TASK: implement the key steps for vorticity confinement
	void Vorticity_Confinement(const real dt)
	{
		real dx=grid.dx;

		////Vorticity confinement step 1: update vorticity
		std::fill(vor.begin(),vor.end(),(real)0);
		for(int i=0;i<node_num;i++){
			if(Bnd(i))continue;		////ignore boundary nodes
			VectorDi node=Coord(i);
			vor[i]=(real)0;

			/*Your implementation starts*/
			/*Your implementation ends*/
		}

		////TASK: Vorticity confinement step 2: update N = (grad(|vor|)) / |grad(|vor|)|
		Array<VectorD> N(node_num,VectorD::Zero());
		for(int i=0;i<node_num;i++){
			if(Bnd(i))continue;		////ignore boundary nodes
			VectorDi node=Coord(i);
			N[i]=VectorD::Zero();

			/*Your implementation starts*/
			/*Your implementation ends*/
		}

		////TASK: Vorticity confinement step 3: calculate confinement force and use it to update velocity
		real vor_conf_coef=(real)4;
		for(int i=0;i<node_num;i++){
			if(Bnd(i))continue;		////ignore boundary nodes
			VectorD f=vor_conf_coef*dx*Cross(N[i],vor[i]);
			u[i]+=f*dt;	////we don't have mass by assuming density=1
		}
	}
	////Your tasks are finished here
	//////////////////////////////////////////////////////////////////////////

	////set source velocity and density
	void Source()
	{
		for(int i=0;i<node_num;i++){
			VectorD pos=grid.Node(i);
			if((pos-src_pos).norm()<src_radius){
				u[i]=src_vel;
				smoke_den[i]=(real)1.;
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	////vortex particles
	////This is the Gaussian kernel function you might need to use in calculating your confinement force
	real Kernel(const real length)
	{
		real r=grid.dx*(real)4;
		if(length>r)return (real)0;
		real coef1=(real)1/(real)2*(r*r);
		real coef2=(real)1/(r*r);
		return (real).1*coef2*exp(-length*length*coef1);
	}

	void Particle_Vorticity_Confinement(const real dt)
	{
		////Now we use vortex particles to preserve the vorticity in the domain
		////particles.C represents the vorticity carried on each particle
		////particles.I denotes whether the particle is valid for calculating vorticity confinement: skip the particle if(particles.I(i)==-1) 
		for(int i=0;i<node_num;i++){
			if(Bnd(i))continue;		////ignore boundary nodes
			VectorDi node=Coord(i);
			VectorD node_pos=grid.Node(node);
			vor[i]=(real)0;

			////Your implementation to calculate the confinement force for each grid node due to each particle
			////Hint: recall your implementation in the vorticity confinement function
		}		
	}
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	////READ: Helper functions
	////You may need these helper functions for your implementation
	//////////////////////////////////////////////////////////////////////////
	////return the node index given its coordinate
	int Idx(const Vector2i& node_coord) const 
	{return grid.Node_Index(node_coord);}
	
	////return the coordinate given its index
	VectorDi Coord(const int node_index) const
	{return grid.Node_Coord(node_index);}

	////return the node position given its index
	VectorD Pos(const int node_index) const
	{return grid.Node(node_index);}

	Vector2 Cross(const Vector2& v,const real w) const
	{return Vector2(v[1]*w,-v[0]*w);}

	////2D bi-linear interpolation for vectors or vectors (the type is specified by T)
	template<class T> T Interpolate(const Array<T>& u,VectorD& pos)
	{
		////clamp pos to ensure it is always inside the grid
		real epsilon=grid.dx*(real)1e-3;
		for(int i=0;i<d;i++){
			if(pos[i]<=grid.domain_min[i])pos[i]=grid.domain_min[i]+epsilon;
			else if(pos[i]>=grid.domain_max[i])pos[i]=grid.domain_max[i]-epsilon;}

		////calculate the index, fraction, and interpolated values from the array
		VectorD cell_with_frac=(pos-grid.domain_min)/grid.dx;
		VectorDi cell=cell_with_frac.template cast<int>();
		VectorD frac=cell_with_frac-cell.template cast<real>();
		return Interpolate_Helper<T>(cell,frac,u);
	}
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	////Other helper functions that you may not need in your implementation
	////2D bi-linear interpolation helper for vectors or vectors (the type is specified by T)
	template<class T> T Interpolate_Helper(const Vector2i& cell,const Vector2& frac,const Array<T>& u)
	{
		return ((real)1-frac[0])*((real)1-frac[1])*u[grid.Node_Index(cell)]
			+frac[0]*((real)1-frac[1])*u[grid.Node_Index(Vector2i(cell[0]+1,cell[1]))]
			+((real)1-frac[0])*frac[1]*u[grid.Node_Index(Vector2i(cell[0],cell[1]+1))]
			+frac[0]*frac[1]*u[grid.Node_Index(Vector2i(cell[0]+1,cell[1]+1))];
	}

	////check if a node is on the boundary of the grid 
	////given its coordinate or index
	bool Bnd(const Vector2i& node_coord) const
	{
		for(int i=0;i<d;i++){
			if(node_coord[i]==0||node_coord[i]==grid.node_counts[i]-1)
				return true;
		}
		return false;	
	}
	bool Bnd(const int node_index) const
	{return Bnd(Coord(node_index));}

	////Particles, for visualization only
public:
	Particles<d> particles;

	void Initialize_Visualization_Particles()
	{
		particles.Resize(1);
		particles.X(0)=VectorD::Zero();
	}

	void Update_Visualization_Particles(const real dt)
	{
		for(int i=0;i<particles.Size();i++){
			VectorD v=Interpolate(u,particles.X(i));
			particles.X(i)+=v*dt;
		}
	}
};

#endif
