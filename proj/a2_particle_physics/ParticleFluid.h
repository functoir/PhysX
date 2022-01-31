//////////////////////////////////////////////////////////////////////////
//// Dartmouth Physical Computing Programming Assignment 2: SPH Particle Fluid
//// Author: TODO: PUT YOUR NAME HERE
////////////////////////////////////////////////////////////////////////// 
#ifndef __ParticleFluid_h__
#define __ParticleFluid_h__
#include "Common.h"
#include "Particles.h"
#include "ImplicitGeometry.h"

//////////////////////////////////////////////////////////////////////////
////Kernel function
template<int d> class Kernel
{using VectorD=Vector<double,d>;
public:
	////precomputed coefs;
	double h;
	double coef_Wspiky;
	double coef_dWspiky;
	double coef_Wvis;
	double coef_d2Wvis;
	double pi=3.1415927;

	void Precompute_Coefs(double _h)
	{
		h=_h;
		coef_Wspiky=15.0/(pi*pow(h,6));
		coef_dWspiky=-45.0/(pi*pow(h,6));
		coef_Wvis=2*pi*pow(h,3);
		coef_d2Wvis=45.0/(pi*pow(h,6));
	}

	////Kernel Spiky
	double Wspiky(const VectorD& xji)
	{
		double r=xji.norm();
		if(r>=0&&r<=h){return 15.0/(pi*pow(h,6))*pow(h-r,3);}
		else{return 0;}
	}

	VectorD gradientWspiky(const VectorD& v){
		double r=v.norm();
		if(r<= h&&r>0){return -45.0/(pi*pow(h,6))*pow(h-r,2)*v/r;}
		else{return VectorD::Zero();}
	}

	////Kernel viscosity
	double Wvis(const VectorD& xji){
		double r=xji.norm();
		if(r>=0&&r<=h){return 15.0/(2*pi*pow(h,3))*((-pow(r,3)/(2*pow(h,3))+r*r/(h*h)+h/(2*r)-1));}
		else{return 0;}
	}
	double laplacianWvis(const VectorD& v){
		double r=v.norm();
		if(r<=h&&r>0){return 45.0/(pi*pow(h,6))*(h-r);}
		else{return 0;}
	}
};

//////////////////////////////////////////////////////////////////////////
////Spatial hashing
template<int d> class SpatialHashing
{using VectorD=Vector<double,d>;using VectorDi=Vector<int,d>;
public:
	double dx=1.;	////grid cell size
	std::unordered_map<VectorDi,std::vector<int> > voxels;

	void Update_Voxels(const std::vector<VectorD>& points)
	{Clear_Voxels();for(int i=0;i<(int)points.size();i++)Add_Point(i,points[i]);}

	void Clear_Voxels(){voxels.clear();}

	bool Add_Point(const int point_idx,const VectorD& point_pos)
	{
		VectorDi cell=Cell_Coord(point_pos);
		auto iter=voxels.find(cell);
		if(iter==voxels.end())iter=voxels.insert(std::make_pair(cell,std::vector<int>())).first;
		std::vector<int>& bucket=iter->second;
		bucket.push_back(point_idx);
		return true;
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P2 TASK): find all the neighboring particles within the "kernel_radius" around "pos" and record their indices in "nbs", the position of the particles are given in "points"
	////You need to traverse all the 3^d neighboring cells in the background grid around the cell occupied by "pos", and then check the distance between each particle in each neighboring cell and the given "pos"
	////Use the helper function Cell_Coord to get the cell coordinates for a given "pos"
	////Use the helper function Nb_R to get the cell coordinates of the ith neighboring cell around the cell "coord"
	bool Find_Nbs(const VectorD& pos,const std::vector<VectorD>& points,const double kernel_radius,/*returned result*/std::vector<int>& nbs) const
	{
		/* Your implementation start */
		/* Your implementation end */
		return nbs.size()>0;
	}

protected:	////Helper functions
	VectorDi Cell_Coord(const VectorD& pos) const
	{VectorD coord_with_frac=(pos)/dx;return coord_with_frac.template cast<int>();}
	Vector2i Nb_R(const Vector2i& coord,const int index) const
	{assert(index>=0&&index<9);int i=index/3;int j=index%3;return coord+Vector2i(-1+i,-1+j);}
	Vector3i Nb_R(const Vector3i& coord,const int index) const
	{assert(index>=0&&index<27);int i=index/9;int m=index%9;int j=m/3;int k=m%3;return coord+Vector3i(-1+i,-1+j,-1+k);}
};

//////////////////////////////////////////////////////////////////////////
////Particle fluid simulator
template<int d> class ParticleFluid
{using VectorD=Vector<double,d>;
public:
	Particles<d> particles;
	std::vector<std::vector<int> > neighbors;
	SpatialHashing<d> spatial_hashing;
	Kernel<d> kernel;

	double kernel_radius=(double).8;			////kernel radius
	double pressure_density_coef=(double)1e1;	////pressure-density-relation coefficient, used in Update_Pressure()
	double density_0=(double)10.;				////rest density, used in Update_Pressure()
	double viscosity_coef=(double)1e1;			////viscosity coefficient, used in Update_Viscosity_Force()
	double kd=(double)1e2;						////stiffness for environmental collision response
	VectorD g=VectorD::Unit(1)*(double)-1.;	////gravity
	
	////Environment objects
	std::vector<ImplicitGeometry<d>* > env_objects;

	virtual void Initialize()
	{
		kernel.Precompute_Coefs(kernel_radius);
	}

	virtual void Update_Neighbors()
	{
		spatial_hashing.Clear_Voxels();
		spatial_hashing.Update_Voxels(particles.XRef());

		neighbors.resize(particles.Size());
		for(int i=0;i<particles.Size();i++){
			std::vector<int> nbs;
			spatial_hashing.Find_Nbs(particles.X(i),particles.XRef(),kernel_radius,nbs);
			neighbors[i]=nbs;}
	}

	virtual void Advance(const double dt)
	{
		for(int i=0;i<particles.Size();i++){
			particles.F(i)=VectorD::Zero();}

		Update_Neighbors();
		Update_Density();
		Update_Pressure();
		Update_Pressure_Force();
		Update_Viscosity_Force();
		Update_Body_Force();
		Update_Boundary_Collision_Force();

		for(int i=0;i<particles.Size();i++){
			particles.V(i)+=particles.F(i)/particles.D(i)*dt;
			particles.X(i)+=particles.V(i)*dt;}
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P2 TASK): update the density (particles.D(i)) of each particle based on the kernel function (Wspiky)
	void Update_Density()
	{
		/* Your implementation start */
		/* Your implementation end */
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P2 TASK): update the pressure (particles.P(i)) of each particle based on its current density (particles.D(i)) and the rest density (density_0)
	void Update_Pressure()
	{
		/* Your implementation start */
		/* Your implementation end */
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P2 TASK): compute the pressure force for each particle based on its current pressure (particles.P(i)) and the kernel function gradient (gradientWspiky), and then add the force to particles.F(i)
	void Update_Pressure_Force()
	{
		/* Your implementation start */
		/* Your implementation end */
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P2 TASK): compute the viscosity force for each particle based on its current velocity difference (particles.V(j)-particles.V(i)) and the kernel function Laplacian (laplacianWvis), and then add the force to particles.F(i)
	void Update_Viscosity_Force()
	{
		/* Your implementation start */
		/* Your implementation end */
	}

	void Update_Body_Force()
	{
		for(int i=0;i<particles.Size();i++){
			particles.F(i)+=particles.D(i)*g;}	
	}

	void Update_Boundary_Collision_Force()
	{
		for(int i=0;i<particles.Size();i++){
			for(int j=0;j<env_objects.size();j++){
				double phi=env_objects[j]->Phi(particles.X(i));
				if(phi<particles.R(i)){
					VectorD normal=env_objects[j]->Normal(particles.X(i));
					particles.F(i)+=normal*kd*(particles.R(i)-phi)*particles.D(i);}}}
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P2 TASK): In addition to the required function implementations, you are also asked to implement one additional feature to enhance the fluid effects. You may modify any part of the starter code for your implementation.

};

#endif
