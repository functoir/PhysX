//////////////////////////////////////////////////////////////////////////
//// Dartmouth Physical Computing Programming Assignment 2: SPH Particle Fluid
//// Author: AMITTAI WEKESA
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
        for (int i = 0; i < pow(3, d); i++) {
            auto coordinate = Cell_Coord(pos);
            auto iter = voxels.find(Nb_R(coordinate, i));
            if (iter != voxels.end()) {
                std::vector<int> bucket = iter->second;
                for (int& j : bucket) {
                    auto distance = (pos - points[j]).norm();
                    if (distance <= kernel_radius)
                    {
                        nbs.push_back(j);
                    }
                }
            }
        }
		/* Your implementation end */
		return !nbs.empty();
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
    std::vector<int> boundary_particles;
	SpatialHashing<d> spatial_hashing;
	Kernel<d> kernel;

	double kernel_radius=(double).8;			////kernel radius
	double pressure_density_coef=(double)1e1;	////pressure-density-relation coefficient, used in Update_Pressure()
	double density_0=(double)10.;				////rest density, used in Update_Pressure()
	double viscosity_coef=(double)1e1;			////viscosity coefficient, used in Update_Viscosity_Force()
	double kd=(double)1e2;						////stiffness for environmental collision response
//	VectorD g=VectorD::Unit(1)*(double)-1.;	////gravity
	VectorD g = VectorD::Zero();
	
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
        
        // track boundary particles
        boundary_particles.clear();

		neighbors.resize(particles.Size());
		for(int i=0;i<particles.Size();i++){
			std::vector<int> nbs;
			spatial_hashing.Find_Nbs(particles.X(i),particles.XRef(),kernel_radius,nbs);
			neighbors[i]=nbs;
            
            /* if less than 10 neighbors, assume it is a boundary particle
               This is a hack,
               and I do not have any scientific justification for it,
               but it seems to work, so I'm inclined to not fix it.
               I printed out the number of neighbors and the averages
               were roughly [14 to 18], so I think < 10 is a good threshold. */
            
            if (nbs.size() < 10) {
                boundary_particles.push_back(i);
            }
        }
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
        Update_Particle_Surface_Tension();

		for(int i=0;i<particles.Size();i++){
			particles.V(i)+=particles.F(i)/particles.D(i)*dt;
			particles.X(i)+=particles.V(i)*dt;}
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P2 TASK): update the density (particles.D(i)) of each particle based on the kernel function (Wspiky)
	void Update_Density()
	{
		/* Your implementation start */
        for (int i = 0; i < particles.Size(); i++)
        {
            particles.D(i) = 0;
            for (int j : neighbors[i])
            {
                particles.D(i) += kernel.Wspiky(particles.X(i) - particles.X(j)) * particles.M(j);
            }
        }
		/* Your implementation end */
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P2 TASK): update the pressure (particles.P(i)) of each particle based on its current density (particles.D(i)) and the rest density (density_0)
	void Update_Pressure()
	{
		/* Your implementation start */
        for (int i = 0; i < particles.Size(); i++)
        {
            particles.P(i) = pressure_density_coef * (particles.D(i) - density_0);
        }
		/* Your implementation end */
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P2 TASK): compute the pressure force for each particle based on its current pressure (particles.P(i)) and the kernel function gradient (gradientWspiky), and then add the force to particles.F(i)
	void Update_Pressure_Force()
	{
		/* Your implementation start */
        for (int i = 0; i < particles.Size(); i++) {
            VectorD pressure_force = VectorD::Zero();
            
            for (int& j : neighbors[i]) {
                auto average_pressure = (particles.P(i) + particles.P(j)) / 2;
                auto other_volume = particles.M(j) / particles.D(j);
                pressure_force += average_pressure * other_volume * kernel.gradientWspiky(particles.X(i) - particles.X(j));
            }
            particles.F(i) -= pressure_force;
        }
		/* Your implementation end */
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P2 TASK): compute the viscosity force for each particle
	/// based on its current velocity difference (particles.V(j)-particles.V(i))
	/// and the kernel function Laplacian (laplacianWvis),
	/// and then add the force to particles.F(i)
	void Update_Viscosity_Force()
	{
        /* Your implementation start */
        for (int i = 0; i < particles.Size(); i++) {
            VectorD viscosity(VectorD::Zero());
            
            for (int& j : neighbors[i]) {
                auto velocity_difference = particles.V(j) - particles.V(i);
                auto other_volume = particles.M(j) / particles.D(j);
                viscosity += velocity_difference * other_volume * kernel.laplacianWvis(particles.X(i) - particles.X(j));
                //TODO: should this be V(i)?
            }
            particles.F(i) += viscosity_coef * viscosity;
            
        }
        /* Your implementation end */
	}

	void Update_Body_Force()
	{
		for (int i = 0; i < particles.Size(); i++) {
			particles.F(i)+=particles.D(i)*g;
        }
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
	////YOUR IMPLEMENTATION (P2 TASK):
	/// In addition to the required function implementations,
	/// you are also asked to implement one additional feature to enhance the fluid effects.
	/// You may modify any part of the starter code for your implementation.
    
//    void Update_Particle_Surface_Tension() {
//        auto surface_tension_coef = 10;                           // good for controlling and testing
//        for (int i = 0; i < boundary_particles.size(); i++) {
//            VectorD surface_tension;
//            for (int& j: neighbors[i]) {
//
//                /* For each neighbor of a boundary particle,
//                 * Find a force of attraction between them constraining their movement to each other point
//                 * (To create the chain effect).
//                 *
//                 * I found one paper describing such a way of computing the surface tension force:
//                 * https://www.osti.gov/servlets/purl/1430982
//                 * I used the "height function approach" listed in the paper:
//                 *       F = coeff * curvature * (unit direction unit from particle to neighbor)
//                 *       where the curvature (as defined by calculus)
//                 *       is the scalar rate of change of the (unit gradient vector).
//                 *       It appears to work and has some adverse & interesting results when you tweak
//                 *       surface_tension_coef to higher values.
//                 *
//                 */
//                auto position_difference = particles.X(i) - particles.X(j);
//                auto position_difference_norm = position_difference.norm() * 1/0.7;
//                auto position_difference_direction = position_difference.normalized() * 0.7;
//                auto gradient = kernel.gradientWspiky(position_difference_direction);
//                auto gradient_norm = gradient.norm() * 1/0.7;
//                auto gradient_direction = gradient.normalized() * 0.7;
//                auto curvature = position_difference_norm * gradient_norm * (kernel.gradientWspiky(gradient_direction)).norm();
//                surface_tension = surface_tension_coef * curvature * gradient_direction;
//                particles.F(i) += surface_tension;
//                particles.F(j) -= surface_tension;
//            }
//        }
//    }
    
    void Update_Particle_Surface_Tension() {
        auto surface_tension_coef = 200;                           // good for controlling and testing
        for (int i = 0; i < particles.Size(); i++) {
            VectorD total = VectorD::Zero();
//            for (int& j: neighbors[i]) {
            for (int j = 0; j < particles.Size(); j++) {
                auto position_difference = particles.X(i) - particles.X(j);
                auto kernel_weight = kernel.Wspiky(position_difference);
                auto other_mass = particles.M(j);
                
                // compute curvature
                auto gradient = kernel.gradientWspiky(position_difference.normalized());
                auto gradient_direction = gradient.normalized() * 0.7;
                auto curvature = position_difference.norm() * gradient.norm() * (kernel.gradientWspiky(gradient_direction)).norm();
//                total += -curvature * kernel_weight * other_mass * position_difference;
                total += -kernel_weight * other_mass * position_difference;
            }
            particles.F(i) += surface_tension_coef * total;
        }
    
    }
    
    


};

#endif
