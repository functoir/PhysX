//////////////////////////////////////////////////////////////////////////
//// Dartmouth Physical Computing Programming Assignment 2: DEM Particle Sand
//// Author: AMITTAI WEKESA
////////////////////////////////////////////////////////////////////////// 

#ifndef __ParticleSand_h__
#define __ParticleSand_h__
#include "Common.h"
#include "Particles.h"
#include "ImplicitGeometry.h"
#include "SoftBodyConstraintDynamics.h"
//#include "BaseDriver.h"

template<int d> class ParticleSand
{using VectorD=Vector<double,d>;using VectorDi=Vector<int,d>;using MatrixD=Matrix<double,d>;
public:
    
    SoftBodyConstraintDynamics dynamics;
    Particles<3>& particles = dynamics.particles;
	VectorD g=VectorD::Unit(1)*(double)-1.;	////gravity
    double ks=(double)2e2;		////stiffness for the collision force
    double kd=(double).5e1;		////damping for the collision force

	////a list of implicit geometries describing the environment, by default it has one element, a circle with its normals pointing inward (Bowl)
	std::vector<ImplicitGeometry<d>*> env_objects;
//    Curve<d>* my_object=nullptr;
	std::vector<Vector2i> particle_environment_collision_pairs;

//	std::vector<VectorD> my_object_vertices={{-1.,5.},{1.,5.}};	////this array stores the positions of the contour of your object for visualization

	virtual void Advance(const double dt)
	{
        //TODO: Note; previous iteration's forces are cleared by SoftBodyConstraintDynamics::Advance()
		////Accumulate body forces
		for(int i=0; i < dynamics.particles.Size();i++){
			dynamics.particles.F(i)+=dynamics.particles.M(i)*g;}
        

		Particle_Environment_Collision_Detection();
		Particle_Environment_Collision_Response();
        dynamics.Advance(dt);
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P1 TASK): detect collision between particles and env_objects (implicit surface) and record the detected collisions in particle_environment_collision_pairs
	////env_objects is a list of implicit geometries, by default there is only one geometry (the bowl) in the list
	////Each element in particle_environment_collision_pairs is a Vector2i, with the first index for the particle and the second index for the env_objects
	virtual void Particle_Environment_Collision_Detection()
	{
		particle_environment_collision_pairs.clear();
		/* Your implementation start */
        for (int particle_index = 0; particle_index < particles.Size(); particle_index++) {
            for (int object_index = 0; object_index < env_objects.size(); object_index++) {
                auto particle_radius = particles.R(particle_index);

                if (abs(env_objects[object_index]->Phi(particles.X(particle_index))) < particle_radius) {
                    particle_environment_collision_pairs.emplace_back(particle_index, object_index);
                }
            }
        }
		/* Your implementation end */
	}
		
	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P1 TASK): compute the penalty-based collision force for the particles that are colliding with the env_objects
	////The collision response force consists of a spring force and a damping force
	virtual void Particle_Environment_Collision_Response()
	{
        for(auto& collision_pair : particle_environment_collision_pairs){
            int particle_index=collision_pair[0];
            int object_index=collision_pair[1];
            VectorD normal = env_objects[object_index]->Normal(particles.X(particle_index));
            auto normal_vel = particles.V(particle_index).dot(normal);
            auto reverse_velocity = normal_vel * normal * -1;
            
            // TODO: This should be 2 (the effect of reversing that component of the velocity)
            particles.V(particle_index) += 2.0 * reverse_velocity;
        }
	}
};

#endif
