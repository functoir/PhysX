//////////////////////////////////////////////////////////////////////////
//// Dartmouth Physical Computing Programming Assignment 2: DEM Particle Sand
//// Author: AMITTAI WEKESA
////////////////////////////////////////////////////////////////////////// 

#ifndef __ParticleSand_h__
#define __ParticleSand_h__
#include "Common.h"
#include "Particles.h"
#include "ImplicitGeometry.h"
//#include "InClassDemoDriver.h"

template<int d> class ParticleSand
{using VectorD=Vector<double,d>;using VectorDi=Vector<int,d>;using MatrixD=Matrix<double,d>;
public:
	Particles<d> particles;
	double ks=(double)2e2;		////stiffness for the collision force
	double kd=(double).5e1;		////damping for the collision force
	VectorD g=VectorD::Unit(1)*(double)-1.;	////gravity

	////a list of implicit geometries describing the environment, by default it has one element, a circle with its normals pointing inward (Bowl)
	std::vector<ImplicitGeometry<d>* > env_objects;
    Curve<d>* my_object=nullptr;
    
	std::vector<Vector2i> particle_particle_collision_pairs;
	std::vector<Vector2i> particle_environment_collision_pairs;

	std::vector<VectorD> my_object_vertices={{-1.,5.},{1.,5.}};	////this array stores the positions of the contour of your object for visualization

	virtual void Advance(const double dt)
	{
		////Clear forces on particles
		for(int i=0;i<particles.Size();i++){
			particles.F(i)=VectorD::Zero();}

		////Accumulate body forces
		for(int i=0;i<particles.Size();i++){
			particles.F(i)+=particles.M(i)*g;}

		Particle_Environment_Collision_Detection();
		Particle_Environment_Collision_Response();
		Particle_Particle_Collision_Detection();
		Particle_Particle_Collision_Response();
		Particle_My_Object_Collision_Detection_And_Response();

		for(int i=0;i<particles.Size();i++){
			particles.V(i)+=particles.F(i)/particles.M(i)*dt;
			particles.X(i)+=particles.V(i)*dt;}
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
                
                if (env_objects[object_index]->Phi(particles.X(particle_index)) < particle_radius) {
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
//		for(auto& collision_pair : particle_environment_collision_pairs){
//			int particle_index = collision_pair[0];	////particle index
//			int object_index = collision_pair[1];	////env_objects index
//
//			/* Your implementation start */
//            double distance = env_objects[object_index]->Phi(particles.X(particle_index));
//            VectorD normal = env_objects[object_index]->Normal(particles.X(particle_index));
//            auto environment_velocity = VectorD::Zero();
//            VectorD spring_force = -ks * (distance - particles.R(particle_index)) * normal;
//            VectorD damping_force = -kd * (environment_velocity - particles.V(particle_index)).dot(normal) * normal;
//            VectorD collision_force = spring_force + damping_force;
//			/* Your implementation end */
//
//			particles.F(particle_index)+=collision_force;
//		}
        
        for(auto& collision_pair : particle_environment_collision_pairs){
            int particle_index=collision_pair[0];
            int object_index=collision_pair[1];
            VectorD collision_force=VectorD::Zero();

            VectorD env_vel=VectorD::Zero();

            double signed_dist = env_objects[object_index]->Phi(particles.X(particle_index));
            VectorD surface_normal = env_objects[object_index]->Normal(particles.X(particle_index));

            VectorD spring_force = ks * (signed_dist - particles.R(particle_index)) * (-1.0 * surface_normal);
            VectorD dampening_force = kd * ((env_vel - particles.V(particle_index)).dot(-1.0 * surface_normal)) * (-1.0 * surface_normal);

            collision_force = spring_force + dampening_force;

            particles.F(particle_index)+=collision_force;
        }
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P1 TASK): find all the pairs of particles that are colliding each other and record the detected pairs in particle_particle_collision_pairs
	////Each element in particle_particle_collision_pairs is a Vector2i specifying the indices of the two colliding particles
	virtual void Particle_Particle_Collision_Detection()
	{
		particle_particle_collision_pairs.clear();
		/* Your implementation start */
        for (int i = 0; i < particles.Size(); i++)
        {
            // NOTE: No need to consider second indices before i --
            // only need to detect each collision pair once.
            for (int j = i + 1; j < particles.Size(); j++) {
                
                auto distance = (particles.X(i) - particles.X(j)).norm();
                
                if (distance < particles.R(i) + particles.R(j)) {
                    particle_particle_collision_pairs.emplace_back(i, j); // implicit constructor
                }
            }
        }
		/* Your implementation end */
	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P1 TASK): compute penalty-based collision forces for pairs of colliding particles in particle_particle_collision_pairs and add the forces to particle.F 
	////The collision response force for each pair consists of a spring force and a damping force
	virtual void Particle_Particle_Collision_Response()
	{
        for (auto& collision_pair : particle_particle_collision_pairs){
            int first_particle = collision_pair[0];
            int second_particle = collision_pair[1];
            
            /* Your implementation start */
            
            // Compute forces
            VectorD& first_particle_position = particles.X(first_particle);
            VectorD& second_particle_position = particles.X(second_particle);
            
            double distance = (second_particle_position - first_particle_position).norm();
            VectorD normal = (second_particle_position - first_particle_position).normalized();
            
            VectorD& first_particle_velocity = particles.V(first_particle);
            VectorD& second_particle_velocity = particles.V(second_particle);
            
            double velocity_difference = (second_particle_velocity - first_particle_velocity).dot(normal);
            double least_possible_separation = particles.R(first_particle) + particles.R(second_particle);
            
            VectorD spring_force = ks * (distance - least_possible_separation) * normal;
            VectorD damping_force = kd * velocity_difference * normal;
            VectorD collision_force = spring_force + damping_force;
            
            // Update particles' forces.
            particles.F(first_particle) += collision_force;
            particles.F(second_particle) -= collision_force;
            /* Your implementation end */
            
        }

	}

	//////////////////////////////////////////////////////////////////////////
	////YOUR IMPLEMENTATION (P1 TASK): implement your the collision detection algorithm and calculate the response forces for your own object
	////YOUR IMPLEMENTATION (P1 TASK): visualize your object by filling up the contour vertex array
	////Note: the visualization does not have to be the same as the way you represent the geometry, 
	////E.g., you may use an implicit function to calculate the geometry and the collision force, but still use the vertices to draw it
	virtual void Particle_My_Object_Collision_Detection_And_Response()
	{
		////if you want to visualize your object, fill the array of my_object_shape with its contour vertices, the commented code provides an example
		my_object_vertices.clear();
		my_object_vertices.push_back(Vector2(-2.,5.));
		my_object_vertices.push_back(Vector2(2.,5.));

		for(int i=0;i<particles.Size();i++){
            auto particle_radius = particles.R(i);
            
            if (my_object->Phi(particles.X(i)) < particle_radius) {
                VectorD collision_force=VectorD::Zero();
    
                VectorD env_vel=VectorD::Zero();
    
                double signed_dist = my_object->Phi(particles.X(i));
                VectorD surface_normal = my_object->Normal(particles.X(i));
    
                VectorD spring_force = ks * (signed_dist - particles.R(i)) * (-1.0 * surface_normal);
                VectorD dampening_force = kd * ((env_vel - particles.V(i)).dot(-1.0 * surface_normal)) * (-1.0 * surface_normal);
    
                collision_force = spring_force + dampening_force;
    
                particles.F(i)+=collision_force;
            }
			VectorD collision_force=VectorD::Zero();
			
			/* Your implementation start */
//            int i =
			/* Your implementation end */
			
			particles.F(i)+=collision_force;
		}
        
        for(auto& collision_pair : particle_environment_collision_pairs){
            int particle_index=collision_pair[0];
            int object_index=collision_pair[1];
            VectorD collision_force=VectorD::Zero();
            
            VectorD env_vel=VectorD::Zero();
            
            double signed_dist = env_objects[object_index]->Phi(particles.X(particle_index));
            VectorD surface_normal = env_objects[object_index]->Normal(particles.X(particle_index));
            
            VectorD spring_force = ks * (signed_dist - particles.R(particle_index)) * (-1.0 * surface_normal);
            VectorD dampening_force = kd * ((env_vel - particles.V(particle_index)).dot(-1.0 * surface_normal)) * (-1.0 * surface_normal);
            
            collision_force = spring_force + dampening_force;
            
            particles.F(particle_index)+=collision_force;
        }
	}
};

#endif
