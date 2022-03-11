//////////////////////////////////////////////////////////////////////////
//// Dartmouth Physical Computing Programming Final Project: Position Based Dynamics
//// Author: AMITTAI
////////////////////////////////////////////////////////////////////////// 

#ifndef __SoftBodyMassSpring_h__
#define __SoftBodyMassSpring_h__
#include "Common.h"
#include "Particles.h"

class SoftBodyMassSpring
{
public:
	////Spring parameters
	Particles<3> particles;								//// The particle system. Each particle has the attributes of position, velocity, mass, and force. Read Particles.h in src to know the details.

    int connections = 0;
    
    int NUM_ITERATIONS = 10;
    double DAMPING_COEFFICIENT = 0.001;
    
    std::vector<std::pair<int, int>> innate_constraints;
    std::vector<double> innate_constraint_strengths;
    std::vector<Vector2i> visualizer_springs;
    std::vector<double> separation_distances;
    std::vector<Vector3> next_positions;
    
	////Boundary nodes
	std::unordered_map<int,Vector3> boundary_nodes;		//// boundary_nodes stores all nodes that are fixed in place.

	//// Gravitational Force
    Vector3 g=Vector3(0, -0.3, 0);			//// gravity

	virtual void Initialize()
	{
		separation_distances.resize(innate_constraints.size());
		for (int i = 0; i < (int)innate_constraints.size(); i++) {
            const std::pair<int, int> &s = innate_constraints[i];
			separation_distances[i] = (particles.X(s.first)-particles.X(s.second)).norm();}
        std::cout << "Passed 67 \n" << std::endl;

	}

	virtual void Advance(const double dt)
	{
        AdvancePBD(dt);
	}
	
	////Set boundary nodes
	void Set_Boundary_Node(const int p,const Vector3 v=Vector3::Zero()){boundary_nodes[p]=v;}
	
	bool Is_Boundary_Node(const int p) {
        return boundary_nodes.find(p) != boundary_nodes.end();
    }
    
    
    void AdvancePBD(double dt) {
        // TODO Step 1: apply forces.
        ApplyForce(dt);
        // TODO Step 2: damp velocities.
        DampVelocities();
        // TODO Step 3: calculate next positions.
        Project(next_positions, dt);
        // TODO Step 4: Generate collision constraints and project them.
        auto collision_constraints = GenerateCollisionConstraints();
        ProjectCollisionConstraints(collision_constraints, dt);
        // TODO: Step 6: update next positions array with collisions.
        Project(next_positions, dt);
        // TODO Step 5: propagate constraints.
        ProjectDistanceConstraints(next_positions, dt);
        // TODO Step 7: Enforce boundary conditions
        //  (reset movement since boundary particles are locked in place).
        for (int i = 0; i < (int)particles.Size(); i++) {
            if (Is_Boundary_Node(i)) {
                next_positions[i] = particles.X(i);
            }
        }
        // TODO Step 5: Commit updates to particles.
        UpdateParticles(next_positions, dt);
    }
    
    void ApplyForce(const double dt)  {
        for (int i = 0; i < (int) particles.Size(); i++) {
            particles.F(i) = g;
            particles.V(i) += particles.W(i) * particles.F(i) * dt;
        }
    }
    
    static Matrix3 GetCrossProductMatrix(Vector3& vec) {
        Matrix3 mat = Matrix3::Zero();
    
        mat(0, 1) = -vec[2];
        mat(0, 2) = vec[1];
    
        mat(1, 0) = vec[2];
        mat(1, 2) = -vec[0];
    
        mat(2, 0) = -vec[1];
        mat(2, 1) = vec[0];
        return mat;
    }
    
    void DampVelocities() {
        Vector3 sum_angular_momentum = Vector3::Zero();
        Matrix3 inertia_tensor = Matrix3::Zero();
        std::vector<Vector3> radii;
        
        // accumulate mass, positions, velocities
        double sum_mass = 0.;
        Vector3 sum_positions = Vector3::Zero();
        Vector3 sum_velocities = Vector3::Zero();
        for (int i = 0; i < (int) particles.Size(); i++) {
            sum_mass += particles.M(i);
            sum_positions += particles.X(i) * particles.M(i);
            sum_velocities += particles.V(i) * particles.M(i);
        }
        
        // find x_cm, v_cm
        Vector3 x_cm = sum_positions / sum_mass;
        Vector3 v_cm = sum_velocities / sum_mass;
        
        for (int i = 0; i < (int) particles.Size(); i++) {
            Vector3 r = particles.X(i) - x_cm;
            auto r_cross_v = r.cross(particles.V(i));
            Matrix3 r_matrix = GetCrossProductMatrix(r);
            inertia_tensor += (r_matrix * r_matrix.transpose()) * particles.M(i);
            
            sum_angular_momentum += r_cross_v * particles.M(i);
            
            radii.emplace_back(r);
        }
   
        if (inertia_tensor.determinant() != 0) {
            Vector3 omega = inertia_tensor.inverse() * sum_angular_momentum;
            for (int i = 0; i < (int) particles.Size(); i++) {
                Vector3 delta_v = v_cm + (omega.cross(radii[i])) - particles.V(i);
                particles.V(i) += delta_v * DAMPING_COEFFICIENT;
            }
        }
    }
    
    void Project(std::vector<Vector3>& next_positions, double dt) {
        next_positions.clear();
        for (int i = 0; i < (int) particles.Size(); i++) {
            next_positions.emplace_back(particles.X(i) + particles.V(i) * dt);
        }
    }
    
    std::vector<std::pair<int, int>> GenerateCollisionConstraints() {
        std::vector<std::pair<int, int>> collision_pairs;
        std::set<std::string> seen_collisions;
        for (int i = 0; i < particles.Size(); i++) {
            for (int j = i; j < particles.Size(); j++) {
                if (i != j) {
                    if ( (particles.X(i) - particles.X(j)).norm() < 1.5 * (particles.R(i) + particles.R(j)) ) {
                        collision_pairs.emplace_back(std::make_pair(i, j));
                    }
                }
            }
        }
        return collision_pairs;
    }
    
    void ProjectCollisionConstraints(std::vector<std::pair<int, int>> &constraints, const double dt) {
        auto step = 0;
        while (step++ < NUM_ITERATIONS) {
            // TODO: propagate collision constraints
            for (auto& constraint : constraints) {
                
                int i = constraint.first;
                int j = constraint.second;
                Vector3 normal = (particles.X(i) - particles.X(j)).normalized();
                Vector3 v_proj_i = particles.V(i).dot(normal) * normal;
                Vector3 v_proj_j = particles.V(j).dot(normal) * normal;
                
                // TODO: why 0.1? I had a bug and tried to trouble-shoot,
                //  somehow one of the two projection vectors turns out bigger than the other.
                particles.V(i) += 0.1 * v_proj_j;
                particles.V(j) -= v_proj_i;
            }
        }
    }
    
    void ProjectDistanceConstraints(std::vector<Vector3> &new_positions, const double dt) {
        for (int i = 0; i < innate_constraints.size(); i++) {
            std::pair<int, int> constraint = innate_constraints[i];
            int first = constraint.first;
            int second = constraint.second;
            double& default_separation = separation_distances[i];
            
            Vector3 delta_p = new_positions[first] - new_positions[second];
            double current_separation = delta_p.norm();
            Vector3 delta_x = (particles.X(first) - particles.X(second));
            
            Vector3 correction = innate_constraint_strengths[i] * (current_separation - default_separation) * (delta_p / current_separation);
            
            new_positions[first] += correction * -(particles.W(first) / (particles.W(first) + particles.W(second)));
            new_positions[second] += correction * (particles.W(second) / (particles.W(first) + particles.W(second)));
            
        }
    }
    
    void UpdateParticles(const std::vector<Vector3>& next_positions, const double dt) {
        for (int i = 0; i < (int) particles.Size(); i++) {
            particles.V(i) = (next_positions[i] - particles.X(i)) / dt;
            particles.X(i) = next_positions[i];
        }
    }
    

	void Enforce_Boundary_Condition()
	{
		/* Your implementation start */
        for (int i=0; i<particles.Size(); i++) {
            if (Is_Boundary_Node(i)) {
                particles.V(i) = boundary_nodes[i];
                particles.F(i) = Vector3::Zero();
            }
        }
		/* Your implementation end */
	}
 

	//////////////////////////////////////////////////////////////////////////
	void Initialize_Hair_Strand()
	{
        //// initialize the particles
        double length = 2.;                                      // hair length
        auto n = /*4;*/ (int) (length * 30);                     // number of particles for modeling hair
        double dX = length / n;                                  // particle spacing
       
        particles.Resize(n);                                // resize the particle array
        connections = 0;                                         // initialize the number of connections
        auto direct_constraints_weight = (double) 1, kd_0 = (double) 1;
        int angle = 0;
        for(int i=0; i<n; i++){
            auto dx = dX * 3 * cos(angle);
            auto dy = (double) i / 2 * dX;
            auto dz = dX * 3 * sin(angle);
            particles.X(i) = Vector3(dx, dy, dz);
            particles.M(i)=(double)1.;
            angle += 120;
        }
        
        particles.InitializeWeights();
        
        // initialize connection constraints
        auto l1 = (particles.X(1) - particles.X(0)).norm();
        for (int i = 0; i < n-1; i++) {
            std::pair<int, int> constraint = {i, i+1};
            innate_constraints.emplace_back(constraint);
            visualizer_springs.emplace_back(constraint.first, constraint.second );
            innate_constraint_strengths.emplace_back(direct_constraints_weight);
            connections++;
        }
        ////set boundary conditions
        Set_Boundary_Node(0);
        
        //// initialize the curl constraints
        auto secondary_constraints_weight = direct_constraints_weight / 100;
        
        for (int i=2; i < 10; i++) {
            addBendConstraints(n, i, secondary_constraints_weight * 5);
        }
        
        for (int i = 13; i < 30; i+=3) {
            addBendConstraints(n, i, secondary_constraints_weight * 5);
        }
		/* Your implementation end */
	}
    void addBendConstraints(int totalSprings, int step, double ksVal)
    {
        auto l = (particles.X(step) - particles.X(0)).norm();
        for (int i = 0; i < totalSprings-step; i++) {
            innate_constraints.emplace_back(i, i + step);
            innate_constraint_strengths.emplace_back(ksVal);
        }
    }
};
#endif