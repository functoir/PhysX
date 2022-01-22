//////////////////////////////////////////////////////////////////////////
//// Dartmouth Physical Computing Programming Assignment 1: Mass Spring
//// Author: AMITTAI WEKESA
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
	std::vector<Vector2i> springs;						//// Each element in springs stores a Vector2i for the indices of the two end points.
	std::vector<double> rest_length;					//// Each element in rest_length specifies the rest length of the spring
	std::vector<double> ks;								//// Each element in ks specifies the spring stiffness of a spring
	std::vector<double> kd;								//// Each element in kd specifies the damping coefficient of a spring

	////Boundary nodes
	std::unordered_map<int,Vector3> boundary_nodes;		//// boundary_notes stores the mapping from node index to its specified velocity. E.g., a fixed node will have a zero velocity.

	////Body force
	Vector3 g=Vector3::Unit(1)*(double)-1.;			//// gravity
	
	enum class TimeIntegration{ExplicitEuler,ImplicitEuler} time_integration=TimeIntegration::ExplicitEuler;	//// set to ExplicitEuler by default; change it to ImplicitEuler when you work on Task 2 (Option 2)

	////Implicit time integration
	SparseMatrixT K;
	VectorX u,b;

	virtual void Initialize()
	{
		////Initialize default spring parameters for standard tests
		auto ks_0 = (double) 1, kd_0 = (double) 1;
		switch(time_integration){
		case TimeIntegration::ExplicitEuler:{
			ks_0=(double)5e2;
			kd_0=(double)1e1;
		}break;
		case TimeIntegration::ImplicitEuler:{
			ks_0=(double)1e5;
			kd_0=(double)1e1;			
		}break;}

		////Allocate arrays for springs and parameters
		rest_length.resize(springs.size());
		for(int i=0;i<(int)springs.size();i++){const Vector2i& s=springs[i];
			rest_length[i]=(particles.X(s[0])-particles.X(s[1])).norm();}
		ks.resize(springs.size(),ks_0);
		kd.resize(springs.size(),kd_0);

		////Allocate sparse matrix if using implicit time integration 
		////This function needs to be called for only once since the mesh doesn't change during the simulation)
		if(time_integration==TimeIntegration::ImplicitEuler)
			Initialize_Implicit_K_And_b();
	}

	virtual void Advance(const double dt)
	{
		switch(time_integration){
		case TimeIntegration::ExplicitEuler:
			Advance_Explicit_Euler(dt);break;
		case TimeIntegration::ImplicitEuler:
			Advance_Implicit_Euler(dt);break;}
	}
	
	////Set boundary nodes
	void Set_Boundary_Node(const int p,const Vector3 v=Vector3::Zero()){boundary_nodes[p]=v;}
	
	bool Is_Boundary_Node(const int p){return boundary_nodes.find(p)!=boundary_nodes.end();}

	//////////////////////////////////////////////////////////////////////////
	//// P1 TASK: explicit Euler integration and spring force calculation

	//////////////////////////////////////////////////////////////////////////
	//// YOUR IMPLEMENTATION (TASK 1): explicit Euler time integration 
	void Advance_Explicit_Euler(const double dt)
	{
		//// Step 0: Clear the force on each particle (already done for you)
		Clear_Force();

		//// Step 1: add a body force to each particle
		Apply_Body_Force(dt);

		//// Step 2: calculate the spring force for each spring and add it to the two connecting particles (Hint: you may want to implement the function Spring_Force and call it in your for-loop)
		Apply_Spring_Force(dt);

		//// Step 3: enforce the boundary conditions (traversing the particles in the unordered_set, set its velocity to be the value read from the unordered_set, and its force to be zero)
		Enforce_Boundary_Condition();

		//// Step 4: time integration by updating the particle velocities according to their forces and updating particle positions according to the positions
		Time_Integration(dt);
	}

	void Clear_Force()
	{
		for (int i=0; i<particles.Size(); i++) {
            particles.F(i)=Vector3::Zero();
        }
	}

	void Apply_Body_Force(const double dt)
	{
		/* Your implementation start */
        for (auto i=0; i < particles.Size(); i++) {
            particles.F(i) += g * particles.M(i);
        }
		/* Your implementation end */	
	}

	void Apply_Spring_Force(const double dt)
	{
		/* Your implementation start */
        for (int i=0; i<springs.size(); i++) {
            Vector3 f = Spring_Force(i);
            const Vector2i& spring = springs[i];
            particles.F(spring[0]) += f;
            particles.F(spring[1]) -= f;
        }
		/* Your implementation end */	
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

	void Time_Integration(const double dt)
	{
		/* Your implementation start */
        for (int i=0; i<particles.Size(); i++) {
            auto acceleration = particles.F(i) / particles.M(i);
            particles.V(i) += acceleration * dt;
            particles.X(i) += particles.V(i) * dt;
        }
		/* Your implementation end */		
	}
	
	Vector3 Spring_Force(const int spring_index)
	{
		//// This is an auxiliary function to compute the spring force f=f_s+f_d for the spring with spring_index. 
		//// You may want to call this function in Apply_Spring_Force
		
		/* Your implementation start */
        const Vector2i& spring = springs[spring_index];
        Vector3 pos1 = particles.X(spring[0]);
        Vector3 pos2 = particles.X(spring[1]);
        Vector3 distance = pos2 - pos1;
        double length = distance.norm();
        Vector3 f = (ks[spring_index] * (length - rest_length[spring_index]) * distance.normalized());
        return f;
		/* Your implementation end */
	}

	//////////////////////////////////////////////////////////////////////////
	//// TASK 2 (OPTION 1): creating bending springs for hair strand simulation
	void Initialize_Hair_Strand()
	{
		//// You need to initialize a hair model by setting the springs and particles to simulate human hair.
		//// A key component for a hair simulator is the bending spring (e.g., by connecting particles with a certain index offset).
		//// Think about how to realize these bending and curly effects with the explicit spring model you have implemented in TASK 1.
		//// You may also want to take a look at the function Initialize_Simulation_Data() in MassSpringInteractiveDriver.h for the model initialization.
		
		/* Your implementation start */
        //// initialize the particles
        double length = 1.;                                      // hair length
        int n = 20;                                               // number of particles
        double dx = length / n;                                  // particle spacing
        particles.Resize(n);                                // resize the particle array
        
        auto ks_0 = (double) 1, kd_0 = (double) 1;
        switch(time_integration){
            case TimeIntegration::ExplicitEuler:{
                ks_0=(double)5e2;
                kd_0=(double)1e1;
            }break;
            case TimeIntegration::ImplicitEuler:{
                ks_0=(double)1e5;
                kd_0=(double)1e1;
            }break;
        }
        
        // initialize the particles
        for(int i=0; i<n; i++){
            particles.X(i) = Vector3::Unit(0.) * i * dx;
            particles.M(i) = 1.;
        }
        // initialize connection springs
        for (int i = 0; i < n-1; i++) {
            springs.emplace_back(Vector2i(i, i+1));
            rest_length.push_back(dx);
//            ks.push_back(ks_0);
        }
        ////set boundary conditions
        Set_Boundary_Node(0);
        
        //// initialize the curl springs
        auto dampening = 0.8;
        for (int i=0; i < n-2; i++) {
            springs.emplace_back(Vector2i(i, i + 2));
            rest_length.push_back(dx * dampening * 2);
//            ks.push_back(ks_0 * 0.6);
        }
        for (int i = 0; i < n-3; i++) {
            springs.emplace_back(Vector2i(i, i + 3));
            rest_length.push_back(dx * dampening * 2.8);
//            ks.push_back(ks_0 * 1.4);
        }
        for (int i = 0; i < n-4; i++) {
            springs.emplace_back(Vector2i(i, i + 4));
            rest_length.push_back(dx * dampening * 3.2);
//            ks.push_back(ks_0 * 1.5);
        }
        for (int i = 0; i < n-5; i++) {
            springs.emplace_back(Vector2i(i, i + 5));
            rest_length.push_back(dx * dampening * 3);
//            ks.push_back(ks_0 * 0.6);
        }
        for (int i = 0; i < n-6; i++) {
            springs.emplace_back(Vector2i(i, i + 6));
            rest_length.push_back(dx * dampening * 2.4);
//            ks.push_back(ks_0 * 0.6);
        }
        for (int i = 0; i < n-7; i++) {
            springs.emplace_back(Vector2i(i, i + 7));
            rest_length.push_back(dx * dampening * 1.9);
//            ks.push_back(ks_0 * 0.6);
        }
        
        ks.resize(springs.size(), ks_0);                                              // clear the spring stiffness array
//        for (int i = 0; i < n-8; i++) {
//            springs.emplace_back(Vector2i(i, i + 8));
//            rest_length.push_back(0.2);
//            ks.push_back(ks_0 * 2.0);
//        }
//        for (int i = 0; i < n-9; i++) {
//            springs.emplace_back(Vector2i(i, i + 9));
//            rest_length.push_back(0.2);
//            ks.push_back(ks_0 * 2.0);
//        }
        //// initialize the rest length
//        for (int i=0; i<n; i++) {
//            rest_length.push_back(0.1);
//        }
//        for (int i = n; i < particles.Size(); i++) {
//            if (i % 2 == 0) {
//                rest_length.push_back(0.15);
//            } else {
//                rest_length.push_back(0.17);
//            }
//        }
		/* Your implementation end */
	}

	//////////////////////////////////////////////////////////////////////////
	//// TASK 2 (OPTION 2): implicit time integration for inextensible cloth simulation
	//// The rest part of this file is all for this task.
	
	////Construct K, step 1: initialize the matrix structure 
	void Initialize_Implicit_K_And_b()
	{
		int n=3*particles.Size();
		K.resize(n,n);u.resize(n);u.fill((double)0);b.resize(n);b.fill((double)0);
		std::vector<TripletT> elements;
		for(auto &spring : springs){
            int i=spring[0];
            int j=spring[1];
			Add_Block_Triplet_Helper(i,i,elements);
			Add_Block_Triplet_Helper(i,j,elements);
			Add_Block_Triplet_Helper(j,i,elements);
			Add_Block_Triplet_Helper(j,j,elements);}
		K.setFromTriplets(elements.begin(),elements.end());
		K.makeCompressed();	
	}
	
	////Construct K, step 2: fill nonzero elements in K
	void Update_Implicit_K_And_b(const double dt)
	{
		////Clear K and b
		K.setZero();
		b.fill((double) 0);

		/* Your implementation start */

		/* Your implementation end */
	}

	////Construct K, step 2.1: compute spring force derivative
	void Compute_Ks_Block(const int s,Matrix3& Ks)
	{
		/* Your implementation start */
  
		/* Your implementation end */
	}

	////Construct K, step 2.2: compute damping force derivative
	void Compute_Kd_Block(const int s,Matrix3& Kd)
	{
		/* Your implementation start */

		/* Your implementation end */
	}

	////Implicit Euler time integration
	void Advance_Implicit_Euler(const double dt)
	{
		//// clear force
		Clear_Force();
		//// add a body force to each particle as in explicit Euler
		Apply_Body_Force(dt);
		//// calculate the spring force as in explicit Euler
		Apply_Spring_Force(dt);
		//// enforce boundary condition
		Enforce_Boundary_Condition();

		Update_Implicit_K_And_b(dt);

		for(int i=0;i<particles.Size();i++){
			for(int j=0;j<3;j++)u[i*3+j]=particles.V(i)[j];}	////set initial guess to be the velocity from the last time step

		SparseSolver::CG(K,u,b);	////solve Ku=b using Conjugate Gradient

		for(int i=0;i<particles.Size();i++){
			Vector3 v;for(int j=0;j<3;j++)v[j]=u[i*3+j];
			particles.V(i)=v;
			particles.X(i)+=particles.V(i)*dt;}
	}

	////Hint: you may want to use these functions when assembling your implicit matrix
	////Add block nonzeros to sparse matrix elements (for initialization)
	void Add_Block_Triplet_Helper(const int i,const int j,std::vector<TripletT>& elements)
	{for(int ii=0;ii<3;ii++)for(int jj=0;jj<3;jj++)elements.push_back(TripletT(i*3+ii,j*3+jj,(double)0));}

	////Add block Ks to K_ij
	void Add_Block_Helper(SparseMatrixT& K,const int i,const int j,const Matrix3& Ks)
	{
		SparseFunc::Add_Block<3,Matrix3>(K,i,i,Ks);
		SparseFunc::Add_Block<3,Matrix3>(K,j,j,Ks);
		if(!Is_Boundary_Node(i)&&!Is_Boundary_Node(j)){
			SparseFunc::Add_Block<3,Matrix3>(K,i,j,-Ks);
			SparseFunc::Add_Block<3,Matrix3>(K,j,i,-Ks);}
	}

	////Set block values on a vector
	void Set_Block(VectorX& b,const int i,const Vector3& bi)
	{for(int ii=0;ii<3;ii++)b[i*3+ii]=bi[ii];}

	////Add block values to a vector
	void Add_Block(VectorX& b,const int i,const Vector3& bi)
	{for(int ii=0;ii<3;ii++)b[i*3+ii]+=bi[ii];}
};

#endif