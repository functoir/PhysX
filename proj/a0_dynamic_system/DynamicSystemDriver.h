#ifndef __DynamicSystemDriver_h__
#define __DynamicSystemDriver_h__
#include <random>
#include "Common.h"
#include "Driver.h"
#include "OpenGLMesh.h"
#include "OpenGLCommon.h"
#include "OpenGLWindow.h"
#include "OpenGLViewer.h"
#include "OpenGLMarkerObjects.h"
#include "OpenGLParticles.h"

class DynamicSystemDriver : public Driver, public OpenGLViewer
{
public:
	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//// simulation-related functions

	////simulation data structures
	std::vector<Vector3> position;				//// array of particle positions			
	std::vector<Vector3> velocity;				//// array of particle velocities
	std::vector<Vector3> color;					//// array of particle colors
	std::vector<double> radii;					//// array of particle radii
	int particle_number=0;						//// number of particles
	double dt=.02;								//// time step
	bool display_particle_trace=true;			//// record and display each particle's trace if true

	//// TODO: initialize your particle system by initializing the values of particle position, velocity, color, radii, and number
	//// Attention: make sure to set the value of particle_number!
	void Initialize_Simulation()
	{
		/*Your implementation here*/
		//// initialize a static, red sphere in the origin by default
		position.push_back(Vector3(0.,0.,0.));
		velocity.push_back(Vector3(0.,0.,0.));
		color.push_back(Vector3(1.,0.,0.));
		radii.push_back(.2);
		particle_number=1;
	}

	//// TODO: advance your particle system by updating the particle position and velocity values
	void Advance_Simulation(const double dt)
	{
		/*Your implementation here */
	}

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//// visualization-related functions (no need to modify)

	virtual void Initialize(){OpenGLViewer::Initialize();}
	virtual void Run(){OpenGLViewer::Run();}

	//// visualization data
	std::vector<OpenGLSegmentMesh*> opengl_trace;						////vector field
	std::vector<OpenGLSphere*> opengl_spheres;							////spheres

	//// synchronize simulation data to visualization data, called in OpenGLViewer::Initialize()
	virtual void Initialize_Data()
	{
		//// initialize simulation data
		Initialize_Simulation();

		//// initialize visualization data 
		Initialize_Visualization();

		////set OpenGL rendering environments
		auto dir_light=OpenGLUbos::Add_Directional_Light(glm::vec3(-1.f,-.1f,-.2f));
		OpenGLUbos::Set_Ambient(glm::vec4(.1f,.1f,.1f,1.f));
		OpenGLUbos::Update_Lights_Ubo();
	}

	void Initialize_Visualization()
	{
		opengl_spheres.resize(particle_number);
		for(int i=0;i<particle_number;i++){
			OpenGLSphere* sphere=Add_Interactive_Object<OpenGLSphere>();
			sphere->pos=position[i];
			sphere->radius=radii[i];
			Set_Color(sphere,OpenGLColor((float)color[i][0],(float)color[i][1],(float)color[i][2],1.));
			sphere->Set_Data_Refreshed();
			sphere->Initialize();
			opengl_spheres[i]=sphere;		
		}	

		if(display_particle_trace){
			opengl_trace.resize(particle_number);
			for(int i=0;i<particle_number;i++){
				OpenGLSegmentMesh* trace=Add_Interactive_Object<OpenGLSegmentMesh>();
				trace->mesh.Elements().resize(1);
				trace->mesh.Vertices().resize(1);
				trace->mesh.Vertices()[0]=position[i];
				trace->mesh.elements[0]=Vector2i(0,0);
				Set_Color(trace,OpenGLColor((float)color[i][0],(float)color[i][1],(float)color[i][2],1.));
				trace->Set_Data_Refreshed();
				trace->Initialize();		
				opengl_trace[i]=trace;
			}		
		}
	}

	void Sync_Simulation_And_Visualization_Data()
	{
		////update and sync data for spheres
		for(int i=0;i<particle_number;i++){
			opengl_spheres[i]->pos=position[i];
			opengl_spheres[i]->Set_Data_Refreshed();
		}

		if(display_particle_trace){
			for(int i=0;i<particle_number;i++){
				opengl_trace[i]->mesh.Vertices().push_back(position[i]);
				int n=(int)opengl_trace[i]->mesh.Elements().size();
				opengl_trace[i]->mesh.Elements().push_back(Vector2i(n-1,n-2));
				opengl_trace[i]->Set_Data_Refreshed();		
			}		
		}
	}

	//// update simulation and visualization for each time step
	virtual void Toggle_Next_Frame()
	{
		Advance_Simulation(dt);
		Sync_Simulation_And_Visualization_Data();
		OpenGLViewer::Toggle_Next_Frame();
	}
};
#endif