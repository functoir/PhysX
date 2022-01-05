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

	//// particle data structures
	std::vector<Vector3> position;				//// array of particle positions (update this array in Advance Simulation for particle motion)			
	std::vector<Vector3> velocity;				//// array of particle velocities
	std::vector<Vector3> color;					//// array of particle colors 
	std::vector<double> radii;					//// array of particle radii
	int particle_number=0;						//// number of particles
	double dt=.02;								//// time step
	
	/*TODO: add your own additional data structures for particles if necessary*/

	//// segment data structure
	std::vector<Vector3> segment_vertices;		//// array of segment vertices (update this array in Advance Simulation for segment motion if necessary)
												//// the vertices are stored as [s0_0,s0_1,s1_0,s1_1,s2_0,s2_1,...], with si_0 and si_1 specified as the first and second vertex of segment si
												//// Attention: you need to update the values in segment_vertices if you want to move the segments in each timestep
	int segment_number=0;						//// number of segments
	Vector3 segment_color;

	//// TODO: initialize your particle system by initializing the values of particle position, velocity, color, radii, and number
	//// Attention: make sure to set the value of particle_number!
	void Initialize_Simulation()
	{
		/*Your implementation here*/
		//// initialize two spheres by default (feel free to change them if you want)
		//// sphere 1
		position.push_back(Vector3(0.,1.,0.));
		velocity.push_back(Vector3(0.,0.,0.));
		color.push_back(Vector3(0.,1.,0.));
		radii.push_back(.2);

		//// sphere 2
		position.push_back(Vector3(0.,0.,0.));
		velocity.push_back(Vector3(0.,0.,0.));
		color.push_back(Vector3(1.,0.,0.));
		radii.push_back(.2);

		particle_number=position.size();

		//// initialize one segment
		segment_vertices.push_back(Vector3(0.,1.,0.));				//// first vertex of segment 0
		segment_vertices.push_back(Vector3(0.,0.,0.));				//// second vertex of segment 0
		segment_color=Vector3(1.,1.,1.);
		segment_number=segment_vertices.size()/2;
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
	std::vector<OpenGLSphere*> opengl_spheres;							////spheres
	OpenGLSegmentMesh* opengl_segments;									////segments

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

		if(segment_number>0){
			opengl_segments=Add_Interactive_Object<OpenGLSegmentMesh>();
			opengl_segments->mesh.Elements().resize(segment_number);
			opengl_segments->mesh.Vertices().resize(segment_number*2);

			for(int i=0;i<segment_number;i++){
				opengl_segments->mesh.Vertices()[i*2]=segment_vertices[i*2];
				opengl_segments->mesh.Vertices()[i*2+1]=segment_vertices[i*2+1];
				opengl_segments->mesh.Elements()[i]=Vector2i(i*2,i*2+1);	
			}		

			Set_Color(opengl_segments,OpenGLColor((float)segment_color[0],(float)segment_color[1],(float)segment_color[2],1.));
			opengl_segments->Set_Data_Refreshed();
			opengl_segments->Initialize();	
		}
	}

	//// this function synchronize the simulation data from position and segment_vertices to the opengl viewer
	void Sync_Simulation_And_Visualization_Data()
	{
		////update and sync data for spheres
		for(int i=0;i<particle_number;i++){
			opengl_spheres[i]->pos=position[i];
			opengl_spheres[i]->Set_Data_Refreshed();
		}

		if(segment_number>0){
			for(int i=0;i<segment_number;i++){
				opengl_segments->mesh.Vertices()[i*2]=segment_vertices[i*2];
				opengl_segments->mesh.Vertices()[i*2+1]=segment_vertices[i*2+1];
				opengl_segments->Set_Data_Refreshed();		
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