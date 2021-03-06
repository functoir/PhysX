//////////////////////////////////////////////////////////////////////////
//// Dartmouth Physical Computing In-Class Demo: Dynamic System
////////////////////////////////////////////////////////////////////////// 

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
	double dt=.05;								//// time step
	Vector3 g=Vector3(0.,-9.8,0.);				//// gravity
	
	//// segment data structure (for visualization purpose only)
	std::vector<std::vector<Vector3> > segment_mesh;			//// array of segment meshes (update this array in Advance Simulation for segment motion if necessary)
	std::vector<Vector3> segment_colors;						//// specify a segment color for each segment_mesh (not each segment)

	//// TODO: initialize your particle system by initializing the values of particle position, velocity, color, radii, and number
	//// Attention: make sure to set the value of particle_number!
	void Initialize_Simulation()
	{
		//Initialize_Ballistic_Motion();
		//Initialize_Two_Body();
		Initialize_Mass_Spring();
	}

	//// TODO: advance your particle system by updating the particle position and velocity values
	void Advance_Simulation(const double dt,const double time)
	{
		//Advance_Ballistic_Motion(dt,time);
		//Advance_Two_Body(dt,time);
		Advance_Mass_Spring(dt,time);
	}

	void Initialize_Ballistic_Motion()
	{
		// Example: ballistic motion
		{
			position.push_back(Vector3(0.,0.,0.));
			velocity.push_back(Vector3(10.,0.,0.));
			color.push_back(Vector3(0.,1.,0.));
			radii.push_back(.1);		

			std::vector<Vector3> segment_vertices;
			segment_vertices.push_back(position[0]);
			segment_vertices.push_back(position[0]);
			segment_mesh.push_back(segment_vertices);
			segment_colors.push_back(Vector3(1.,0.,0.));
		}

		//// draw a paraloba: the analytical solution
		{
			double u=velocity[0][0];
			std::vector<Vector3> segment_vertices;
			int n=5000;
			for(int i=0;i<n;i++){
				double t=(double)i*.01;
				double x=u*t;
				double y=.5*g[1]*t*t;
				Vector3 pos(x,y,0.);

				segment_vertices.push_back(pos);
				if(i>0&&i<n-1)segment_vertices.push_back(pos);
			}
			segment_mesh.push_back(segment_vertices);
			segment_colors.push_back(Vector3(1.,1.,1.));
		}

		////// visualizing the ceiling wall (this was the illustration code used in the lecture)
		//{
		//	double y=-1.;
		//	std::vector<Vector3> segment_vertices;
		//	segment_vertices.push_back(Vector3(-1.,y,0.));				//// first vertex of segment 1
		//	segment_vertices.push_back(Vector3(1.,y,0.));				//// second vertex of segment 1
		//	
		//	int n=8;
		//	double step_size=2./(double)n;
		//	Vector3 start=Vector3(-1.,y,0.);
		//	for(int i=0;i<=n;i++){
		//		Vector3 point1=start+(double)i*step_size*Vector3(1.,0.,0.);
		//		Vector3 point2=point1+Vector3(-0.2,-0.2,0.);
		//		segment_vertices.push_back(point1);
		//		segment_vertices.push_back(point2);
		//	}

		//	segment_mesh.push_back(segment_vertices);
		//	segment_colors.push_back(Vector3(1.,0.,0.));		
		//}	
	}

	void Advance_Ballistic_Motion(const double dt,const double time)
	{
		//// Example 1: ballistic motion
		Vector3 u0=velocity[0];
		Vector3 u1=velocity[0]+g*dt;
		position[0]+=.5*(u0+u1)*dt;
		velocity[0]+=g*dt;

		//// sync segments
		{
			segment_mesh[0].push_back(segment_mesh[0][segment_mesh[0].size()-1]);
			segment_mesh[0].push_back(position[0]);
		}

		//// collision detection with the ground
		//double y=-1.;
		//if(position[0][1]<y){
		//	velocity[0][1]*=-1.;
		//}	
	}

	void Initialize_Two_Body()
	{
		// Example: Two-body
		//// initialize two spheres by default (feel free to change them if you want)
		//// sphere 1
		{
			position.push_back(Vector3(0.,1.,0.));
			velocity.push_back(Vector3(0.,0.,0.));
			color.push_back(Vector3(0.,1.,0.));
			radii.push_back(.1);		
		}

		//// sphere 2
		{
			position.push_back(Vector3(0.,0.,0.));
			velocity.push_back(Vector3(0.,0.,0.));
			color.push_back(Vector3(1.,0.,0.));
			radii.push_back(.1);		
		}

		//// initialize segments (for visualization purpose only)
		//// visualizing the rod connecting the two spheres
		{
			std::vector<Vector3> segment_vertices;
			segment_vertices.push_back(Vector3(0.,1.,0.));				//// first vertex of segment 0
			segment_vertices.push_back(Vector3(0.,1.,0.));				//// second vertex of segment 0
		
			segment_mesh.push_back(segment_vertices);
			segment_colors.push_back(Vector3(0.,1.,1.));		
		}

		{
			std::vector<Vector3> segment_vertices;
			segment_vertices.push_back(Vector3(0.,0.,0.));				//// first vertex of segment 0
			segment_vertices.push_back(Vector3(0.,0.,0.));				//// second vertex of segment 0
		
			segment_mesh.push_back(segment_vertices);
			segment_colors.push_back(Vector3(1.,0.,1.));		
		}		
	}

	void Advance_Two_Body(const double dt,const double time)
	{
		// Example 2: two body
		{
			double m0=2;
			double m1=1;
			double G=2;
			Vector3 dis=(position[1]-position[0]);
			double r=dis.norm();
			Vector3 dir=dis/r;
			Vector3 force=G*m0*m1*dir/pow(r,2);
			velocity[0]+=force/m0*dt;
			velocity[1]+=-force/m1*dt;
			position[0]+=velocity[0]*dt;
			position[1]+=velocity[1]*dt;		
		}

		//// synchronize particle positions to segment vertices
		int max_seg_num=64;
		for(int i=0;i<2;i++){
			if(segment_mesh[i].size()>max_seg_num*2){
				segment_mesh[i].erase(segment_mesh[i].begin());
				segment_mesh[i].erase(segment_mesh[i].begin());
			}

			segment_mesh[i].push_back(segment_mesh[i][segment_mesh[i].size()-1]);
			segment_mesh[i].push_back(position[i]);
		}
	}

	void Initialize_Mass_Spring()
	{
		//// particle 1
		position.push_back(Vector3(0.,1.,0.));
		velocity.push_back(Vector3(0.,0.,0.));
		color.push_back(Vector3(0.,1.,0.));
		radii.push_back(.2);		

		//// sphere 2
		position.push_back(Vector3(0.,0.,0.));
		velocity.push_back(Vector3(0.,0.,0.));
		color.push_back(Vector3(1.,0.,0.));
		radii.push_back(.2);		

		//// spring 1
		std::vector<Vector3> segment_vertices;
		segment_vertices.push_back(Vector3(0.,1.,0.));				//// first vertex of segment 0
		segment_vertices.push_back(Vector3(0.,0.,0.));				//// second vertex of segment 0
		
		segment_mesh.push_back(segment_vertices);
		segment_colors.push_back(Vector3(1.,1.,1.));		
	}

	void Advance_Mass_Spring(const double dt,const double time)
	{
		double mass=1.;
		double ks=10.;
		double rest_length=2.;

		double distance=(position[1]-position[0]).norm();
		Vector3 direction=(position[1]-position[0])/distance;
		Vector3 force=ks*(distance-rest_length)*direction;
		
		Vector3 vel=velocity[1]-velocity[0];
		double proj=vel.dot(direction);
		double kd=10.;
		Vector3 damping=kd*proj*direction;
		force+=damping;

		velocity[0]+=force*dt/mass;
		velocity[1]-=force*dt/mass;
		position[0]+=velocity[0]*dt;
		position[1]+=velocity[1]*dt;

		//// synchronize particle positions to segment vertices
		segment_mesh[0][0]=position[0];
		segment_mesh[0][1]=position[1];
	}

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	//// visualization-related functions (no need to modify)

	virtual void Initialize(){OpenGLViewer::Initialize();}
	virtual void Run(){OpenGLViewer::Run();}

	//// visualization data
	std::vector<OpenGLSphere*> opengl_spheres;											////spheres
	std::vector<OpenGLSegmentMesh*> opengl_segments;									////segments

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
		int particle_number=(int)position.size();
		opengl_spheres.resize(particle_number);
		double default_radius=0.1;
		Vector3 default_color=Vector3(0.,1.,1.);
		for(int i=0;i<particle_number;i++){
			OpenGLSphere* sphere=Add_Interactive_Object<OpenGLSphere>();
			sphere->pos=position[i];
			if(i<radii.size())sphere->radius=radii[i];
			else sphere->radius=default_radius;
			if(i<color.size()) Set_Color(sphere,OpenGLColor((float)color[i][0],(float)color[i][1],(float)color[i][2],1.));
			else Set_Color(sphere,OpenGLColor((float)default_color[0],(float)default_color[1],(float)default_color[2],1.));
			sphere->Set_Data_Refreshed();
			sphere->Initialize();
			opengl_spheres[i]=sphere;		
		}	

		if(segment_mesh.size()>0){
			for(int s=0;s<segment_mesh.size();s++){
				int segment_number=(int)segment_mesh[s].size()/2;
				std::vector<Vector3>& segment_vertices=segment_mesh[s];
				Vector3 segment_color=Vector3(1.,1.,0.1);
				if(s<segment_color.size())segment_color=segment_colors[s];
				OpenGLSegmentMesh* opengl_segment_mesh=Add_Interactive_Object<OpenGLSegmentMesh>();
				opengl_segments.push_back(opengl_segment_mesh);
				opengl_segment_mesh->mesh.Elements().resize(segment_number);
				opengl_segment_mesh->mesh.Vertices().resize(segment_number*2);

				for(int i=0;i<segment_number;i++){
					opengl_segment_mesh->mesh.Vertices()[i*2]=segment_vertices[i*2];
					opengl_segment_mesh->mesh.Vertices()[i*2+1]=segment_vertices[i*2+1];
					opengl_segment_mesh->mesh.Elements()[i]=Vector2i(i*2,i*2+1);	
				}		

				Set_Color(opengl_segment_mesh,OpenGLColor((float)segment_color[0],(float)segment_color[1],(float)segment_color[2],1.));
				Set_Line_Width(opengl_segment_mesh,1.f);
				opengl_segment_mesh->Set_Data_Refreshed();
				opengl_segment_mesh->Initialize();				
			}
		}
	}

	//// this function synchronizes the simulation data from position and segment_vertices to the opengl viewer
	void Sync_Simulation_And_Visualization_Data()
	{
		int particle_number=(int)position.size();

		////update and sync data for spheres
		for(int i=0;i<particle_number;i++){
			opengl_spheres[i]->pos=position[i];
			opengl_spheres[i]->Set_Data_Refreshed();
		}

		if(segment_mesh.size()>0){
			for(int s=0;s<segment_mesh.size();s++){
				int vtx_num=(int)segment_mesh[s].size();
				int segment_number=(int)segment_mesh[s].size()/2;
				std::vector<Vector3>& segment_vertices=segment_mesh[s];
				OpenGLSegmentMesh* opengl_segment_mesh=opengl_segments[s];
				if(opengl_segment_mesh->mesh.Vertices().size()<vtx_num){
					opengl_segment_mesh->mesh.Vertices().resize(vtx_num);
				}
				if(opengl_segment_mesh->mesh.Elements().size()<vtx_num/2){
					opengl_segment_mesh->mesh.Elements().resize(vtx_num/2);
					for(int i=0;i<vtx_num/2;i++){
						opengl_segment_mesh->mesh.Elements()[i]=Vector2i(2*i,2*i+1);
					}
				}
				for(int i=0;i<segment_number;i++){
					opengl_segment_mesh->mesh.Vertices()[i*2]=segment_vertices[i*2];
					opengl_segment_mesh->mesh.Vertices()[i*2+1]=segment_vertices[i*2+1];
					opengl_segment_mesh->Set_Data_Refreshed();		
				}	
			}
		}
	}

	//// update simulation and visualization for each time step
	virtual void Toggle_Next_Frame()
	{
		static double time=0.;
		time+=dt;
		Advance_Simulation(dt,time);
		Sync_Simulation_And_Visualization_Data();

		OpenGLViewer::Toggle_Next_Frame();
	}
};
#endif