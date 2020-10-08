#ifndef __InClassCompetitionWeekFourDriver_h__
#define __InClassCompetitionWeekFourDriver_h__

#include "Common.h"
#include "InClassDemoDriver.h"

class InClassCompetitionWeekFourDriver : public InClassDemoDriver
{using Base=Driver;
	real dt=.02;

	////particles
	////You may put any number of particles here, with a minimum number of 4
	static const int p_num=4;	////particle number
	Vector2 vel[p_num];			////velocity of point
	Vector2 pos[p_num];			////position of point
	real mass[p_num];			////mass
	Vector2 force[p_num];		////force

	////springs
	Array<Vector2i> springs;	////spring particle indices
	Array<real> rest_length;	////spring rest length 

	Vector2 g=Vector2(0.,-1.);	////gravity

	////visualization data
	Curve left_ground;
	Curve right_ground;
	Segments segments;
	Point points[p_num];

public:
	////initialize simulation data and its visualizations
	virtual void Initialize_Data()
	{
		////particle position
		pos[0]=Vector2(-1.,0.);
		pos[1]=Vector2(0.,0.);
		pos[2]=Vector2(1.,0.);
		pos[3]=Vector2(0.,1.);

		////particle mass
		mass[0]=1.;
		mass[1]=10.;
		mass[2]=1.;
		mass[3]=1.;

		////Are two springs enough?
		springs={{0,1},{1,2}};

		//////////////////////////////////////////////////////////////////////////
		////visualization of particles and springs, you don't need to modify this part
		for(int i=0;i<p_num;i++){
			points[i].Initialize(this);
			points[i].Sync_Data(pos[i]);}
		points[1].Set_Color(1.,0.,0.);
		points[1].Set_Radius(2.);

		segments.Initialize(this);
		segments.Sync_Data(pos,p_num,springs);
		segments.Set_Color(0.,0.,0.);
		segments.Set_Color(.2,.2,.2);

		////visualization for the grounds
		Array<Vector2> left_ground_vertices={{-2.,-.2},{-.8,-.2},{-.8,-1.}};
		left_ground.Initialize(this);
		left_ground.Sync_Data(left_ground_vertices);
		left_ground.Set_Linewidth(2.);
		left_ground.Set_Color(0.,0.,0.);

		Array<Vector2> right_ground_vertices={{2.,-.2},{.8,-.2},{.8,-1.}};
		right_ground.Initialize(this);
		right_ground.Sync_Data(right_ground_vertices);
		right_ground.Set_Linewidth(2.);
		right_ground.Set_Color(0.,0.,0.);
	}

	////calculate spring force for each particle
	virtual void Mass_Spring_Simulation(const real dt)
	{
		for(int i=0;i<p_num;i++){
			force[i]=Vector2::Zero();}

		////hey, it seems I missed my spring code, want to write some simulation code here?
	}

	////advance simulation timesteps
	virtual void Advance(const real dt)
	{	
		for(int i=0;i<p_num;i++){
			vel[i]+=g*dt;}

		Mass_Spring_Simulation(dt);

		//////////////////////////////////////////////////////////////////////////
		////do not change the following piece of code
		////time integration
		for(int i=0;i<p_num;i++){
			vel[i]+=g*dt;					////gravity
			vel[i]+=force[i]/mass[i]*dt;	////spring force
			pos[i]+=vel[i]*dt;}

		////simple collision detection
		for(int i=0;i<p_num;i++){
			real x=pos[i][0];real y=pos[i][1];
			if(y<-.2&&(x<-.8||x>.8)){
				pos[i][1]=-.2;
				vel[i][1]=0.;}
		}
	}

	////update simulation data to its visualization counterparts
	virtual void Sync_Simulation_And_Visualization_Data()
	{
		for(int i=0;i<p_num;i++)
			points[i].Sync_Data(pos[i]);

		segments.Sync_Data(pos,p_num,springs);
	}
};
#endif