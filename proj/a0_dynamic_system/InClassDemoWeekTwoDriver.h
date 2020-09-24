#ifndef __InClassDemoWeekTwoDriver_h__
#define __InClassDemoWeekTwoDriver_h__
#include "Common.h"
#include "InClassDemoDriver.h"

class InClassDemoSimpleDynamicsDriver : public InClassDemoDriver
{using Base=Driver;
	////simulation data
	real dt=.02;
	Vector3 vel;	////velocity of point
	Vector3 pos;	////position of point

	////visualization data
	Curve curve;
	Point point;
public:

	////initialize simulation data and its visualizations
	virtual void Initialize_Data()
	{
		////initialize curve
		real r=(real)1;
		int n=32;
		real theta_0=2.*3.1415927/(real)n;
		Array<Vector3> vertices;
		for(int i=0;i<=n;i++){
			real theta=(real)i*theta_0;
			real x=r*cos(theta);
			real y=r*sin(theta);
			vertices.push_back(Vector3(x,y,(real)0));}

		curve.Initialize(this);
		curve.Sync_Data(vertices);

		////initialize point
		pos=vertices[0];

		point.Initialize(this);
		point.Sync_Data(pos);
	}

	////advance simulation timesteps
	virtual void Advance(const real dt)
	{
		////manipulate vel and pos
		vel=Vector3::Unit(0);
		pos+=vel*dt;
	}

	////update simulation data to its visualization counterparts
	virtual void Sync_Simulation_And_Visualization_Data()
	{
		point.Sync_Data(pos);
	}
};

class InClassDemoMassSpringDriver : public InClassDemoDriver
{using Base=Driver;
	////simulation data
	real dt=.02;
	Vector3 vel[2];	////velocity of point
	Vector3 pos[2];	////position of point

	////visualization data
	Curve curve;
	Point points[2];
public:

	////initialize simulation data and its visualizations
	virtual void Initialize_Data()
	{
		vel[0]=-Vector3(1.,0.,0.);
		vel[1]=Vector3(1.,0.,0.);
		pos[0]=Vector3(1.,0.,0.);
		pos[1]=Vector3(2.,0.,0.);

		curve.Initialize(this);
		curve.Sync_Data(pos,2);

		for(int i=0;i<2;i++){
			points[i].Initialize(this);
			points[i].Sync_Data(pos[i]);}
	}

	////advance simulation timesteps
	virtual void Advance(const real dt)
	{
		////TODO

		////time integration
		for(int i=0;i<2;i++){
			pos[i]+=vel[i]*dt;}
	}

	////update simulation data to its visualization counterparts
	virtual void Sync_Simulation_And_Visualization_Data()
	{
		for(int i=0;i<2;i++)
			points[i].Sync_Data(pos[i]);
		curve.Sync_Data(pos,2);
	}
};
#endif