#ifndef __InClassDemoWeekTwoDriver_h__
#define __InClassDemoWeekTwoDriver_h__
#include "Common.h"
#include "InClassDemoDriver.h"

class InClassDemoWeekTwoDriver : public InClassDemoDriver
{using VectorD=Vector3;using VectorDi=Vector2i;using Base=Driver;
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
			vertices.push_back(VectorD(x,y,(real)0));}
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
	}

	////update simulation data to its visualization counterparts
	virtual void Sync_Simulation_And_Visualization_Data()
	{
		point.Sync_Data(pos);
	}
};
#endif