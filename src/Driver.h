//#####################################################################
// Basic simulation driver
// Dartmouth COSC 89.18/189.02: Computational Methods for Physical Systems, Assignment starter code
// Contact: Bo Zhu (bo.zhu@dartmouth.edu)
//#####################################################################
#ifndef __Driver_h__
#define __Driver_h__
#include "Common.h"

class Driver
{
public:
	int test=1;
	std::string output_dir="output";
	std::string frame_dir;
	int first_frame=0,last_frame=200,current_frame=0;
	double frame_rate=50;
	double time=(double)0,current_time=(double)0;
	int scale=1;
	double time_step=(double)1;
	bool verbose=true;

	double Time_At_Frame(const int frame){return (double)frame/frame_rate;}
	int Frame_At_Time(const double time){return (int)((double)time*frame_rate);}
	virtual double Timestep(){return time_step;}

	virtual void Initialize(){}

	virtual void Run()
	{
		while(current_frame<last_frame){
			current_frame++;
			Advance_To_Target_Time(Time_At_Frame(current_frame));}
	}

	virtual void Advance_To_Target_Time(const double target_time)
	{
		bool done=false;
		for(int substep=1;!done;substep++){
			double dt=Timestep();
			if(time+dt>=target_time){dt=target_time-time;done=true;}
			else if(time+2*dt>=target_time){dt=(double).5*(target_time-time);}
			Advance_One_Time_Step(dt,time);
			time+=dt;
        }
	}

	virtual void Advance_One_Time_Step(const double dt,const double time)
	{
		if(verbose)std::cout<<"Advance one time step by dt="<<dt<<" to time "<<time<<std::endl;
	}
};
#endif
