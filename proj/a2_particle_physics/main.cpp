//#####################################################################
// Main
// Dartmouth COSC 89.18/189.02: Computational Methods for Physical Systems, Assignment starter code
// Contact: Bo Zhu (bo.zhu@dartmouth.edu)
//#####################################################################
#include <iostream>
#include "ParticleSandDriver.h"
#include "ParticleFluidDriver.h"

#ifndef __Main_cpp__
#define __Main_cpp__

int main(int argc,char* argv[])
{
	//// change the mode id to switch between sand and fluid
	for (int i = 0; i < argc; i++) std::cout << "argv[" << i << "] = " << argv[i] << std::endl;
//	int mode=2;
    int mode = strtol(argv[2], nullptr, 10);
	switch(mode){
	case 1 : {
		ParticleSandDriver<2> driver;
		driver.Initialize();
		driver.Run();	
	} break;
	case 2:{
		ParticleFluidDriver<2> driver;
		driver.Initialize();
		driver.Run();		
	}break;
    
    default : {
        std::cout << "Please specify a mode id: 1 for sand, 2 for fluid" << std::endl;
    }
	}
}

#endif
