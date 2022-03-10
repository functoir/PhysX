//#####################################################################
// Main
// Dartmouth COSC 89.18/189.02: Computational Methods for Physical Systems, Assignment starter code
// Contact: Bo Zhu (bo.zhu@dartmouth.edu)
//#####################################################################
#include <iostream>
#include <cstdio>
#include "MassSpringInteractiveDriver.h"

#ifndef __Main_cpp__
#define __Main_cpp__

int main(int argc,char* argv[])
{
	////default arguments
	int test=1;
	int scale=1;

	//parse from command line
	for(int i=0;i<argc;i++){
        std::cout << "argv @ " << i << ": " << argv[i] << std::endl;
		if (strcmp(argv[i], "-test") == 0) {
			test = atoi(argv[i+1]);
            std::cout<<"test="<<test<<std::endl;
        }
		else if (strcmp(argv[i],"-scale") == 0) {
            scale = atoi(argv[++i]);
        }
	}
	std::cout<<"[Mass spring simulation driver arguments]: -test = "<<test<<", -scale = "<<scale<<std::endl;

    
    // custom test toggle
//    test=2;
	////initialize driver
	MassSpringInteractivDriver driver;
	driver.scale=scale;
	driver.test=test;
	driver.Initialize();
	driver.Run();	
}

#endif