//#####################################################################
// Main
// Dartmouth COSC 89.18/189.02: Computational Methods for Physical Systems, Assignment starter code
// Contact: Bo Zhu (bo.zhu@dartmouth.edu)
//#####################################################################
#include <iostream>
#include "SoftBodyDriver.h"
#include "ParticleSandDriver.h"

#ifndef __Main_cpp__
#define __Main_cpp__

int main(int argc,char* argv[]) {
    ////default arguments
    int test = 1;
    int scale = 3;
    
    //parse from command line
    for (int i = 0; i < argc; i++) {
        std::cout << "argv @ " << i << ": " << argv[i] << std::endl;
        if (strcmp(argv[i], "-test") == 0) {
            test = atoi(argv[i + 1]);
            std::cout << "test=" << test << std::endl;
        } else if (strcmp(argv[i], "-scale") == 0) {
            scale = atoi(argv[++i]);
        }
    }
    std::cout << "[Mass spring simulation driver arguments]: -test = " << test << ", -scale = " << scale << std::endl;
    
    ////initialize driver
    
    if (test < 7) {
        MassSpringInteractivDriver driver;
        driver.scale = scale;
        driver.test = test;
        driver.Initialize();
        driver.Run();
    } else if (test == 7) {
        ParticleSandDriver<3> driver;
//        driver.test = test;
//        driver.scale = scale;
        
        std::cout << "ParticleSandDriver<3> driver.Initialize()" << std::endl;
        driver.Initialize();
        
        std::cout << "ParticleSandDriver<3> driver.Run()" << std::endl;
        driver.Run();
    }
    
    return 0;
}


#endif