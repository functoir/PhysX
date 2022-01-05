//#####################################################################
// Main
// Dartmouth COSC 89.18/189.02: Computational Methods for Physical Systems, Assignment starter code
// Contact: Bo Zhu (bo.zhu@dartmouth.edu)
//#####################################################################
#include <iostream>
#include "CppPractice.h"
#include "DynamicSystemDriver.h"

#ifndef __Main_cpp__
#define __Main_cpp__

int main(int argc,char* argv[])
{
	//// HW0 Part 1: cpp practice. Implement the function TODOs in CppPractice.h
	CppPractice::Practice_0();
	CppPractice::Practice_1();

	//// HW0 Part 2: particle system. Uncomment the following code to initialize the opengl window.
	DynamicSystemDriver driver;
	driver.Initialize();
	driver.Run();	
}

#endif