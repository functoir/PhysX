//#####################################################################
// Main
// Dartmouth COSC 89.18/189.02: Computational Methods for Physical Systems, Assignment starter code
// Contact: Bo Zhu (bo.zhu@dartmouth.edu)
//#####################################################################
#include <iostream>
#include "DynamicSystemDriver.h"
#include "InClassDemoDriver.h"
#include "InClassDemoWeekTwoDriver.h"

#ifndef __Main_cpp__
#define __Main_cpp__

int main(int argc,char* argv[])
{
	int driver=1;

	switch(driver){
	case 1:{
		DynamicSystemDriver driver;
		driver.Initialize();
		driver.Run();	
	}break;
	case 2:{
		InClassDemoWeekTwoDriver driver;
		driver.Initialize();
		driver.Run();
	}
	}
}

#endif