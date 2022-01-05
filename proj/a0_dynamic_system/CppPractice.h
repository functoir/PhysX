//////////////////////////////////////////////////////////////////////////
//// Dartmouth Physical Computing Programming Assignment 0
//// Author: TODO: PUT YOUR NAME HERE
////////////////////////////////////////////////////////////////////////// 

#ifndef __CppPractice_h__
#define __CppPractice_h__
#include <vector>		//// for std::vector
#include <iostream>		//// for printing texts on screen
#include "Common.h"		//// for accessing Eigen
using namespace std;

namespace CppPractice{
//// Practice of using a c++ array (6 TODOs)
void Practice_0()
{
	std::cout<<">>>>>>>> Cpp practice 0 start <<<<<<<<"<<std::endl;

	////TODO 0: declare an int std::vector with the name array (already done for you).
	std::vector<int> array;

	//// TODO 1: Initialize the array with ten elements from 1-10. Print the array elements on screen.
	
	/*Your implementation here*/

	//// TODO 2: Add two new elements 11 and 12 to the end of the array. Print the array elements on screen.

	/*Your implementation here*/

	//// TODO 3: Add an new element -1 between the previous array[5] and array[6]. 
	//// Keep the other element values unchanged (i.e., the current array values should be {1,2,3,4,5,6,-1,6,7,8,...})
	//// Print the array elements on screen.

	/*Your implementation here*/

	//// TODO 4: Return the current size of the array. Print the array size on screen.

	/*Your implementation here*/

	//// TODO 5: Resize the array to 20 elements with each element array[i]=2*i. Print the array elements on screen.

	/*Your implementation here*/

	//// TODO 6: Clear the array and print the current array size on screen.

	/*Your implementation here*/

	std::cout<<">>>>>>>> Cpp practice 0 end <<<<<<<<"<<std::endl;
}

//// Practice of using Eigen vectors (10 TODOs)
void Practice_1()
{
	std::cout<<">>>>>>>> Cpp practice 1 start <<<<<<<<"<<std::endl;

	//// TODO 0: Declare a 2d vector with the name u and initial values [1,2] and print u on screen as "u=xxx" (already done for you).
	Vector2 u=Vector2(1,2);
	std::cout<<"u="<<u.transpose()<<std::endl;	//// Eigen vectors are column vectors by default.

	//// TODO 1: Change the x component of u to 3 and print u on screen as "u=xxx".
	
	/*Your implementation here*/

	//// TODO 2: Declare another 2d vector with the name v and initial values [2,3] and print v on screen as "v=xxx".

	/*Your implementation here*/

	//// TODO 3: Declare the third 2d vector with the name w and initial values as w=u+v and print w on screen as "w=xxx".

	/*Your implementation here*/

	//// TODO 4: Add u's x component to v's y component; and subtract u's y component from v's x component. 
	//// Print the current value of v on screen as "v=xxx".

	/*Your implementation here*/

	//// TODO 5: Calculate the dot product between u and v and print the result.

	/*Your implementation here*/

	//// TODO 6: Calculate the length of u and print the result.

	/*Your implementation here*/

	//// TODO 7: Normalize vector u and print the result.

	/*Your implementation here*/

	//// TODO8: Initialize an array (std::vector) array_1 of 3d vectors with three initial elements [1,0,0], [0,1,0], and [0,0,1].
	//// Print the values of the array on screen.

	/*Your implementation here*/

	//// TODO9: Initialize another array (std::vector) array_2 with their initial values as 
	//// 2 times the element in array_1 (i.e., the values should be [2,0,0],[0,2,0], and [0,0,2]. But you need to use the array_1 values to initialize array_2 instead of using these numbers directly.)
	//// Print the current values of array_2 on screen.

	/*Your implementation here*/

	//// TODO 10: Add the values of array_2 to array_1 and print the results of array_1 on screen (you are suppose to see [3,0,0],[0,3,0], and [0,0,3])

	/*Your implementation here*/

	std::cout<<">>>>>>>> Cpp practice 1 end <<<<<<<<"<<std::endl;
}
};
#endif