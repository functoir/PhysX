#ifndef __InClassDemoWeekFourDriver_h__
#define __InClassDemoWeekFourDriver_h__
#include "Common.h"
#include "InClassDemoDriver.h"

void Test_Sparse_Matrix_And_Solver()
{
	SparseMatrix<real> A;
	Array<Triplet<real> > elements;
	elements.push_back(Triplet<real>(0,0,2));
	elements.push_back(Triplet<real>(0,1,-1));
	elements.push_back(Triplet<real>(1,0,-1));
	elements.push_back(Triplet<real>(1,1,2));
	elements.push_back(Triplet<real>(2,1,-1));
	elements.push_back(Triplet<real>(2,2,2));
	elements.push_back(Triplet<real>(2,3,-1));
	elements.push_back(Triplet<real>(3,3,2));
	elements.push_back(Triplet<real>(1,2,-1));
	elements.push_back(Triplet<real>(3,2,-1));
	elements.push_back(Triplet<real>(0,2,0));
	elements.push_back(Triplet<real>(2,0,0));

	A.resize(4,4);
	A.setFromTriplets(elements.begin(),elements.end());
	A.makeCompressed();

	VectorX x(4);
	VectorX b(4);
	b[0]=1.;
	b[1]=0.;
	b[2]=0.;
	b[3]=1.;
	
	SparseSolver::CG(A,x,b);

	std::cout<<"\nA:\n"<<A<<std::endl;
	std::cout<<"\nx:\n"<<x<<std::endl;
	std::cout<<"\nb:\n"<<b<<std::endl;
}


#endif