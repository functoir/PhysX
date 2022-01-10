#include "Common.h"

void Test_Sparse_Matrix_Solver()
{
	SparseMatrix<double> A;
	VectorN<double> x;
	VectorN<double> b;
	A.resize(4,4);

	////Approach 1: initialize the matrix directly
	std::vector<Triplet<double> > A_elements;
	A_elements.push_back(Triplet<double>(0,0,2));
	A_elements.push_back(Triplet<double>(0,1,-1));
	A_elements.push_back(Triplet<double>(1,0,-1));
	A_elements.push_back(Triplet<double>(1,1,2));
	A_elements.push_back(Triplet<double>(1,2,-1));
	A_elements.push_back(Triplet<double>(2,1,-1));
	A_elements.push_back(Triplet<double>(2,2,2));
	A_elements.push_back(Triplet<double>(2,3,-1));
	A_elements.push_back(Triplet<double>(3,2,-1));
	A_elements.push_back(Triplet<double>(3,3,1));
	A.setFromTriplets(A_elements.begin(),A_elements.end());
	A.makeCompressed();
	std::cout<<"A:\n"<<A<<std::endl;

	////Approach 2: allocate memory first and then assign values
	A_elements.push_back(Triplet<double>(0,0,0));
	A_elements.push_back(Triplet<double>(0,1,0));
	A_elements.push_back(Triplet<double>(1,0,0));
	A_elements.push_back(Triplet<double>(1,1,0));
	A_elements.push_back(Triplet<double>(1,2,0));
	A_elements.push_back(Triplet<double>(2,1,0));
	A_elements.push_back(Triplet<double>(2,2,0));
	A_elements.push_back(Triplet<double>(2,3,0));
	A_elements.push_back(Triplet<double>(3,2,0));
	A_elements.push_back(Triplet<double>(3,3,0));
	A.setFromTriplets(A_elements.begin(),A_elements.end());
	A.makeCompressed();

	A.coeffRef(0,0)=2;
	A.coeffRef(0,1)=-1;
	A.coeffRef(1,0)=-1;
	A.coeffRef(1,1)=2;
	A.coeffRef(1,2)=-1;
	A.coeffRef(2,1)=-1;
	A.coeffRef(2,2)=2;
	A.coeffRef(2,3)=-1;
	A.coeffRef(3,2)=-1;
	A.coeffRef(3,3)=1;
	std::cout<<"A:\n"<<A<<std::endl;

	////Approach 3: initialize the matrix indirectly
	SparseMatrix<double> L;
	L.resize(4,4);
	std::vector<Triplet<double> > L_elements;
	L_elements.push_back(Triplet<double>(0,0,1));
	L_elements.push_back(Triplet<double>(1,0,-1));
	L_elements.push_back(Triplet<double>(1,1,1));
	L_elements.push_back(Triplet<double>(2,1,-1));
	L_elements.push_back(Triplet<double>(2,2,1));
	L_elements.push_back(Triplet<double>(3,2,-1));
	L_elements.push_back(Triplet<double>(3,3,1));
	L.setFromTriplets(L_elements.begin(),L_elements.end());
	L.makeCompressed();

	SparseMatrix<double> M;
	M.resize(4,4);
	std::vector<Triplet<double> > M_elements;
	M_elements.push_back(Triplet<double>(0,0,1));
	M_elements.push_back(Triplet<double>(1,1,1));
	M_elements.push_back(Triplet<double>(2,2,1));
	M_elements.push_back(Triplet<double>(3,3,1));
	M.setFromTriplets(M_elements.begin(),M_elements.end());
	M.makeCompressed();

	A=L.transpose()*M*L;
	std::cout<<"A:\n"<<A<<std::endl;

	////traverse the non-zero elements
	for(int i=0;i<A.rows();i++){
		for(InnerIterator<double> iter(A,i);iter;++iter){
			int j=(int)iter.col();
			double A_ij=iter.value();
			std::cout<<"nz ele: "<<i<<", "<<j<<": "<<A_ij<<std::endl;
		}
	}

	////Solve Ax=b
	x.resize(4);
	x.fill((double)0);

	b.resize(4);
	b.fill((double)0);

	b[0]=1;
	b[1]=2;
	b[2]=3;
	b[3]=4;

	SparseSolver::CG(A,x,b);
	std::cout<<"x:\n"<<x<<std::endl;
}

int main()
{
	Test_Sparse_Matrix_Solver();
}
