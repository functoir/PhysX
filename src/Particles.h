//////////////////////////////////////////////////////////////////////////
////Dartmouth Physical Computing Starter Code
////http://www.dartmouth.edu/~boolzhu/cosc89.18.html
//////////////////////////////////////////////////////////////////////////

#ifndef __Particles_h__
#define __Particles_h__
#include "Common.h"
#include <memory>
#include <set>
#include <cstdbool>

template<int d> class Particles
{using VectorD=Vector<double,d>;
public:
	////attributes
	ArrayPtr<VectorD> x;		////position
	ArrayPtr<VectorD> v;		////velocity
	ArrayPtr<VectorD> f;		////force
	ArrayPtr<double> m;			////mass
	ArrayPtr<double> c;			////color
	ArrayPtr<double> r;			////radius
	ArrayPtr<double> p;			////pressure
	ArrayPtr<double> den;		////density
	ArrayPtr<int> idx;			////index, for rigid body
	
    //// project extensions
    ArrayPtr<std::set<int>> neighbors;	////neighbor list
    ArrayPtr<int> cardinality;  //// cardinality, i.e. number of neighbors
    ArrayPtr<int> nb_max;		//// maximum number of neighbors
    ArrayPtr<double> stiffness; //// stiffness parameter
    ArrayPtr<bool> equality;    //// equality --> true = equality, false = inequality
    ArrayPtr<double> w;         //// calculation weight --> just inverse of mass, not actual weight!
    

	//////////////////////////////////////////////////////////////////////////
	////common functions
	Particles()
	{
		if(x==nullptr)x.reset(new std::vector<VectorD>());	
		if(v==nullptr)v.reset(new std::vector<VectorD>());	
		if(f==nullptr)f.reset(new std::vector<VectorD>());	
		if(m==nullptr)m = std::make_shared<std::vector<double>>();
		if(c==nullptr)c = std::make_shared<std::vector<double>>();
		if(r==nullptr)r = std::make_shared<std::vector<double>>();
		if(p==nullptr)p = std::make_shared<std::vector<double>>();
		if(den==nullptr)den = std::make_shared<std::vector<double>>();
		if(idx==nullptr)idx = std::make_shared<std::vector<int>>();
        
        //// project extensions
        if (neighbors == nullptr)     neighbors     = std::make_shared<std::vector<std::set<int>>>();
        if (cardinality == nullptr)   cardinality   = std::make_shared<std::vector<int>>();
        if (neighbors == nullptr)     stiffness     = std::make_shared<std::vector<double>>();
        if (equality == nullptr)      equality      = std::make_shared<std::vector<bool>>();
        if (w == nullptr)             w             = std::make_shared<std::vector<double>>();
	}

	void Resize(const int size)
	{
		x->resize((size_type)size,VectorD::Zero());
		v->resize((size_type)size,VectorD::Zero());
		f->resize((size_type)size,VectorD::Zero());
		m->resize((size_type)size,(double)0);
		c->resize((size_type)size,(double)0);
		r->resize((size_type)size,(double)0);
		p->resize((size_type)size,(double)0);
		den->resize((size_type)size,(double)0);
		idx->resize((size_type)size,0);
        
        //// project extensions
        neighbors->resize((size_type)size);
        cardinality->resize((size_type)size);
        stiffness->resize((size_type)size);
        equality->resize((size_type)size);
        w->resize((size_type)size);
	}

	int Add_Element()
	{
		x->push_back(VectorD::Zero());
		v->push_back(VectorD::Zero());
		f->push_back(VectorD::Zero());
		m->push_back((double)0);
		c->push_back((double)0);
		r->push_back((double)0);
		p->push_back((double)0);
		den->push_back((double)0);
		idx->push_back(0);
        
        //// project extensions
        neighbors->push_back(std::set<int>());
        cardinality->push_back(0);
        stiffness->push_back(0);
        equality->push_back(false);
        w->push_back((double)0);
		return (int)x->size()-1;
	}

	int Size() const {return (int)(*x).size();}

	//////////////////////////////////////////////////////////////////////////
	////functions for separate attributes
	////functions for x
    
    //// Extension methods for Final Project.
    std::set<int>& Neighbors(const int i) {
        return (*neighbors)[i];
    }
    
    int Cardinality(const int i) {
        return (*cardinality)[i];
    }
    
    double Stiffness(const int i) {
        return (*stiffness)[i];
    }
    
    bool Equality(const int i) {
        return (*equality)[i];
    }
    
    double W(const int i) {
        return (*w)[i];
    }
    
    void InitializeWeights() {
        for (int i = 0; i < Size(); i++) {
            (*w)[i] = 1. / M(i);
        }
    }
    
    
    
	VectorD& X(const int i)
	{return (*x)[i];}
	
	const VectorD& X(const int i) const 
	{return (*x)[i];}

	std::vector<VectorD>* X()
	{return x.get();}

	const std::vector<VectorD>* X() const 
	{return x.get();}
	
	ArrayPtr<VectorD> XPtr()
	{return x;}
	
	const ArrayPtr<VectorD> XPtr() const
	{return x;}
	
	std::vector<VectorD>& XRef()
	{return *x;}

	const std::vector<VectorD>& XRef() const 
	{return *x;}

	//////////////////////////////////////////////////////////////////////////
	////functions for v
	VectorD& V(const int i)
	{return (*v)[i];}
	
	const VectorD& V(const int i) const 
	{return (*v)[i];}

	std::vector<VectorD>* V()
	{return v.get();}

	const std::vector<VectorD>* V() const 
	{return v.get();}
	
	ArrayPtr<VectorD> VPtr()
	{return v;}
	
	const ArrayPtr<VectorD> VPtr() const
	{return v;}
	
	std::vector<VectorD>& VRef()
	{return *v;}

	const std::vector<VectorD>& VRef() const 
	{return *v;}

	//////////////////////////////////////////////////////////////////////////
	////functions for f
	VectorD& F(const int i)
	{return (*f)[i];}
	
	const VectorD& F(const int i) const 
	{return (*f)[i];}

	std::vector<VectorD>* F()
	{return f.get();}

	const std::vector<VectorD>* F() const 
	{return f.get();}
	
	ArrayPtr<VectorD> FPtr()
	{return f;}
	
	const ArrayPtr<VectorD> FPtr() const
	{return f;}
	
	std::vector<VectorD>& FRef()
	{return *f;}

	const std::vector<VectorD>& FRef() const 
	{return *f;}

	//////////////////////////////////////////////////////////////////////////
	////functions for m
	double& M(const int i)
	{return (*m)[i];}
	
	const double& M(const int i) const 
	{return (*m)[i];}

	std::vector<double>* M()
	{return m.get();}

	const std::vector<double>* M() const 
	{return m.get();}
	
	ArrayPtr<double> MPtr()
	{return m;}
	
	const ArrayPtr<double> MPtr() const
	{return m;}
	
	std::vector<double>& MRef()
	{return *m;}

	const std::vector<double>& MRef() const 
	{return *m;}

	//////////////////////////////////////////////////////////////////////////
	////functions for c
	double& C(const int i)
	{return (*c)[i];}
	
	const double& C(const int i) const 
	{return (*c)[i];}

	std::vector<double>* C()
	{return c.get();}

	const std::vector<double>* C() const 
	{return c.get();}
	
	ArrayPtr<double> CPtr()
	{return c;}
	
	const ArrayPtr<double> CPtr() const
	{return c;}
	
	std::vector<double>& CRef()
	{return *c;}

	const std::vector<double>& CRef() const 
	{return *c;}

	//////////////////////////////////////////////////////////////////////////
	////functions for r
	double& R(const int i)
	{return (*r)[i];}
	
	const double& R(const int i) const 
	{return (*r)[i];}

	std::vector<double>* R()
	{return r.get();}

	const std::vector<double>* R() const 
	{return r.get();}
	
	ArrayPtr<double> RPtr()
	{return r;}
	
	const ArrayPtr<double> RPtr() const
	{return r;}
	
	std::vector<double>& RRef()
	{return *r;}

	const std::vector<double>& RRef() const 
	{return *r;}

	//////////////////////////////////////////////////////////////////////////
	////functions for p
	double& P(const int i)
	{return (*p)[i];}
	
	const double& P(const int i) const 
	{return (*p)[i];}

	std::vector<double>* P()
	{return p.get();}

	const std::vector<double>* P() const 
	{return p.get();}
	
	ArrayPtr<double> PPtr()
	{return p;}
	
	const ArrayPtr<double> PPtr() const
	{return p;}
	
	std::vector<double>& PRef()
	{return *p;}

	const std::vector<double>& PRef() const 
	{return *p;}

	//////////////////////////////////////////////////////////////////////////
	////functions for den
	double& D(const int i)
	{return (*den)[i];}
	
	const double& D(const int i) const 
	{return (*den)[i];}

	std::vector<double>* D()
	{return den.get();}

	const std::vector<double>* D() const 
	{return den.get();}
	
	ArrayPtr<double> DPtr()
	{return den;}
	
	const ArrayPtr<double> DPtr() const
	{return den;}
	
	std::vector<double>& DRef()
	{return *den;}

	const std::vector<double>& DRef() const 
	{return *den;}

	//////////////////////////////////////////////////////////////////////////
	////functions for idx
	int& I(const int i)
	{return (*idx)[i];}
	
	const int& I(const int i) const 
	{return (*idx)[i];}

	std::vector<int>* I()
	{return idx.get();}

	const std::vector<int>* I() const 
	{return idx.get();}
	
	ArrayPtr<int> IPtr()
	{return idx;}
	
	const ArrayPtr<int> IPtr() const
	{return idx;}
	
	std::vector<int>& IRef()
	{return *idx;}

	const std::vector<int>& IRef() const 
	{return *idx;}
};
#endif
