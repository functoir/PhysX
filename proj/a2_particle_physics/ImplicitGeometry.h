#ifndef __ImplicitGeometry_h__
#define __ImplicitGeometry_h__

#include <random>
#include "Common.h"
#include "Driver.h"
#include "Particles.h"
#include "OpenGLMesh.h"
#include "OpenGLCommon.h"
#include "OpenGLWindow.h"
#include "OpenGLViewer.h"
#include "OpenGLMarkerObjects.h"
#include "OpenGLParticles.h"

template<int d> class ImplicitGeometry
{using VectorD=Vector<double,d>;
public:
	virtual double Phi(const VectorD& pos) const {return 0.;}
	virtual VectorD Normal(const VectorD& pos) const {return VectorD::Zero();}
};

template<int d> class Bowl : public ImplicitGeometry<d>
{using VectorD=Vector<double,d>;
public:
	VectorD center;
	double radius;
	Bowl(VectorD _center=VectorD::Zero(),double _radius=1.):center(_center),radius(_radius){}
	virtual double Phi(const VectorD& pos) const {return radius-(pos-center).norm();}
	virtual VectorD Normal(const VectorD& pos) const {return (center-pos).normalized();}
};

template<int d> class Sphere : public ImplicitGeometry<d>
{using VectorD=Vector<double,d>;
public:
	VectorD center;
	double radius;
	Sphere(VectorD _center=VectorD::Zero(),double _radius=1.):center(_center),radius(_radius){}
	virtual double Phi(const VectorD& pos) const {return (pos-center).norm()-radius;}
	virtual VectorD Normal(const VectorD& pos) const {return (pos-center).normalized();}
};


template<int d>
class Curve : Sphere<d>
{
    using VectorD=Vector<double,d>;
    
public:
    VectorD center;
    double radius;
    Curve(VectorD _center=VectorD::Zero(),double _radius=1.) : center(_center),radius(_radius){}
    virtual double Phi(const VectorD& pos) const {return (pos-center).norm()-radius;}
    virtual VectorD Normal(const VectorD& pos) const {return (pos-center).normalized();}
    
    OpenGLSegmentMesh* opengl_trace=nullptr;
    OpenGLViewer* driver=nullptr;
    OpenGLColor default_color=OpenGLColor(.0,1.,.0,1.);
    
    void Initialize(OpenGLViewer* _driver)
    {
        driver=_driver;
        opengl_trace=driver->Add_Interactive_Object<OpenGLSegmentMesh>();
        opengl_trace->mesh.Elements().resize(1);
        opengl_trace->mesh.Vertices().resize(1);
        opengl_trace->mesh.Vertices()[0]=Vector3(0.,0.,0.);
        opengl_trace->mesh.elements[0]=Vector2i(0,0);
        opengl_trace->Set_Data_Refreshed();
        opengl_trace->Initialize();
    
        auto opengl_circle=driver->Add_Interactive_Object<OpenGLCircle>();
        opengl_circle->n=64;
        opengl_circle->pos=V3(this->center);
        opengl_circle->radius=this->radius;
//        opengl_circle->color=OpenGLColor(1.f,.6f,.2f);
//        opengl_circle->line_width=4.f;
        opengl_circle->Set_Data_Refreshed();
        opengl_circle->Initialize();
    }
    
    ////Helper function to convert a vector to 3d, for c++ template
    Vector3 V3(const Vector2& v2){return {v2[0],v2[1],.0};}
    Vector3 V3(const Vector3& v3){return v3;}
    
    void Sync_Data(const std::vector<Vector3>& vertices)
    {
        int n=(int)vertices.size();
        opengl_trace->mesh.Vertices()=vertices;
        opengl_trace->mesh.Elements().resize(n-1);
        for(int i=0;i<n-1;i++)opengl_trace->mesh.Elements()[i]=Vector2i(i,i+1);
        opengl_trace->Set_Data_Refreshed();
    }
    
    void Sync_Data(const std::vector<Vector2>& vertices)
    {
        int n=(int)vertices.size();
        opengl_trace->mesh.Vertices().resize(n);
        for(int i=0;i<n;i++)opengl_trace->mesh.Vertices()[i]=Vector3(vertices[i][0],vertices[i][1],(double)0);
        opengl_trace->mesh.Elements().resize(n-1);
        for(int i=0;i<n-1;i++)opengl_trace->mesh.Elements()[i]=Vector2i(i,i+1);
        opengl_trace->Set_Data_Refreshed();
    }
    
    void Sync_Data(const Vector3* vertices,const int n)
    {
        opengl_trace->mesh.Vertices().resize(n);
        for(int i=0;i<n;i++)opengl_trace->mesh.Vertices()[i]=vertices[i];
        opengl_trace->mesh.Elements().resize(n-1);
        for(int i=0;i<n-1;i++)opengl_trace->mesh.Elements()[i]=Vector2i(i,i+1);
        opengl_trace->Set_Data_Refreshed();
    }
    
    void Sync_Data(const Vector2* vertices,const int n)
    {
        opengl_trace->mesh.Vertices().resize(n);
        for(int i=0;i<n;i++)opengl_trace->mesh.Vertices()[i]=Vector3(vertices[i][0],vertices[i][1],(double)0);
        opengl_trace->mesh.Elements().resize(n-1);
        for(int i=0;i<n-1;i++)opengl_trace->mesh.Elements()[i]=Vector2i(i,i+1);
        opengl_trace->Set_Data_Refreshed();
    }
    
    void Set_Color(const double r,const double g,const double b)
    {
        default_color=OpenGLColor((float)r,(float)g,(float)b,1.);
        if(opengl_trace!=nullptr)opengl_trace->color=default_color;
    }
    
    void Set_Linewidth(const double line_width)
    {
        if(opengl_trace!=nullptr)opengl_trace->line_width=(GLfloat)line_width;
    }
};

#endif
