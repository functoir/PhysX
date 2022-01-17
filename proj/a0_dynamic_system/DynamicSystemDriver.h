//////////////////////////////////////////////////////////////////////////
//// Dartmouth Physical Computing Programming Assignment 0
//// Author: AMITTAI WEKESA
//////////////////////////////////////////////////////////////////////////

#ifndef __DynamicSystemDriver_h__
#define __DynamicSystemDriver_h__
#include <random>
#include "Common.h"
#include "Driver.h"
#include "OpenGLMesh.h"
#include "OpenGLCommon.h"
#include "OpenGLWindow.h"
#include "OpenGLViewer.h"
#include "OpenGLMarkerObjects.h"
#include "OpenGLParticles.h"

#include <ctime>

#include <iostream>
using namespace std;

const int GLOBAL_SCALE = (int) (pow(10, 6));
const int MASS_SCALE = (int) (pow(10, 24));
const int TIME_SCALE = (int) (3.154 * pow(10, 4));

class DynamicSystemDriver : public Driver, public OpenGLViewer
{
    using Vector3f = Eigen::Vector3f;

public:
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //// simulation-related functions m
    
    /// PHYSICS CONSTANTS
    // PI
    const long double PI = 3.14159265358979323846;
    const long double G = 6.67 * pow(10, -11);
    
    
    //// particle data structures
    std::vector<Vector3> position;				//// array of particle positions (update this array in Advance Simulation for particle motion)
    std::vector<Vector3> velocity;				//// array of particle velocities
    std::vector<Vector3> color;					//// array of particle colors
    std::vector<double> radii;					//// array of particle radii
    double dt=.02;								//// time step
    
    /*TODO: add your own additional data structures for particles if necessary*/
    std::vector<double> masses;
    
    //// segment data structure (for visualization purpose only)
    std::vector<std::vector<Vector3> > segment_mesh;			//// array of segment meshes (update this array in Advance Simulation for segment motion if necessary)
    //// each segment mesh stores its segment vertices are stored as [s0_0,s0_1,s1_0,s1_1,s2_0,s2_1,...], with si_0 and si_1 as the first and second vertex of the segment si
    std::vector<Vector3> segment_colors;						//// specify a segment color for each segment_mesh (not each segment)
    
    //// TODO: initialize your particle system by initializing the values of particle position, velocity, color, radii, and number
    //// Attention: make sure to set the value of particle_number!
    void Initialize_Simulation()
    {
        addPlanets();
        cout << "Initialization Done!!" << endl;
    }
    
    /**
     * Function to generate a random number within a specified range.
     * Negatives allowed.
     * Integers? Floats? Who cares?
     * \param min: lower bound.
     * \param max: upper bound.
     * \return: random number in range, (double)
     */
    double randomFloat(int min = 0, int max = 1)
    {
        double num = rand();
        while (num > (max - min)) num /= 7;
        return num + min;
    }
    
    
    /**
     * Initialize the system and add all physical bodies.
     *
     */
    void addPlanets()
    {
        const double MASS_SUN = 1988500 * 10;
        const double MASS_MERCURY = 0.330;
        const double MASS_VENUS = 4.87;
        const double MASS_EARTH = 5.97;
        const double MASS_MARS = 0.642;
        const double MASS_JUPITER = 1898;
        const double MASS_SATURN = 568;
        const double MASS_URANUS = 86.8;
        const double MASS_NEPTUNE = 102;
        const double MASS_PLUTO = 0.0130;
        const double MASS_MOON = 0.073;
        
        
        const int RADIUS_SUN = 20;
        const int RADIUS_MERCURY = 5;
        const int RADIUS_VENUS = 12;
        const int RADIUS_EARTH = 12;
        const int RADIUS_MARS = 7;
        const int RADIUS_JUPITER = 28;
        const int RADIUS_SATURN = 24;
        const int RADIUS_URANUS = 18;
        const int RADIUS_NEPTUNE = 16;
        const int RADIUS_PLUTO = 5;
        
        
        const int DIST_MERCURY = 58;
        const int DIST_VENUS = 108;
        const int DIST_EARTH = 150;
        const int DIST_MARS = 228;
        const int DIST_JUPITER = 778;
        const int DIST_SATURN = 1438;
        const int DIST_URANUS = 2867;
        const int DIST_NEPTUNE = 4515;
        const int DIST_PLUTO = 5906;
        
        /* add masses */
        {
            masses.push_back(MASS_SUN);
            masses.push_back(MASS_MERCURY);
            masses.push_back(MASS_VENUS);
            masses.push_back(MASS_EARTH);
            masses.push_back(MASS_MARS);
            masses.push_back(MASS_JUPITER);
            masses.push_back(MASS_SATURN);
            masses.push_back(MASS_URANUS);
            masses.push_back(MASS_NEPTUNE);
            masses.push_back(MASS_PLUTO);
        }
        
        /* ADD RADII */
        {
            radii.push_back(RADIUS_SUN);
            radii.push_back(RADIUS_MERCURY);
            radii.push_back(RADIUS_VENUS);
            radii.push_back(RADIUS_EARTH);
            radii.push_back(RADIUS_MARS);
            radii.push_back(RADIUS_JUPITER);
            radii.push_back(RADIUS_SATURN);
            radii.push_back(RADIUS_URANUS);
            radii.push_back(RADIUS_NEPTUNE);
            radii.push_back(RADIUS_PLUTO);
        }
        
        {
            position.emplace_back(Vector3(0, 0, 0));
            position.emplace_back(Vector3(DIST_MERCURY, 0, 0));
            position.emplace_back(Vector3(DIST_VENUS, 0, 0));
            position.emplace_back(Vector3(DIST_EARTH, 0, 0));
            position.emplace_back(Vector3(DIST_MARS, 0, 0));
            position.emplace_back(Vector3(DIST_JUPITER, 0, 0));
            position.emplace_back(Vector3(DIST_SATURN, 0, 0));
            position.emplace_back(Vector3(DIST_URANUS, 0, 0));
            position.emplace_back(Vector3(DIST_NEPTUNE, 0, 0));
            position.emplace_back(Vector3(DIST_PLUTO, 0, 0));
            
        }
        
        {
            velocity.emplace_back(Vector3(0, 0, 0));
            velocity.emplace_back(Vector3(0, sqrt(G * MASS_SUN / DIST_MERCURY), 0));
            velocity.emplace_back(Vector3(0, sqrt(G * MASS_SUN / DIST_VENUS), 0));
            velocity.emplace_back(Vector3(0, sqrt(G * MASS_SUN / DIST_EARTH), 0));
            velocity.emplace_back(Vector3(0, sqrt(G * MASS_SUN / DIST_MARS), 0));
            velocity.emplace_back(Vector3(0, sqrt(G * MASS_SUN / DIST_JUPITER), 0));
            velocity.emplace_back(Vector3(0, sqrt(G * MASS_SUN / DIST_SATURN), 0));
            velocity.emplace_back(Vector3(0, sqrt(G * MASS_SUN / DIST_URANUS), 0));
            velocity.emplace_back(Vector3(0, sqrt(G * MASS_SUN / DIST_NEPTUNE), 0));
            velocity.emplace_back(Vector3(0, sqrt(G * MASS_SUN / DIST_PLUTO), 0));
        }
        
        {
            color.emplace_back(Vector3(255, 142, 0));
            color.emplace_back(Vector3(186, 0, 180));
            color.emplace_back(Vector3(255, 23, 77));
            color.emplace_back(Vector3(0, 0, 255));
            color.emplace_back(Vector3(255, 23, 0));
            color.emplace_back(Vector3(25, 25, 255));
            color.emplace_back(Vector3(6, 255, 0));
            color.emplace_back(Vector3(0, 0, 162));
            color.emplace_back(Vector3(23, 73, 162));
            color.emplace_back(Vector3(255, 255, 20));
        }
        
        
    }
    
    /**
     * Find the distance between two points represented as vectors.
     *
     * \param a: First point.
     * \param b: Second point.
     * \return Scalar distance between the points.
     */
    double findDistance(int body, int otherBody)
    {
        Vector3 positionA = position[body];
        Vector3 positionB = position[otherBody];
        double dx = positionA[0] - positionB[0];
        double dy = positionA[1] - positionB[1];
        double dz = positionA[2] - positionB[2];
        return sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
    }
    
    /**
     * Find the gravitational force between two weighted masses.
     * Assumes access to arrays containing their respective masses and positions.
     *
     * \param body: Current body.
     * \param otherBody: Second body.
     * \return: Force vector due to gravity (3D vector)
     */
    Vector3 gravitationalForce(int body, int otherBody)
    {
        double distance = findDistance(body, otherBody) * GLOBAL_SCALE;
        double bodyM = masses[body] * MASS_SCALE;
        double otherBodyM = masses[otherBody] * MASS_SCALE;
        double forceScalar = G * bodyM * otherBodyM / pow(distance, 2);
        Vector3 direction = position[body] - position[otherBody];
        direction.normalize();
        Vector3 forceVector = -direction * forceScalar;
        return forceVector;
    }
    
    //// TODO: advance your particle system by updating the particle position and velocity values
    void Advance_Simulation(const double dt, const double time)
    {
        /*Your implementation here. Please comment out the default implementation. */
        
        auto numBodies = position.size();
        
        /* Calculate total forces on each body in the system */
        std::vector<Vector3> forces;
        for (auto body = 0; body < numBodies; body++) {
            Vector3 netForce(0, 0, 0);
            for (int otherBody = 0; otherBody < numBodies; otherBody++) {
                if (body == otherBody) continue;
                netForce += gravitationalForce(body, otherBody);
            }
            forces.push_back(netForce);
        }
        
        for (int body = 0; body < numBodies; body++) {
            
            /* update position with current velocity */
            Vector3 posA = position[body] + velocity[body] * dt * TIME_SCALE;
            Vector3 acceleration = (forces[body] / masses[body]) / sqrt(TIME_SCALE);
            velocity[body] += acceleration * dt;
            Vector3 posB = position[body] + velocity[body] * dt * TIME_SCALE;
            position[body] = (posA + posB) / 2;
        }
    }
    
    //////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////
    //// visualization-related functions (no need to modify)
    
    virtual void Initialize(){OpenGLViewer::Initialize();}
    virtual void Run(){OpenGLViewer::Run();}
    
    //// visualization data
    std::vector<OpenGLSphere*> opengl_spheres;											////spheres
    std::vector<OpenGLSegmentMesh*> opengl_segments;									////segments
    
    //// synchronize simulation data to visualization data, called in OpenGLViewer::Initialize()
    virtual void Initialize_Data()
    {
        //// initialize simulation data
        Initialize_Simulation();
        
        //// initialize visualization data
        Initialize_Visualization();
        
        ////set OpenGL rendering environments
        auto dir_light=OpenGLUbos::Add_Directional_Light(glm::vec3(-1.f,-.1f,-.2f));
        OpenGLUbos::Set_Ambient(glm::vec4(.1f,.1f,.1f,1.f));
        OpenGLUbos::Update_Lights_Ubo();
    }
    
    void Initialize_Visualization()
    {
        int particle_number=position.size();
        opengl_spheres.resize(particle_number);
        double default_radius=0.1;
        Vector3 default_color=Vector3(0.,1.,1.);
        for(int i=0;i<particle_number;i++){
            auto* sphere=Add_Interactive_Object<OpenGLSphere>();
            sphere->pos=position[i];
            if(i<radii.size())sphere->radius=radii[i];
            else sphere->radius=default_radius;
            if(i<color.size()) Set_Color(sphere,OpenGLColor((float)color[i][0],(float)color[i][1],(float)color[i][2],1.));
            else Set_Color(sphere,OpenGLColor((float)default_color[0],(float)default_color[1],(float)default_color[2],1.));
            sphere->Set_Data_Refreshed();
            sphere->Initialize();
            opengl_spheres[i]=sphere;
        }
        
        if(!segment_mesh.empty()){
            for(int s=0;s<segment_mesh.size();s++){
                int segment_number=segment_mesh[s].size()/2;
                std::vector<Vector3>& segment_vertices=segment_mesh[s];
                Vector3 segment_color=Vector3(1.,1.,0.1);
                if(s<segment_color.size())segment_color=segment_colors[s];
                OpenGLSegmentMesh* opengl_segment_mesh=Add_Interactive_Object<OpenGLSegmentMesh>();
                opengl_segments.push_back(opengl_segment_mesh);
                opengl_segment_mesh->mesh.Elements().resize(segment_number);
                opengl_segment_mesh->mesh.Vertices().resize(segment_number*2);
                
                for(int i=0;i<segment_number;i++){
                    opengl_segment_mesh->mesh.Vertices()[i*2]=segment_vertices[i*2];
                    opengl_segment_mesh->mesh.Vertices()[i*2+1]=segment_vertices[i*2+1];
                    opengl_segment_mesh->mesh.Elements()[i]=Vector2i(i*2,i*2+1);
                }
                
                Set_Color(opengl_segment_mesh,OpenGLColor((float)segment_color[0],(float)segment_color[1],(float)segment_color[2],1.));
                Set_Line_Width(opengl_segment_mesh,8.f);
                opengl_segment_mesh->Set_Data_Refreshed();
                opengl_segment_mesh->Initialize();
            }
        }
    }
    
    //// this function synchronizes the simulation data from position and segment_vertices to the opengl viewer
    void Sync_Simulation_And_Visualization_Data()
    {
        int particle_number=position.size();
        
        ////update and sync data for spheres
        for(int i=0;i<particle_number;i++){
            opengl_spheres[i]->pos=position[i];
            opengl_spheres[i]->Set_Data_Refreshed();
        }
        
        if(!segment_mesh.empty()){
            for(int s=0;s<segment_mesh.size();s++){
                auto vtx_num=segment_mesh[s].size();
                auto segment_number=segment_mesh[s].size()/2;
                std::vector<Vector3>& segment_vertices=segment_mesh[s];
                OpenGLSegmentMesh* opengl_segment_mesh=opengl_segments[s];
                if(opengl_segment_mesh->mesh.Vertices().size()<vtx_num){
                    opengl_segment_mesh->mesh.Vertices().resize(vtx_num);
                }
                if(opengl_segment_mesh->mesh.Elements().size()<vtx_num/2){
                    opengl_segment_mesh->mesh.Elements().resize(vtx_num/2);
                    for(int i=0;i<vtx_num/2;i++){
                        opengl_segment_mesh->mesh.Elements()[i]=Vector2i(2*i,2*i+1);
                    }
                }
                
                for(int i=0;i<segment_number;i++){
                    opengl_segment_mesh->mesh.Vertices()[i*2]=segment_vertices[i*2];
                    opengl_segment_mesh->mesh.Vertices()[i*2+1]=segment_vertices[i*2+1];
                    opengl_segment_mesh->Set_Data_Refreshed();
                }
            }
        }
    }
    
    //// update simulation and visualization for each time step
    virtual void Toggle_Next_Frame()
    {
        static double time=0.;
        time+=dt;
        Advance_Simulation(dt,time);
        Sync_Simulation_And_Visualization_Data();
        
        
        OpenGLViewer::Toggle_Next_Frame();
    }
};
#endif