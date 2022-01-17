# COSC 89.18: Physical Computing

## Student: Amittai Wekesa

## Assignment 0: Dynamic System

### Design

For this lab, I modelled the solar system with the sun and the nine (or is it eight) planets.
I tried to maintain real parameters and only scale values down where applicable or after computations.

I used the gravitational formula $F = \frac{Gm_1 m_2}{r^2}$ to calculate the gravitational force between two planets / planet and sun.
I then used the dynamics equation $F = ma$ to then calculate the accelerations of each planed.

The starting velocities for starting the simulation were determined by the equation $v_0 = \sqrt{\frac{GM}{r}}$ where $G$ is the gravitational constant and $M$ is the mass of the planet.

### Results

* My system worked out fine, although the expanse is a bit sparse!
* When opened, one has to zoom out to be able to see the whole system.
* Some planets also tend to move rather slow. I tried to fiddle with the mass of the sun, initial velocities, etc., but I find that some of them start getting knocked into highly elliptic and inaccurate orbits. In my opinion, the slow motion is expected since the system is modelled with real planet data.

### Reflections

This was pretty fun. Reminiscent of a similar system we built in CS-1, although that was more focused on having building classes and objects. It was also 2D. It's definitely more interesting in 3D.