# \textsc{Dartmouth COSC 89.18: Physical Computing}

## \textsc{Assignment  02: Particle Physics}

## \textsc{Student: Amittai Wekesa}

### \textsc{Part 1}

For the first part of the assignment, I implemented the algorithms discussed in the lecture notes
to detect particle collisions in the system and update forces on the different bodies, then
integrate using $F = ma$ and $v_{i+1} = v_{i} + a_{i}$ to update the velocities and positions
of the particles.

I also edited `ImplicitGeometry.h` to add a new body supporting collision detection.

### \textsc{Part 2}

For the second part, I implemented the $Navier-Stokes$ equations to simulate the flow of an incompressible fluid.
The forces modelled are the pressure force, viscocity, and weight.
I also implemented surface tension to mimic a more realistic fluid boundary.
