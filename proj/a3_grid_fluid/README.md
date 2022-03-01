# COSC 89.18: Physical Computing

## Programming Assignment 03: Smoke Simulation

## Student: Amittai Wekesa

&nbsp;

For this assignment, I implemented the equations studied in class to model smoke diffusion, advection, and vorticity.

I also implemented a generic source affecting the smoke field. This can either be radial (diverging or converging), tangential (spinning), or a mix of both.

I implemented vorticity as described in tfeh paper (the second approach):

![Vorticity Discussion by Andrew Selle, Nick Rasmussen, and Ronald Fedkiw](./vorticity.png)

\newpage

For the custom sources, I implemented a class that can be used to create a source that can be added to the smoke simulation.

Each source has it's own position information, velocity it induces, and the type it is. Additionally,each source has a `getVelocity()` method that returns the velocity vector that the source induces on a given position. If the position is outside of the source's radius of influence, the velocity is zero.

```cpp

Vector2 getVelocity(Vector2& position) const
{
  Vector2 velocity = Vector2::Zero();
  if (isInRange(position)) {
    Vector2 normal = position - this->pos;
    if (normal.norm() == 0) {
      double x = rand() / 100;
      x -= floor(x);
      double y = rand();
      y -= floor(y);
      
      return {x, y};
    }
    if (type == "radial") {
      velocity = normal.normalized() * this->vel;
    }
    else if (type == "tangential") {
      velocity = Vector2(normal[1], -normal[0]).normalized() * this->vel;
    }
    else if (type == "radial_tangential") {
      velocity = normal.normalized() * this->vel;
      velocity += Vector2(normal[1], -normal[0]).normalized() * this->vel;
      velocity /= 2;
    }
  }
  return velocity;
}
```
