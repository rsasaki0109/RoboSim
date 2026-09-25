# Height field terrain

`ColliderShape::HeightField` samples a surface over the local XZ plane, which is
the shape to reach for when a scene needs ground that is not flat. It behaves
like the rest of the collider shapes for ordinary bodies, and this document
exists for the one way it does not.

## A height field is a surface, not a volume

A cuboid ground plate is a solid: a body that ends a step inside it is pushed
back out, because the contact constraint has somewhere to push it to. A height
field is an open triangle surface with no thickness and no inside. Nothing about
it resolves a penetration — a body that finishes a step below the surface simply
keeps going, and does not come back.

For bodies moving under gravity and contact this never comes up, because the
per-step motion is far smaller than the geometry. It comes up when something
*drives* a body harder than the contact can resist within one step, which is
exactly what a stiff joint motor does.

## Measured

`a_stiff_motor_is_lost_through_an_open_height_field_but_not_through_a_solid_volume`
puts a 22 mm foot sphere on a prismatic multibody joint and commands it 0.40 m
below the hip — well past the ground — with a stiffness of 4000 and a 200 N
force ceiling. That is the regime a stiff legged stance puts its feet in.

| ground | 60 Hz | 240 Hz | 960 Hz |
| --- | --- | --- | --- |
| height field | **rests at −0.0500 m** (lost) | 0.0194 m | 0.0191 m |
| solid cuboid | 0.0170 m (dips to −0.0194 m) | 0.0170 m | 0.0170 m |

Two things to read out of this:

- **At 60 Hz the motor out-muscles the contact on both grounds.** The solid
  cuboid is penetrated by 19 mm and pushes the foot back out to its resting
  height. The height field is penetrated by the same kind of margin and cannot,
  so the foot ends at the motor's commanded position, below the terrain, and is
  gone.
- **Slope is not the variable.** The same measurement at 0, 10 and 20 degrees is
  identical. It is the per-step penetration against the contact, not the terrain
  shape.

Raising the rate fixes it because it bounds how far the motor can drive the body
in one step. 240 Hz was sufficient here; the sufficient rate depends on the
motor stiffness, the driven mass, and the collider size, so it is a property of
the robot rather than a constant to copy.

## What to do about it

- **Step fast enough for the stiffest motor in the scene.** This is the direct
  fix and the only one that addresses the cause.
- **Prefer a solid volume where the robot actually stands** and reserve the
  height field for terrain the robot does not press into with a stiff actuator.
  A cuboid or a compound beneath the sampled surface gives the contact somewhere
  to push back to.
- **Check for it rather than assume it did not happen.** A body lost through the
  surface leaves no contact and no error; it just falls. Asserting that a
  robot's mass-weighted centre stays above the terrain catches it, and a run in
  which it did not hold measures nothing about whatever was being tested.

## Not addressed

Giving the height field a skirt or a swept solid volume beneath the sampled
surface would remove the hazard at the source. Continuous collision detection
would bound the per-step penetration without raising the whole scene's rate;
`PhysicsWorldDesc` does not expose it today.
