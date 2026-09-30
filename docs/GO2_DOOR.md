# Go2 turns a door knob, opens the door, and shuts it

Example 132 (`examples/132_go2_door`) walks the arm-carrying Go2 up to a latched
swing door, turns the round knob with its gripper, pushes the door open, walks
through, and shuts it behind itself so the latch catches again. The knob is
held only while both fingers are measured on it; apart from the latch, nothing
drives the door, and no part of the robot but the hand touches it.

![The Go2 turning the knob, pushing the door open, walking through, and pushing it shut](media/showcase-go2-door.gif)

```text
cargo run --release -p go2_door --example 132_go2_door -- --smoke
cargo run --release -p go2_door --example 132_go2_door -- --capture
```

`--capture` renders the README showcase (`docs/media/showcase-go2-door.*`, 960 x
540) after checking that the rendered run replays the headless run's final
state digest exactly: one frame every 4.5 s from the room view, and one every
0.4 s from close by, without the scan, while the hand works the knob.

## Assets

| Asset | What it is |
| --- | --- |
| `assets/robots/unitree_go2_arm.rne.robot.toml` | The welded Go2 (`unitree_go2_jump`) with a generic 5-DOF arm and a two-finger parallel gripper on its back: yaw, shoulder, elbow and wrist pitch, 0.28 m links, wrist roll, and two prismatic fingers opening to 0.08 m; 1.46 kg in all, effort limits 10 / 20 / 12 / 5 / 5 N·m and 40 N. It is not a specific product. `tools/generate_go2_arm_urdf.py` generates it. |
| `assets/robots/swing_door.rne.robot.toml` | A 0.96 m x 0.90 m, 6 kg leaf on a vertical hinge: closed at the joint's lower limit, opening to 100°, hinge damping 1 N·m·s/rad and friction 0.2 N·m. A round knob on each face, 56 mm across, 0.58 m up and 0.10 m from the free edge, turns ±1.2 rad on its spindle. |
| `assets/scenes/unitree_go2_door.rne.scene.toml` | Example 131's two rooms with the door hinged on the doorway's north jamb, 2 cm proud of the partition face so the leaf clears the wall at every angle. |

`UnitreeGo2ModelTrot::with_total_mass_kg` gives the trot the arm's extra weight,
and `UnitreeGo2ModelTrot::stand` holds the Go2 still on all four feet while the
arm works.

## The latch and the knob

The latch is modelled by the example: while it is engaged a stiff hinge motor
holds the leaf shut, as a bolt in its strike plate would. It lets go once the
knob has turned past 0.7 rad, and engages again when the leaf is back within
0.02 rad of shut with the knob within 0.3 rad of rest. A weak spring (0.5 N·m/rad)
returns the knob.

- **Squaring up.** The Go2 walks onto the knob's line, faces the door, and steps
  forward or back until the knob is 0.60 m ahead of its body. Then it stands.
- **Grasp.** The open hand goes to 0.08 m short of the knob, then around it,
  with the gripper centre servoed sideways and up onto the knob's centre (the
  palm stops 12 mm short, or it pushes the body back off its stance). The
  fingers close; once both are measured on the knob in the same step, the knob
  is welded to the hand where it is.
- **Turn.** The arm goes soft (25 N·m/rad) so it follows the knob instead of
  wrenching the body against the door, and the wrist roll turns the knob until
  the latch lets go.
- **Crack.** Holding the knob turned, the hand pushes on through it, following
  it as the leaf gives, until the leaf is 0.035 rad (2°) open, past the catch.
- **Let go.** The fingers open, the hand slides off the knob through the gap
  between them, the wrist unwinds, and the hand draws back. Each step of that
  order fixes a failure seen on the way: unwound on the knob, a finger still
  touching it levered the body into a 40° roll; drawn straight back, a fingertip
  caught behind the knob and pulled the leaf shut, so the latch caught again.

The knob's position relative to the body is read from the simulator while the
hand works it, standing in for a camera's detection; walking uses the SLAM
estimate only.

## How the door moves

- **Opening.** The Go2 walks over to the doorway's centre line and on through it
  with the hand turned 0.35 m to its left, toward the hinge. The hand meets the
  leaf near the hinge, and as the robot advances the contact slides outward
  along the leaf, so the leaf swings ahead of the body.
- **Closing.** From the far room the Go2 walks around the open leaf and back
  along a line 0.52 m east of the hinge, heading south with the hand on its
  right, and stops 0.65 m from the hinge, clear of the room-B knob.
- **Final nudge.** Standing beside the nearly shut leaf, the robot sweeps the
  hand from 0.30 m to 0.62 m out to its right, pressing the leaf shut, where the
  latch catches it.

Every walking command comes from the robot's own estimate: Mid-360 returns in
the sensor frame at their emission times, levelled by IMU attitude, de-skewed by
drifting leg odometry, and matched by `rne_slam::Slam2d`, as in example 131.
The door's position is given, as a map annotation would give it; its angle is
read only to score the run and to model the latch.

The Mid-360 sees the door like any other surface. Returns inside the door's
swing (1.05 m from the hinge, on the side it opens to) are left out of SLAM,
the way a map marks a moving object's zone: matched against a map that still
holds the closed door, the leaf the robot is pushing dragged the estimate up to
1.06 m off. Standing at the knob, the odometry's yaw-rate bias kept
integrating with no new scan matched, and the estimate drifted 0.3 rad; a scan
is now matched at least once a second. A weak match is more likely a wrong
one: over two runs good matches scored 0.90 to 1.00, while a 0.51 match had
jumped the estimate 0.5 m, so matches below 0.85 are rejected in favour of the
odometry prediction.

## Measured

| Measure | Result | Gate |
| --- | --- | --- |
| Knob held | 1.57 s, both fingers on it for all of it | both fingers every held step |
| Knob turned to | 0.809 rad; the latch let go at 24.0 s | > 0.7 rad |
| Other links on the knob | none (the palm may touch it too) | none |
| Door opened to | 1.506 rad (86°) | > 1.4 rad |
| Door left at | 0.000 rad, latched again at 134.8 s | < 0.02 rad, latched |
| Hand in contact with the leaf | 18.3 s | > 0 |
| Any other robot link in contact with the leaf | never | never |
| Localization error | 0.058 m RMS, 0.142 m worst | RMS < 0.15 m |
| Clearance to walls and furniture | at least 0.46 m | > 0.3 m |
| Lowest body height | 0.258 m | > 0.2 m |
| Whole sequence | 140.5 s | finishes |

**Robustness.** With the approach's stopping point moved to 1.70, 1.75, 1.80,
1.83, 1.85, 1.90, and 2.00 m, all seven runs pass every gate. On the way there,
two of seven had fallen in the far room's turn: with a damping of 4 the
stowed arm, now heavier at the hand, swung and rocked the trotting body up to
59°; at 10 the worst sway was 33°.

## Dressed interior

`examples/go2_indoor` dresses the two rooms for examples 131 and 132: oak
planks in room A, grey tiles in room B, a rug, plaster walls with baseboards,
a kitchen island, a bookshelf, board and cardboard crates, a sofa, plants, and a
counter. Every model is drawn **inside its object's collider**, and the sofa
is three colliders (seat, back, arms) so its seat really is lower than its
back: the Mid-360, the planner, and the clearance checks see what is drawn.
Floors and the rug are flat overlays. Textures are procedural and
deterministic, so captures replay byte for byte. Clearance checks read the
scene's fixed colliders (`go2_indoor::static_obstacles`) rather than a copied
list.

The added furniture changed the SLAM trajectory enough to push this run's
localization error from 0.059 m to 0.129 m RMS with the default matcher. With
all 360 scan beams and a finer search (7 samples per axis, 4 levels) it was
0.052 m, and example 131 went from 0.099 m to 0.042 m on the same settings.
Once the scan also saw the door (below), the door run settled at 0.081 m.

## Known limitations

- **Collision groups across URDF robots.** A URDF spawned with
  `self_collisions = false` puts every link in collision group 1 with group 1
  filtered out (`rne_urdf_import`, `CollisionGroups::without_self_collision(1)`).
  Two such robots in one scene therefore never touch each other. The example
  moves the door leaf to its own group; a per-robot group in the importer would
  fix it generally, but would also change existing multi-robot scenes such as
  the OpenArm pair.
- **Scans used to skip the door.** Before this was fixed, `sample_livox_mid360`
  skipped every URDF link in the scene, not just the Go2's own, so the door was
  invisible to the Mid-360. It now skips only the sensor robot's links; the
  test `mid360_scans_other_articulated_bodies_but_not_its_own_robot` counts
  2,141 door returns over five frames where the old filter gave 22 (wall edge
  points only).
- **The arm and the Mid-360.** The rig occlusion table was measured without an
  arm, and the scene's raycasts skip the robot's own links, so the arm does not
  shadow the scan.
- **The latch is a motor.** The door model has no latch bolt; the example
  engages and frees a stiff hinge motor on the knob angle and the leaf angle.
  The door has no closer.
- **The weld holds the knob's orientation.** A real grip lets a round knob
  swivel a little between the fingers. The weld does not, so as the leaf opens
  the held knob yaws with it and wrenches the hand; the example lets go at 2°.
  A weld that locks only the twist about the spindle would model the grip
  better, but tried here, an impulse joint in this Rapier version locking one
  angular axis on a multibody link also held the link's other rotation: a
  pendulum welded that way at its hinge swung 0.001 m where the free one swung
  0.157 m. The example does not use it.
