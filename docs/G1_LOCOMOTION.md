# G1 locomotion: the torque pathway on a biped

The torque actuation pathway built for the Go2
([GO2_LOCOMOTION.md](GO2_LOCOMOTION.md)) ports to the official 23-DoF G1
humanoid. Everything here was measured on the dynamic multibody plant and is
pinned by tests in `unitree_g1_gait_episode.rs`.

## The pathway ports

- The reduced-coordinate joint readback agrees with the position-target
  convention on the settled 23-joint stand
  (`g1_joint_state_readback_matches_the_position_convention`).
- A ±20 N·m feed-forward on one knee moves it with the commanded sign while
  every other joint holds
  (`g1_feed_forward_torque_moves_a_knee_with_the_commanded_sign`).

## Gains scale with the plant

The direct port of the Go2's walking gains (kp 40–60, kd 0.5–2) **folds the
humanoid**: the hip carries an order of magnitude more gravity torque than
the quadruped's, so the torso sags through the proportional term, the robot
falls, and the crumpled contact chatter blows the solver up into NaNs — the
explosions are downstream of the fall, not of the torque mode.
Humanoid-scale gains (kp 300, kd 10 on hips and knees) hold the stand
quietly at full height, and the hips' large driven inertia is exactly what
makes that higher damping discretely stable at the same 60 Hz
(`g1_torque_pd_stand_needs_humanoid_gains` pins the fold and the stand).

## The hybrid architecture

The ankles are the discrete-stability bottleneck: their small inertia puts
the 60 Hz damping bound near zero, so they — and the light arms — stay
position-held while the eight proximal joints (hips, knees) run torque PD.
`UrdfSceneSim::set_joint_position_targets` updates the servo-held joints'
targets *without stepping*, so both control regimes advance in the same
physics tick. At the scripted gait's stable operating point the hybrid
marches at full height with bounded drift
(`g1_hybrid_torque_gait_steps_in_place`).

## Honest limits

The scripted G1 gait itself is a near-stationary stepper: at its full
stride (0.20 rad) it falls under **both** control regimes, so striding
locomotion on the G1 is a gait problem, not a torque-pathway problem. That
is the next chapter's work — the transport-objective search that out-walked
the Go2's scripted trot (`examples/61_go2_learned_sprint`) is the template.
