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

The scripted G1 gait itself is a near-stationary stepper across its entire
stable envelope: measured transport is under 0.1 m per 12 s at every stride
that stands, and 0.15 rad falls — under **both** control regimes. Striding
locomotion on the G1 is therefore a gait problem, not a torque-pathway
problem.

## The first real steps

`examples/62_g1_learned_stride` answers it with the Go2 transport search's
structure: a contact-gated Fourier torque overlay
(`UnitreeG1TorqueOverlay`, 48 coefficients on the eight proximal joints)
rides the hybrid tick, and the anti-cheat window-displacement objective —
scored as the ensemble median of ulp-perturbed replays, with solver
blow-ups from wild candidates deterministically scored at the floor —
searches for transport the stepper does not have.

The winner (`UnitreeG1TorqueOverlay::LEARNED_STRIDE`) is the first G1 gait
in these measurements that genuinely covers ground: **0.26 m per 8 s window
(3.5× the stepper), 0.68 m per 24 s**, at full height (0.784 m), dead
straight (|yaw| ≈ 0.01 rad), with ulp-perturbed replays inside a centimeter
of each other. It is a slow shuffle (0.028 m/s), honestly reported as such —
but the humanoid walks, and every tool that carried the Go2 campaign
(torque pathway, deterministic resumable CEM, ensemble objectives,
12-decimal pinning) carried straight over.
`learned_torques_make_the_g1_stride` pins the comparison, uprightness,
straightness, and a bit-exact replay — at the cross-platform bar the Go2
campaign's chaos-floor doctrine demands: a degraded humanoid orbit does not
merely score less, it can blow the solver up mid-step (the ulp-shifted
orbit on Linux CI did exactly that), so every replay runs under
catch-unwind (a panic is a fall) and the pinned claim is the **median of
three ulp-perturbed replays**.

The development and test profiles use `opt-level = 1` while retaining debug
assertions and symbols. Fully unoptimized physics builds can take a different
chaotic orbit and turn a long G1 replay into a solver blow-up; this modest
optimization keeps local and CI headless validation on the same practical
simulation path.
