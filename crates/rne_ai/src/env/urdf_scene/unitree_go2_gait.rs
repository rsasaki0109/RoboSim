use super::UrdfJointPositionTarget;

/// Command for the official Unitree Go2 diagonal-pair trot generator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeGo2GaitCommand {
    /// Thigh swing amplitude in radians, clamped to `[0, 0.3]`.
    pub stride_rad: f64,
    /// Additional swing-leg calf flexion in radians, clamped to `[0, 0.4]`.
    pub foot_lift_rad: f64,
    /// Simulation steps per gait cycle, clamped to `[40, 180]`.
    pub cycle_steps: u64,
    /// Hip-abduction posture correction in radians, clamped to `[-0.8, 0.8]`
    /// (well inside the official Go2 hip range of ±1.05 rad).
    ///
    /// Applied with opposite signs on the left and right legs, so a positive value
    /// shifts the stance laterally. A balance controller feeds the measured body
    /// roll back through this term to lean the legs against a disturbance; the
    /// scripted open-loop trot leaves it at zero.
    pub roll_correction_rad: f64,
    /// Thigh-pitch posture correction in radians, clamped to `[-0.3, 0.3]`.
    ///
    /// Applied uniformly to every thigh, pitching the body against forward or
    /// backward disturbances.
    pub pitch_correction_rad: f64,
    /// Left/right differential calf extension in radians, clamped to `[-0.5, 0.5]`.
    ///
    /// A positive value straightens the left calves and flexes the right calves,
    /// so the leg-length asymmetry rolls the body toward the shorter side. This is
    /// the "push up with the downhill legs" recovery channel — measured to actuate
    /// the same lean axis as the hip correction, but independently of it, which is
    /// what lets a balance controller escape the hip-authority saturation.
    pub lateral_extension_rad: f64,
}

impl Default for UnitreeGo2GaitCommand {
    fn default() -> Self {
        Self {
            stride_rad: 0.12,
            foot_lift_rad: 0.16,
            cycle_steps: 90,
            roll_correction_rad: 0.0,
            pitch_correction_rad: 0.0,
            lateral_extension_rad: 0.0,
        }
    }
}

/// Generates one force-limited target pose for all 12 official Go2 joints.
pub fn unitree_go2_trot_targets(
    step: u64,
    command: UnitreeGo2GaitCommand,
) -> [UrdfJointPositionTarget<'static>; 12] {
    let stride = command.stride_rad.clamp(0.0, 0.3);
    let lift = command.foot_lift_rad.clamp(0.0, 0.4);
    let cycle = command.cycle_steps.clamp(40, 180);
    let roll = command.roll_correction_rad.clamp(-0.8, 0.8);
    let pitch = command.pitch_correction_rad.clamp(-0.3, 0.3);
    let extension = command.lateral_extension_rad.clamp(-0.5, 0.5);
    let phase = (step % cycle) as f64 / cycle as f64;
    let a = gait_wave(phase);
    let b = gait_wave((phase + 0.5) % 1.0);
    let leg = |prefix: &'static str, wave: (f64, f64)| {
        // The Go2 hip abduction axes are not mirrored between sides, so the same
        // joint angle on every hip shifts all four feet the same lateral direction
        // and the body leans the opposite way. (Mirrored signs would merely widen
        // the stance, which stabilizes nothing.)
        let (hip, thigh, calf, side) = match prefix {
            "FL" => ("FL_hip", "FL_thigh", "FL_calf", 1.0),
            "FR" => ("FR_hip", "FR_thigh", "FR_calf", -1.0),
            "RL" => ("RL_hip", "RL_thigh", "RL_calf", 1.0),
            _ => ("RR_hip", "RR_thigh", "RR_calf", -1.0),
        };
        [
            target(hip, roll),
            target(thigh, 0.8 + stride * wave.0 + pitch),
            target(calf, -1.5 - lift * wave.1 + side * extension),
        ]
    };
    let fl = leg("FL", a);
    let fr = leg("FR", b);
    let rl = leg("RL", b);
    let rr = leg("RR", a);
    [
        fl[0], fl[1], fl[2], fr[0], fr[1], fr[2], rl[0], rl[1], rl[2], rr[0], rr[1], rr[2],
    ]
}

/// Fourier joint-offset overlay on the official trot.
///
/// This is the search space for optimized gaits: for every joint the overlay
/// adds `c + s₁·sin(2π·p) + k₁·cos(2π·p) + s₂·sin(π·p) + k₂·cos(π·p)` to the
/// scripted trot target, where `p` is the gait phase over a **two-cycle**
/// period. The half-frequency terms deliberately break the trot's half-cycle
/// symmetry — the symmetry that cancels every hand-scripted steering pattern —
/// and the twelve joints × five coefficients give a 60-dimensional space that
/// can express per-leg phase, asymmetry, and cycle-to-cycle alternation (see
/// `docs/GO2_LOCOMOTION.md` for the measured steering boundary motivating it).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeGo2GaitOverlay {
    /// Per-joint `[constant, sin, cos, half_sin, half_cos]` coefficients; legs
    /// ordered FL, FR, RL, RR, joints ordered hip, thigh, calf within each leg.
    pub coefficients: [[f64; 5]; 12],
}

impl UnitreeGo2GaitOverlay {
    /// The neutral overlay: reproduces the scripted trot exactly.
    pub const ZERO: Self = Self {
        coefficients: [[0.0; 5]; 12],
    };

    /// A turning gait found by deterministic cross-entropy search (seed 42,
    /// `examples/54_go2_learned_turn -- --train`) on the fast walking trot.
    ///
    /// Every hand-scripted steering mechanism measured in
    /// `docs/GO2_LOCOMOTION.md` fails to yaw this platform. This overlay is the
    /// first gait that turns it at a genuinely *sustained* rate — about
    /// 0.025 rad/s, positive across disjoint measurement windows — while
    /// staying upright; `learned_overlay_turns_the_walking_trot` pins that
    /// behavior. The magnitude is also the honest boundary: an order of
    /// magnitude short of practical steering, because additive joint offsets on
    /// a fixed contact schedule cannot re-sequence the contacts.
    pub const LEARNED_TURN: Self = Self {
        coefficients: [
            [0.129912, -0.217959, 0.002348, -0.189749, 0.064767],
            [0.071184, -0.071463, -0.212554, -0.046149, 0.032646],
            [-0.233136, -0.043006, -0.039531, 0.077401, -0.024237],
            [0.078918, -0.039693, 0.089503, -0.046611, 0.045946],
            [-0.166883, 0.103000, 0.176065, 0.094167, 0.000696],
            [0.001261, -0.017031, 0.062574, -0.052474, -0.150523],
            [-0.128368, -0.062886, 0.046139, -0.018919, 0.022867],
            [-0.020196, -0.053897, -0.047735, -0.066953, 0.051198],
            [0.008270, -0.045094, -0.144627, -0.154906, -0.014030],
            [-0.230480, 0.192577, 0.051289, -0.210476, -0.151049],
            [-0.080313, -0.050767, -0.048977, 0.080942, -0.007959],
            [0.045380, -0.051971, -0.030212, 0.129360, -0.120137],
        ],
    };

    /// Joint offsets at a two-cycle gait phase in `[0, 1)`, each clamped to
    /// ±0.5 rad so no candidate can command a joint far outside the gait's
    /// working range.
    pub fn offsets(&self, two_cycle_phase: f64) -> [f64; 12] {
        let full = 4.0 * std::f64::consts::PI * two_cycle_phase;
        let half = 2.0 * std::f64::consts::PI * two_cycle_phase;
        let (sin, cos) = full.sin_cos();
        let (half_sin, half_cos) = half.sin_cos();
        let mut offsets = [0.0; 12];
        for (offset, coefficient) in offsets.iter_mut().zip(self.coefficients.iter()) {
            *offset = (coefficient[0]
                + coefficient[1] * sin
                + coefficient[2] * cos
                + coefficient[3] * half_sin
                + coefficient[4] * half_cos)
                .clamp(-0.5, 0.5);
        }
        offsets
    }
}

/// The official trot with a Fourier overlay added joint by joint.
pub fn unitree_go2_trot_targets_with_overlay(
    step: u64,
    command: UnitreeGo2GaitCommand,
    overlay: &UnitreeGo2GaitOverlay,
) -> [UrdfJointPositionTarget<'static>; 12] {
    let mut targets = unitree_go2_trot_targets(step, command);
    let cycle = command.cycle_steps.clamp(40, 180);
    let two_cycle_phase = (step % (2 * cycle)) as f64 / (2 * cycle) as f64;
    let offsets = overlay.offsets(two_cycle_phase);
    for (target, offset) in targets.iter_mut().zip(offsets.iter()) {
        target.position += offset;
    }
    targets
}

/// Contact-gated Fourier feed-forward *torque* overlay on the torque-PD walk.
///
/// This is the force-level mirror of [`UnitreeGo2GaitOverlay`]: for every joint
/// it adds `g·(stance gate) + c + s₁·sin(2π·p) + k₁·cos(2π·p) + s₂·sin(π·p) +
/// k₂·cos(π·p)` newton-meters of feed-forward torque on top of the tracking
/// PD, where `p` is the two-cycle gait phase and the stance gate is the leg's
/// *measured* foot contact — a coupling to the actual contact state that no
/// position-space overlay can express. Nine hand-designed steering mechanisms
/// across both actuation regimes produce no sustained yaw on this platform
/// (see `docs/GO2_LOCOMOTION.md`); the twelve joints × six coefficients give
/// the 72-dimensional space the torque-space search runs in.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeGo2TorqueOverlay {
    /// Per-joint `[stance_const, constant, sin, cos, half_sin, half_cos]`
    /// torque coefficients in N·m; legs ordered FL, FR, RL, RR, joints ordered
    /// hip, thigh, calf within each leg.
    pub coefficients: [[f64; 6]; 12],
}

impl UnitreeGo2TorqueOverlay {
    /// The neutral overlay: reproduces the plain torque-PD walk exactly.
    pub const ZERO: Self = Self {
        coefficients: [[0.0; 6]; 12],
    };

    /// A turning gait found by deterministic cross-entropy search (seed 42,
    /// `examples/56_go2_torque_turn -- --train`) on the torque-PD walking trot.
    ///
    /// This is the measurement the torque arc exists for: three position-space
    /// searches and a five-fold torque scan all plateaued at ~0.02 rad/s, which
    /// localized the steering boundary in the contact mechanics under hard
    /// position servos. This overlay sustains ~0.034 rad/s — above every
    /// position-space result — while the robot keeps walking, by shaping
    /// stance-leg torques the servos could never express;
    /// `learned_torque_overlay_out_turns_the_position_plateau` pins it.
    /// A chaos-robust turning gait found by the *ensemble-median* CEM
    /// (`examples/56_go2_torque_turn -- --train-robust`, seed 42, warm-started
    /// from [`Self::LEARNED_TURN`]).
    ///
    /// The single-trajectory winner turned out to live on a knife edge: its
    /// second measurement window swings ±0.3 rad under one-ulp perturbations
    /// (the ~16 s chaos horizon in `docs/GO2_LOCOMOTION.md`). Scoring the
    /// median of three perturbed replays instead selects for gaits whose turn
    /// survives trajectory-level noise — and the winner it found is locally
    /// *contracting*: its perturbed replays land on the same windows
    /// (+0.250/+0.274 rad per 8 s, ~0.031 rad/s) instead of diverging.
    /// `robust_torque_turn_survives_perturbation` pins that property.
    pub const LEARNED_ROBUST_TURN: Self = Self {
        coefficients: [
            [
                -1.344032192829,
                -0.800645266699,
                -0.119000789846,
                0.260304857208,
                -0.604194423541,
                -0.272409233533,
            ],
            [
                -1.113048181238,
                0.137764794535,
                -0.927880548188,
                -0.400064359385,
                -0.592582951512,
                -0.467594143673,
            ],
            [
                -2.630116511638,
                -0.908808017816,
                1.343855005722,
                -0.197648017923,
                -0.336050769397,
                0.564757379100,
            ],
            [
                -0.347502081487,
                -0.109015627985,
                0.645110936273,
                -0.881021767904,
                -0.646562242222,
                0.509986312813,
            ],
            [
                0.177568417164,
                -2.981399368489,
                0.363701821573,
                -0.318981206426,
                -0.208701515043,
                -1.958031908291,
            ],
            [
                -1.819395566336,
                -0.387826861631,
                -1.791286440581,
                -0.458175260573,
                -0.555312597872,
                0.226706496853,
            ],
            [
                -0.542436115975,
                0.862706137390,
                -0.763200481265,
                -0.589259219427,
                0.773874160789,
                -1.606538970087,
            ],
            [
                -0.067741171244,
                -2.136433969606,
                -0.776040000364,
                -0.354388027298,
                2.383252183144,
                0.514102596678,
            ],
            [
                -0.063743487961,
                0.020082243324,
                0.245389314781,
                -1.266169200995,
                -0.738189468682,
                2.157687929519,
            ],
            [
                0.575858501217,
                0.005801549348,
                1.695861443421,
                -0.780431331315,
                -1.232766041230,
                -1.021659781892,
            ],
            [
                -0.970284937507,
                0.041379702001,
                0.237385054745,
                -1.383587224545,
                -0.489221759291,
                0.627853387139,
            ],
            [
                0.938686719533,
                -1.518392179889,
                -0.402910488196,
                -0.448164145462,
                0.876773734300,
                0.635796162852,
            ],
        ],
    };

    /// The coefficients are pinned at the search state's full 12-decimal
    /// precision: the contact-gated rollout is chaotic enough that rounding
    /// them to 6 decimals lands on a different (non-turning) trajectory.
    pub const LEARNED_TURN: Self = Self {
        coefficients: [
            [
                -1.327191630960,
                -0.443702376317,
                0.107819976633,
                -0.171779321719,
                -0.632204009006,
                -0.049557599946,
            ],
            [
                -0.932011779890,
                -0.652217401467,
                -0.935988874340,
                -0.042604218599,
                -0.298732599012,
                -0.474388218727,
            ],
            [
                -2.261866843812,
                -0.707556500989,
                1.509843263500,
                -0.663352488635,
                -0.218519778592,
                0.525062510185,
            ],
            [
                -0.501438491381,
                -0.670995319983,
                0.696193324812,
                -0.417298785439,
                -0.398595223810,
                0.384560587744,
            ],
            [
                0.374943544521,
                -2.841783218489,
                -0.178455015705,
                -0.013116078914,
                -0.616831677540,
                -1.429023665516,
            ],
            [
                -1.777217236609,
                -0.268534735568,
                -1.697381895963,
                -0.454282621106,
                -0.480144779936,
                0.088435830409,
            ],
            [
                -0.405104492760,
                0.962020679169,
                -0.102913928300,
                -0.943228596202,
                0.224753254702,
                -1.117856295123,
            ],
            [
                -0.247448823776,
                -2.196627917665,
                -0.423107552283,
                -0.226034176651,
                1.963312881826,
                0.260406823170,
            ],
            [
                -0.421418640647,
                -0.840759197393,
                0.061748964210,
                -0.925564774779,
                -0.744693016349,
                1.936534693867,
            ],
            [
                0.759346604287,
                0.241073789704,
                1.528121788456,
                -1.112941656111,
                -0.595139709508,
                -0.998664941771,
            ],
            [
                -0.487578942316,
                0.258013847934,
                0.425145137766,
                -0.157244158329,
                0.173974596728,
                0.441134060631,
            ],
            [
                0.718453115860,
                -0.503509358315,
                -0.008691286076,
                -0.261935190200,
                0.689870925884,
                0.931042114500,
            ],
        ],
    };

    /// Feed-forward joint torques at a two-cycle gait phase in `[0, 1)` with
    /// the measured per-leg stance gates (legs ordered FL, FR, RL, RR), each
    /// clamped to ±8 N·m so no candidate can out-shout the tracking PD.
    pub fn torques_nm(&self, two_cycle_phase: f64, stance: [bool; 4]) -> [f64; 12] {
        let full = 4.0 * std::f64::consts::PI * two_cycle_phase;
        let half = 2.0 * std::f64::consts::PI * two_cycle_phase;
        let (sin, cos) = full.sin_cos();
        let (half_sin, half_cos) = half.sin_cos();
        let mut torques = [0.0; 12];
        for (index, (torque, coefficient)) in
            torques.iter_mut().zip(self.coefficients.iter()).enumerate()
        {
            let gate = if stance[index / 3] { 1.0 } else { 0.0 };
            *torque = (coefficient[0] * gate
                + coefficient[1]
                + coefficient[2] * sin
                + coefficient[3] * cos
                + coefficient[4] * half_sin
                + coefficient[5] * half_cos)
                .clamp(-8.0, 8.0);
        }
        torques
    }
}

/// Number of features a [`UnitreeGo2TorquePolicy`] consumes.
pub const UNITREE_GO2_POLICY_FEATURES: usize = 8;

/// State-feedback torque policy: a linear map from body-state features to
/// per-joint feed-forward torques on the torque-PD walk.
///
/// Where [`UnitreeGo2TorqueOverlay`] indexes its torques by gait *phase*, this
/// policy reads the *state* — the actual lean, lean rates, and yaw rate — so
/// it can react to what the body is doing instead of replaying a clock. The
/// features (assembled by the caller, see `examples/57_go2_torque_policy`)
/// are chosen yaw-invariant and wrap-free: body-frame components of the
/// world-up direction and of the angular velocity, the world yaw rate, the
/// two-cycle gait phase as sine/cosine, and a bias term.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeGo2TorquePolicy {
    /// Per-joint feature weights in N·m per unit feature; legs ordered FL,
    /// FR, RL, RR, joints ordered hip, thigh, calf within each leg.
    pub weights: [[f64; UNITREE_GO2_POLICY_FEATURES]; 12],
}

impl UnitreeGo2TorquePolicy {
    /// The neutral policy: reproduces the plain torque-PD walk exactly.
    pub const ZERO: Self = Self {
        weights: [[0.0; UNITREE_GO2_POLICY_FEATURES]; 12],
    };

    /// The authority-hypothesis winner, pinned as a NEGATIVE result
    /// (`examples/59_go2_authority_turn -- --train`, seed 42).
    ///
    /// Hypothesis: raising the feed-forward clamp to ±12 N·m and giving the
    /// policy an integral of the yaw-rate error (a learned PI structure)
    /// would lift the commanded turn's absolute amplitude above the ±0.3 rad
    /// cross-platform chaos floor. Refuted: the best commanded score reached
    /// 0.094 — *below* the ±8 N·m winner's 0.188 — and its windows stay an
    /// order of the floor's size too small. Feed-forward authority is not the
    /// lever; the commanded amplitude is bounded by the platform's turn
    /// capability, which is itself the size of the chaos floor.
    /// `authority_and_integral_do_not_lift_the_commanded_turn` pins this so
    /// it cannot rot. Feature convention is example 59's (integral in slot 3).
    pub const LEARNED_AUTHORITY_TURN: Self = Self {
        weights: [
            [
                -1.534138230421,
                -0.132141010923,
                -1.062841252385,
                -0.474775042218,
                1.892636448202,
                1.522633656909,
                -0.664034108358,
                -0.631362118104,
            ],
            [
                -1.718918285029,
                -0.167284275788,
                0.303720103870,
                0.878512918009,
                -2.735702424900,
                0.351427463906,
                -0.084000392633,
                -0.258257454501,
            ],
            [
                -2.511522836997,
                -3.613633533479,
                -0.523856650618,
                1.102879288365,
                -0.706826935142,
                1.117531315895,
                -1.022285346689,
                1.596013184925,
            ],
            [
                1.154745548106,
                1.087787284766,
                -0.739044886930,
                0.462592131641,
                -0.403166183408,
                0.084559640298,
                -0.382928177918,
                1.675236479140,
            ],
            [
                -0.339901690568,
                1.057314244967,
                0.954465590395,
                0.119369548089,
                -0.662067234987,
                1.664634657162,
                -1.815742410559,
                -1.043454592531,
            ],
            [
                -0.250928707739,
                -0.179620749098,
                0.421336481399,
                -0.425869368478,
                0.479545805094,
                0.522511010513,
                0.302131897525,
                0.071531477564,
            ],
            [
                -1.057228387278,
                -0.187792700480,
                -0.996016168503,
                -1.200153561819,
                1.321231823670,
                1.770771636899,
                -1.445715139297,
                -1.426693162415,
            ],
            [
                0.235378782920,
                0.348076263633,
                -0.830929354804,
                0.369296928355,
                0.249073436351,
                0.564813932215,
                -0.281517398510,
                -1.036497915140,
            ],
            [
                -0.017997379958,
                1.995558601062,
                0.257553246001,
                0.259013742974,
                -0.256097002148,
                1.402095880290,
                0.116927270736,
                -1.269646035614,
            ],
            [
                0.659953906830,
                -2.348031396800,
                -1.554121833621,
                -0.196765514887,
                -1.111252469945,
                1.252056338837,
                0.651813785942,
                -3.306221977909,
            ],
            [
                -1.068576505778,
                0.780814681691,
                -0.640639621364,
                0.520039981546,
                -1.790539942253,
                -0.067172913986,
                -0.416175852522,
                -0.522452980420,
            ],
            [
                0.127404776239,
                1.013712489546,
                0.693524254715,
                0.571840440332,
                -1.061780056238,
                -0.812121876085,
                -1.097808084893,
                -1.374078787373,
            ],
        ],
    };

    /// A commanded yaw-rate tracking policy found by the two-direction CEM
    /// (`examples/58_go2_steered_turn -- --train`, seed 42).
    ///
    /// The first controller on this platform whose turn *direction obeys a
    /// command*: feature 4 is the tracking error against a commanded yaw
    /// rate (see example 58's feature assembly), and the search scored every
    /// candidate by the worse of its two commanded directions, so no
    /// chaos-selected one-way orbit could win. With one set of weights the
    /// commanded sign flips the turn (+0.094/+0.117 rad per window under
    /// +0.25 rad/s, −0.095/−0.109 under −0.25). The magnitude is honest and
    /// modest (~0.013 rad/s, far below the reference) — command obedience is
    /// the result, rate tracking is not yet.
    /// `commanded_yaw_reference_steers_both_ways` pins it.
    pub const LEARNED_COMMANDED_TURN: Self = Self {
        weights: [
            [
                -1.300013615065,
                -0.034158782561,
                -1.265988014318,
                -0.136165458142,
                1.057769577131,
                1.680180767009,
                -0.624380996855,
                -0.466534813983,
            ],
            [
                -0.676761971810,
                0.236085351664,
                1.020278420112,
                0.035778384761,
                -3.025841493720,
                0.419616496579,
                -0.860839669793,
                -0.314750326771,
            ],
            [
                -1.890382835716,
                -4.030223304320,
                -0.130196339972,
                -2.037112675725,
                -1.150920121307,
                0.408695097791,
                -1.488576352776,
                0.535080028695,
            ],
            [
                -0.075649078643,
                0.194932660119,
                -1.683898757400,
                4.707546081473,
                -0.280795614460,
                -0.486795140443,
                -0.386345535433,
                1.301704141828,
            ],
            [
                -0.230290825454,
                1.141775859124,
                0.570876274350,
                -0.464300229008,
                -0.723158604552,
                0.734468987765,
                -1.516022908286,
                -0.773815701898,
            ],
            [
                -0.206601259318,
                0.582320325722,
                -0.425883429832,
                0.761097575337,
                0.666006897447,
                0.644805232700,
                0.402224880334,
                -0.242488740600,
            ],
            [
                -0.430188643181,
                -0.337919474141,
                -0.623647932103,
                0.138914412239,
                1.843993388465,
                1.380103005993,
                -0.886513459464,
                -1.739800660845,
            ],
            [
                0.270005822689,
                0.059969871345,
                -1.171904027812,
                0.647441855047,
                -0.020353970741,
                0.959172932254,
                -0.370817351481,
                -0.015688986902,
            ],
            [
                -0.067765437679,
                1.366379762717,
                0.438741786207,
                0.334216755109,
                0.104480949010,
                1.182878480142,
                0.607508023165,
                -1.696485893321,
            ],
            [
                0.432897388264,
                -2.179728442628,
                -1.281164035593,
                -0.073987860785,
                0.088578231434,
                0.623906309158,
                0.304734062384,
                -2.910127834363,
            ],
            [
                -1.396366862045,
                0.317028538926,
                -0.514579378821,
                0.712970428926,
                -1.050273325067,
                0.529207860000,
                0.006273086227,
                -0.080410784257,
            ],
            [
                0.478626415795,
                1.116694294581,
                1.145937360142,
                0.590288733253,
                -0.556831014836,
                -0.889947052020,
                -0.469007359741,
                -0.534198127063,
            ],
        ],
    };

    /// A turning policy found by the ensemble-median CEM
    /// (`examples/57_go2_torque_policy -- --train`, seed 42).
    ///
    /// The first closed-loop controller in the steering campaign. Its turn
    /// rate matches the feed-forward torque overlay's rather than beating it
    /// (+0.226/+0.346 rad per 8 s window versus the overlay's
    /// +0.250/+0.274), but it holds a different operating point: it keeps
    /// walking while it turns (3.8 m per 24 s versus the overlay's 1.4 m),
    /// and its ulp-perturbed replays land on identical windows — the
    /// contraction the ensemble objective selects for.
    /// `torque_policy_turns_while_walking` pins it. Weights carry the search
    /// state's full 12-decimal precision (contact-gated rollouts diverge
    /// under 6-decimal rounding).
    pub const LEARNED_TURN: Self = Self {
        weights: [
            [
                -2.145808389623,
                -1.126046872952,
                -0.597103148949,
                -0.675189417817,
                -0.997585472110,
                -0.095727755957,
                -1.959122908352,
                -0.720299659313,
            ],
            [
                -1.367480630837,
                -0.319674298622,
                1.239810137798,
                0.475987165301,
                0.103277581848,
                -1.161936863572,
                0.529762757140,
                0.890245118620,
            ],
            [
                -2.489505543492,
                1.290223704962,
                0.940870247097,
                1.127111067814,
                -0.369626844344,
                -1.358777203364,
                0.015121827241,
                2.180899511817,
            ],
            [
                0.135790941006,
                -0.094514014930,
                0.737233032536,
                1.423106908225,
                0.013667574758,
                1.037591389899,
                -1.876302102809,
                1.010061036492,
            ],
            [
                -0.756340285587,
                1.301265161921,
                0.506445238849,
                -0.232824764885,
                0.708145248270,
                0.351430617467,
                0.367686568878,
                -0.132993113046,
            ],
            [
                0.996535207636,
                0.644884301717,
                1.820296793995,
                0.458033586880,
                1.110328516048,
                0.451426198316,
                0.152377370601,
                0.821719231219,
            ],
            [
                -3.262660835022,
                0.667865331261,
                0.845412970022,
                -0.624704643637,
                -0.380117740803,
                1.389751112707,
                -0.719805195352,
                0.265655581028,
            ],
            [
                2.179340315300,
                -0.084067895669,
                -0.105718113365,
                -0.895470863883,
                2.355916010865,
                0.104000843679,
                0.352087088152,
                0.633060777704,
            ],
            [
                0.312117351201,
                -0.445618663938,
                0.162126868301,
                -1.599995214348,
                0.028208550419,
                -0.790196627072,
                0.183677776535,
                -0.059207232171,
            ],
            [
                -0.026682593897,
                0.428415407107,
                0.328180598002,
                0.487196584186,
                1.633253497041,
                2.029198752019,
                -0.963719791062,
                -1.262231035611,
            ],
            [
                0.536803995574,
                -1.070932574760,
                1.365875830251,
                -1.008598350931,
                0.774291418253,
                1.024874565899,
                -1.508930873931,
                1.694697197859,
            ],
            [
                -0.396876276988,
                1.332780914849,
                0.755480039540,
                0.877023416338,
                0.328901863491,
                0.556788231928,
                0.857358232396,
                -0.016185092323,
            ],
        ],
    };

    /// Feed-forward joint torques for a feature vector, each clamped to
    /// ±8 N·m so the policy cannot out-shout the tracking PD.
    pub fn torques_nm(&self, features: &[f64; UNITREE_GO2_POLICY_FEATURES]) -> [f64; 12] {
        self.torques_nm_with_limit(features, 8.0)
    }

    /// Feed-forward joint torques with an explicit per-joint clamp in N·m.
    ///
    /// The commanded-authority search uses a higher clamp than the original
    /// ±8 N·m: the absolute obedience of the ±8 winner sits below the
    /// cross-platform chaos floor, and authority is one of its levers. The
    /// clamp is part of a pinned policy's operating contract — replaying a
    /// constant with a different limit is a different controller.
    pub fn torques_nm_with_limit(
        &self,
        features: &[f64; UNITREE_GO2_POLICY_FEATURES],
        limit_nm: f64,
    ) -> [f64; 12] {
        let mut torques = [0.0; 12];
        for (torque, weights) in torques.iter_mut().zip(self.weights.iter()) {
            *torque = weights
                .iter()
                .zip(features.iter())
                .map(|(weight, feature)| weight * feature)
                .sum::<f64>()
                .clamp(-limit_nm, limit_nm);
        }
        torques
    }
}

fn gait_wave(phase: f64) -> (f64, f64) {
    gait_wave_with_duty(phase, 0.7)
}

/// Stance/swing waveform with an explicit duty factor: the thigh ramps
/// backward through stance (`phase < duty`) and returns with a lifted calf
/// through swing.
fn gait_wave_with_duty(phase: f64, duty: f64) -> (f64, f64) {
    if phase < duty {
        (1.0 - 2.0 * phase / duty, 0.0)
    } else {
        let swing = (phase - duty) / (1.0 - duty);
        (-1.0 + 2.0 * swing, (std::f64::consts::PI * swing).sin())
    }
}

/// Per-leg contact-schedule parameters for the generalized Go2 gait.
///
/// This is the search space the fixed trot cannot express: *when* each leg is
/// on the ground (`phase_offset`, `duty`), how far it strides
/// (`stride_scale`), and where it is planted laterally (`hip_offset_rad` at
/// touchdown, swept by `hip_stance_sweep_rad` through stance). Re-sequencing
/// contacts is what steering requires on a platform without hip-yaw joints
/// (see `docs/GO2_LOCOMOTION.md`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeGo2LegSchedule {
    /// Gait-phase offset of this leg in `[0, 1)`.
    pub phase_offset: f64,
    /// Stance fraction of the cycle, clamped to `[0.3, 0.85]`.
    ///
    /// Below 0.5 the two diagonal pairs no longer cover the cycle and the
    /// gait acquires flight phases — the aerial-duty region the steering
    /// campaign's chaos-floor closure named as a morphological lever.
    pub duty: f64,
    /// Multiplier on the commanded stride, clamped to `[0.3, 1.6]`.
    pub stride_scale: f64,
    /// Constant hip-abduction placement offset in radians, clamped to ±0.4.
    pub hip_offset_rad: f64,
    /// Hip sweep through stance in radians, clamped to ±0.35: the hip is at
    /// `+sweep` at touchdown and `-sweep` at liftoff, dragging the stance foot
    /// laterally under the body and repositioning it during swing.
    pub hip_stance_sweep_rad: f64,
}

impl UnitreeGo2LegSchedule {
    /// The scripted trot's schedule for a leg with the given phase offset.
    pub const fn trot(phase_offset: f64) -> Self {
        Self {
            phase_offset,
            duty: 0.7,
            stride_scale: 1.0,
            hip_offset_rad: 0.0,
            hip_stance_sweep_rad: 0.0,
        }
    }
}

/// Full contact schedule for all four legs, ordered FL, FR, RL, RR.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeGo2GaitSchedule {
    /// Per-leg schedules ordered FL, FR, RL, RR.
    pub legs: [UnitreeGo2LegSchedule; 4],
}

impl UnitreeGo2GaitSchedule {
    /// The diagonal-pair trot: reproduces [`unitree_go2_trot_targets`] exactly.
    pub const TROT: Self = Self {
        legs: [
            UnitreeGo2LegSchedule::trot(0.0),
            UnitreeGo2LegSchedule::trot(0.5),
            UnitreeGo2LegSchedule::trot(0.5),
            UnitreeGo2LegSchedule::trot(0.0),
        ],
    };

    /// The best schedule found with the duty range opened into the aerial
    /// region (seed 42, `examples/55_go2_stepped_turn -- --train-aerial`) —
    /// pinned as a NEGATIVE result.
    ///
    /// Hypothesis: flight phases (duty < 0.5) change the contact mechanics
    /// enough to unlock the steering the walkable-schedule search could not
    /// find. Given duty freedom down to 0.30, the search *declined it*: every
    /// winning leg settles at duty ≥ 0.52, and the turn (~0.014 rad/s)
    /// matches the walkable-schedule plateau instead of beating the overlay.
    /// `aerial_duty_freedom_is_declined_by_the_search` pins both facts.
    pub const LEARNED_AERIAL_TURN: Self = Self {
        legs: [
            UnitreeGo2LegSchedule {
                phase_offset: 0.062048197426,
                duty: 0.570963923682,
                stride_scale: 1.146743054174,
                hip_offset_rad: 0.279447872946,
                hip_stance_sweep_rad: 0.153158270918,
            },
            UnitreeGo2LegSchedule {
                phase_offset: 0.553895383458,
                duty: 0.726038444645,
                stride_scale: 0.783766212154,
                hip_offset_rad: -0.012107707832,
                hip_stance_sweep_rad: 0.024374181638,
            },
            UnitreeGo2LegSchedule {
                phase_offset: 0.518305094098,
                duty: 0.523228125717,
                stride_scale: 1.319286988452,
                hip_offset_rad: -0.186675330533,
                hip_stance_sweep_rad: 0.015823524804,
            },
            UnitreeGo2LegSchedule {
                phase_offset: 0.937095795797,
                duty: 0.688457233373,
                stride_scale: 1.043668205007,
                hip_offset_rad: -0.224230297737,
                hip_stance_sweep_rad: 0.157045850775,
            },
        ],
    };

    /// The best turning contact schedule found by deterministic cross-entropy
    /// search (seed 42, `examples/55_go2_stepped_turn -- --train`).
    ///
    /// This is a *negative-result pin*: the schedule sustains a turn
    /// (~0.015 rad/s, positive through disjoint measurement windows) but does
    /// **not** beat the fixed-schedule joint-offset overlay
    /// ([`UnitreeGo2GaitOverlay::LEARNED_TURN`], ~0.025 rad/s) — refuting the
    /// hypothesis that re-sequencing contacts within walkable schedules
    /// unlocks steering, and, together with the torque-limit scan in
    /// `docs/GO2_LOCOMOTION.md`, pointing at contact friction and morphology
    /// as the plateau's cause.
    pub const LEARNED_TURN: Self = Self {
        legs: [
            UnitreeGo2LegSchedule {
                phase_offset: 0.074632,
                duty: 0.550000,
                stride_scale: 1.351674,
                hip_offset_rad: 0.171945,
                hip_stance_sweep_rad: 0.128867,
            },
            UnitreeGo2LegSchedule {
                phase_offset: 0.567315,
                duty: 0.721690,
                stride_scale: 0.755357,
                hip_offset_rad: 0.033470,
                hip_stance_sweep_rad: 0.053825,
            },
            UnitreeGo2LegSchedule {
                phase_offset: 0.526908,
                duty: 0.627953,
                stride_scale: 0.897632,
                hip_offset_rad: -0.296140,
                hip_stance_sweep_rad: -0.024478,
            },
            UnitreeGo2LegSchedule {
                phase_offset: 0.983421,
                duty: 0.709291,
                stride_scale: 1.059287,
                hip_offset_rad: -0.254034,
                hip_stance_sweep_rad: -0.031375,
            },
        ],
    };
}

/// Force-limited targets for all 12 joints under an explicit contact schedule.
pub fn unitree_go2_scheduled_targets(
    step: u64,
    command: UnitreeGo2GaitCommand,
    schedule: &UnitreeGo2GaitSchedule,
) -> [UrdfJointPositionTarget<'static>; 12] {
    let stride = command.stride_rad.clamp(0.0, 0.3);
    let lift = command.foot_lift_rad.clamp(0.0, 0.4);
    let cycle = command.cycle_steps.clamp(40, 180);
    let roll = command.roll_correction_rad.clamp(-0.8, 0.8);
    let pitch = command.pitch_correction_rad.clamp(-0.3, 0.3);
    let extension = command.lateral_extension_rad.clamp(-0.5, 0.5);
    let phase = (step % cycle) as f64 / cycle as f64;
    let names: [(&'static str, &'static str, &'static str, f64); 4] = [
        ("FL_hip", "FL_thigh", "FL_calf", 1.0),
        ("FR_hip", "FR_thigh", "FR_calf", -1.0),
        ("RL_hip", "RL_thigh", "RL_calf", 1.0),
        ("RR_hip", "RR_thigh", "RR_calf", -1.0),
    ];
    let mut targets = [target("FL_hip", 0.0); 12];
    for (leg_index, ((hip, thigh, calf, side), leg)) in
        names.iter().zip(schedule.legs.iter()).enumerate()
    {
        let leg_phase = (phase + leg.phase_offset.rem_euclid(1.0)).rem_euclid(1.0);
        let duty = leg.duty.clamp(0.3, 0.85);
        let wave = gait_wave_with_duty(leg_phase, duty);
        let leg_stride = stride * leg.stride_scale.clamp(0.3, 1.6);
        // Hip: base correction + placement offset + stance sweep. The sweep
        // follows the same stance ramp as the thigh, so the foot is planted at
        // `+sweep` and dragged to `-sweep` before lifting.
        let sweep = leg.hip_stance_sweep_rad.clamp(-0.35, 0.35);
        let hip_position = roll + leg.hip_offset_rad.clamp(-0.4, 0.4) + sweep * wave.0;
        targets[leg_index * 3] = target(hip, hip_position);
        targets[leg_index * 3 + 1] = target(thigh, 0.8 + leg_stride * wave.0 + pitch);
        targets[leg_index * 3 + 2] = target(calf, -1.5 - lift * wave.1 + side * extension);
    }
    targets
}

fn target(link_name: &'static str, position: f64) -> UrdfJointPositionTarget<'static> {
    UrdfJointPositionTarget {
        link_name,
        position,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn trot_schedule_reproduces_the_scripted_trot() {
        let commands = [
            UnitreeGo2GaitCommand::default(),
            UnitreeGo2GaitCommand {
                stride_rad: 0.24,
                cycle_steps: 45,
                roll_correction_rad: 0.2,
                lateral_extension_rad: -0.1,
                ..UnitreeGo2GaitCommand::default()
            },
        ];
        for command in commands {
            for step in [0, 13, 44, 89, 130] {
                assert_eq!(
                    unitree_go2_trot_targets(step, command),
                    unitree_go2_scheduled_targets(step, command, &UnitreeGo2GaitSchedule::TROT),
                    "step {step}"
                );
            }
        }
    }

    #[test]
    fn zero_overlay_reproduces_the_scripted_trot() {
        let command = UnitreeGo2GaitCommand::default();
        for step in [0, 17, 45, 89] {
            assert_eq!(
                unitree_go2_trot_targets(step, command),
                unitree_go2_trot_targets_with_overlay(step, command, &UnitreeGo2GaitOverlay::ZERO)
            );
        }
    }

    #[test]
    fn overlay_offsets_are_clamped() {
        let overlay = UnitreeGo2GaitOverlay {
            coefficients: [[1.0, 1.0, 1.0, 1.0, 1.0]; 12],
        };
        for offset in overlay.offsets(0.13) {
            assert!(offset.abs() <= 0.5);
        }
    }

    #[test]
    fn official_go2_trot_is_periodic_and_bounded() {
        let command = UnitreeGo2GaitCommand::default();
        assert_eq!(
            unitree_go2_trot_targets(0, command),
            unitree_go2_trot_targets(command.cycle_steps, command)
        );
        for target in unitree_go2_trot_targets(30, command) {
            assert!(target.position.is_finite());
            assert!(target.position.abs() <= 2.0);
        }
    }
}
