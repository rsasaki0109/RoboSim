use super::*;

/// Evaluates one DC motor from terminal voltage and completed rotor velocity.
///
/// With no inductance, current is the algebraic equivalent-circuit solution
/// `I = (V - k_e omega) / R`. With inductance, current advances by explicit Euler from
/// `dI/dt = (V - k_e omega - R I) / L` and is then limited. Shaft loss combines viscous
/// friction and a regularized Coulomb term: at standstill Coulomb friction cancels available
/// electromagnetic torque up to its declared magnitude rather than inventing motion.
pub fn evaluate_dc_motor(
    spec: DcMotorSpec,
    state: DcMotorState,
    command_voltage_v: f64,
    rotor_velocity_rad_s: f64,
    dt_s: f64,
) -> Result<DcMotorEvaluation, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !state.current_a.is_finite()
        || !command_voltage_v.is_finite()
        || !rotor_velocity_rad_s.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    if !dt_s.is_finite() || dt_s <= 0.0 {
        return Err(MobilityPlantEvaluationError::InvalidTimeStep);
    }

    let voltage_saturated = command_voltage_v.abs() > spec.supply_voltage_v;
    let limited_command_voltage_v =
        command_voltage_v.clamp(-spec.supply_voltage_v, spec.supply_voltage_v);
    let terminal_voltage_v = match spec.failure_mode {
        DcMotorFailureMode::Nominal => limited_command_voltage_v,
        DcMotorFailureMode::OpenCircuit | DcMotorFailureMode::ShortCircuit => 0.0,
    };
    let back_emf_v = spec.back_emf_constant_v_s_rad * rotor_velocity_rad_s;
    let unconstrained_current_a = match (spec.failure_mode, spec.inductance_h) {
        (DcMotorFailureMode::OpenCircuit, _) => 0.0,
        (_, Some(inductance_h)) => {
            state.current_a
                + (terminal_voltage_v - back_emf_v - spec.resistance_ohm * state.current_a)
                    / inductance_h
                    * dt_s
        }
        (_, None) => (terminal_voltage_v - back_emf_v) / spec.resistance_ohm,
    };
    let current_saturated = unconstrained_current_a.abs() > spec.current_limit_a;
    let current_a = unconstrained_current_a.clamp(-spec.current_limit_a, spec.current_limit_a);
    let electromagnetic_torque_nm = spec.torque_constant_nm_a * current_a;
    let viscous_loss_nm = spec.viscous_friction_nm_s_rad * rotor_velocity_rad_s;
    let coulomb_loss_nm = if rotor_velocity_rad_s.abs() > 1.0e-12 {
        spec.coulomb_friction_nm * rotor_velocity_rad_s.signum()
    } else {
        electromagnetic_torque_nm.clamp(-spec.coulomb_friction_nm, spec.coulomb_friction_nm)
    };
    let shaft_loss_torque_nm = viscous_loss_nm + coulomb_loss_nm;

    Ok(DcMotorEvaluation {
        state: DcMotorState { current_a },
        terminal_voltage_v,
        back_emf_v,
        electromagnetic_torque_nm,
        shaft_loss_torque_nm,
        shaft_torque_nm: electromagnetic_torque_nm - shaft_loss_torque_nm,
        voltage_saturated,
        current_saturated,
    })
}

/// Maps motor torque and rotor inertia to a wheel coordinate without backend types.
///
/// This is the rigid static map. Declared backlash and compliance require a later stateful
/// driveline evaluator and are intentionally not approximated by hidden backend joints.
pub fn evaluate_transmission(
    spec: TransmissionSpec,
    motor_rotor_inertia_kg_m2: f64,
    motor_torque_nm: f64,
    wheel_velocity_rad_s: f64,
) -> Result<TransmissionEvaluation, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !motor_rotor_inertia_kg_m2.is_finite()
        || motor_rotor_inertia_kg_m2 < 0.0
        || !motor_torque_nm.is_finite()
        || !wheel_velocity_rad_s.is_finite()
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    let ratio = spec.ratio_motor_rad_per_wheel_rad;
    let motor_velocity_rad_s = wheel_velocity_rad_s * ratio;
    let applied_efficiency_ratio = if motor_torque_nm * motor_velocity_rad_s >= 0.0 {
        spec.drive_efficiency_ratio
    } else {
        spec.backdrive_efficiency_ratio
    };
    Ok(TransmissionEvaluation {
        motor_velocity_rad_s,
        wheel_torque_nm: motor_torque_nm * ratio * applied_efficiency_ratio,
        reflected_rotor_inertia_kg_m2: motor_rotor_inertia_kg_m2 * ratio * ratio,
        applied_efficiency_ratio,
    })
}

/// Returns wheel rolling-resistance torque opposing completed wheel motion.
///
/// The v1 law is `Crr * normal_load * radius` and returns zero at exact standstill so it
/// cannot create a direction. A later wheel/ground solver may use impending slip to model
/// static rolling resistance.
pub fn wheel_rolling_resistance_torque_nm(
    spec: WheelAssemblySpec,
    normal_load_n: f64,
    wheel_velocity_rad_s: f64,
) -> Result<f64, MobilityPlantEvaluationError> {
    if !spec.is_valid() {
        return Err(MobilityPlantEvaluationError::InvalidSpec);
    }
    if !normal_load_n.is_finite() || normal_load_n < 0.0 || !wheel_velocity_rad_s.is_finite() {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    Ok(if wheel_velocity_rad_s == 0.0 {
        0.0
    } else {
        -wheel_velocity_rad_s.signum()
            * spec.rolling_resistance_coefficient
            * normal_load_n
            * spec.radius_m
    })
}

/// Bounds Coulomb rolling resistance so one explicit step can stop, but not reverse, a wheel.
pub(crate) fn bounded_rolling_resistance_torque_nm(
    spec: WheelAssemblySpec,
    normal_load_n: f64,
    wheel_velocity_before_rolling_rad_s: f64,
    total_wheel_inertia_kg_m2: f64,
    dt_s: f64,
) -> Result<f64, MobilityPlantEvaluationError> {
    if !total_wheel_inertia_kg_m2.is_finite()
        || total_wheel_inertia_kg_m2 <= 0.0
        || !dt_s.is_finite()
        || dt_s <= 0.0
    {
        return Err(MobilityPlantEvaluationError::InvalidInput);
    }
    let unconstrained = wheel_rolling_resistance_torque_nm(
        spec,
        normal_load_n,
        wheel_velocity_before_rolling_rad_s,
    )?;
    let stopping_torque_nm =
        wheel_velocity_before_rolling_rad_s.abs() * total_wheel_inertia_kg_m2 / dt_s;
    Ok(unconstrained.signum() * unconstrained.abs().min(stopping_torque_nm))
}

/// Result of applying one actuator command.
#[derive(Clone, Debug, PartialEq)]
pub enum CommandApplyResult {
    /// Command applied successfully.
    Applied,
    /// Command rejected because the target entity was invalid.
    InvalidTarget,
    /// Command rejected because the joint validation failed.
    JointRejected(JointValidationError),
    /// Command ignored because it was stale.
    Stale,
}

/// Result of commanding a kinematic Ackermann drive.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AckermannCommandResult {
    /// The finite command was clamped to the drive limits and applied.
    Applied,
    /// The target entity has no valid [`AckermannDrive`].
    InvalidTarget,
    /// At least one command value was non-finite; the previous target was preserved.
    NonFiniteCommand,
}

/// Result of commanding a multirotor position target.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MultirotorCommandResult {
    /// The finite position and heading target was applied.
    Applied,
    /// The target entity has no valid [`MultirotorFlight`].
    InvalidTarget,
    /// At least one command value was non-finite; the previous target was preserved.
    NonFiniteCommand,
}
