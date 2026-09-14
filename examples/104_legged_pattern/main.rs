//! Plans a deterministic straight-line walking pattern with the native legged
//! template layer and demonstrates capture-point push recovery.
//!
//! Everything is headless and backend-free: the LIPM/DCM/Gait primitives in
//! `rne_legged` turn a footstep request into a center-of-mass trajectory whose
//! Zero Moment Point tracks the reference.
//!
//! Run with `cargo run -p legged_pattern --example 104_legged_pattern`.

use rne_legged::{
    capture_point, footstep_from_dcm, plan_walking_pattern, propagate_constant_zmp, GaitSchedule,
    Horizontal, LimpParams, LimpState, StraightWalkRequest, ZmpPreviewController,
};

fn main() {
    let params = LimpParams::new(0.30, 9.81);
    let controller =
        ZmpPreviewController::new(params, 0.005, 1.0, 1.0e-7, 400).expect("preview controller");
    let request = StraightWalkRequest {
        direction: Horizontal::new(1.0, 0.0),
        start_com_m: Horizontal::ZERO,
        steps: 8,
        step_length_m: 0.20,
        step_width_m: 0.20,
        schedule: GaitSchedule::default(),
    };

    let pattern = plan_walking_pattern(&params, &controller, &request).expect("walking pattern");
    let start = pattern.com_m.first().copied().unwrap_or_default();
    let end = pattern.com_m.last().copied().unwrap_or_default();
    println!(
        "walk: footsteps={} duration={:.2}s samples={} com=({:.3}, {:.3}) -> ({:.3}, {:.3})",
        pattern.plan.footsteps.len(),
        pattern.duration_s(),
        pattern.sample_count(),
        start.x_m,
        start.z_m,
        end.x_m,
        end.z_m,
    );
    println!(
        "zmp tracking: overall={:.4} m steady(after 1s)={:.4} m",
        pattern.max_zmp_tracking_error_m(),
        pattern.max_zmp_tracking_error_after_m(1.0),
    );
    println!(
        "bounds: max_com_speed={:.3} m/s max_dcm_offset={:.3} m",
        pattern.max_com_speed_m_s(),
        pattern.max_dcm_offset_m(&params),
    );

    // A lateral push is arrested by stepping to the capture point.
    let disturbed = LimpState::new(Horizontal::ZERO, Horizontal::new(0.0, 0.40));
    let dcm = capture_point(&params, &disturbed);
    let capture_step = footstep_from_dcm(&params, dcm, dcm, 0.35);
    let (rest, _) = propagate_constant_zmp(&params, &disturbed, capture_step, 2.0);
    println!(
        "push recovery: dcm=({:.3}, {:.3}) step=({:.3}, {:.3}) rest_com=({:.3}, {:.3}) rest_speed={:.4}",
        dcm.x_m,
        dcm.z_m,
        capture_step.x_m,
        capture_step.z_m,
        rest.com_m.x_m,
        rest.com_m.z_m,
        rest.com_velocity_m_s.norm(),
    );

    // The three-stage pipeline is deterministic.
    let again = plan_walking_pattern(&params, &controller, &request).expect("walking pattern");
    let deterministic = pattern.com_m == again.com_m && pattern.zmp_m == again.zmp_m;

    let finite = pattern.com_m.iter().all(|value| value.is_finite())
        && pattern.zmp_m.iter().all(|value| value.is_finite());
    let recovered =
        (rest.com_m - capture_step).norm() < 0.01 && rest.com_velocity_m_s.norm() < 0.02;
    let tracked = pattern.max_zmp_tracking_error_after_m(1.0) < 0.02;
    let advanced = end.x_m > start.x_m + 0.7;
    let walked = deterministic && finite && tracked && advanced && recovered;

    println!("deterministic={deterministic} finite={finite} tracked={tracked} advanced={advanced} recovered={recovered}");
    if !walked {
        eprintln!("legged pattern diagnostics failed");
        std::process::exit(1);
    }
    println!("legged pattern diagnostics: ok");
}
