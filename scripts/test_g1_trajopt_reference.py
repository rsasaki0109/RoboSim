"""Checks for the optional external G1 benchmark (run in its conda environment)."""

import argparse
import unittest

import g1_trajopt_flight_check as flight_check
import g1_trajopt_reference as reference
import numpy as np
import pinocchio as pin


class ReferenceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.api = reference.load_upstream(
            reference.ROOT / "target/research/se3_trajopt"
        )

    def arguments(self):
        return argparse.Namespace(
            contacts="sole",
            mass_policy="rne",
            integration="body-euler",
            task="stand",
            dt_s=0.03,
            push_steps=2,
            flight_steps=2,
            landing_steps=2,
            tolerance=1e-4,
        )

    def test_independent_flight_integrator_converges_to_ballistic_motion(self):
        model = pin.Model()
        joint = model.addJoint(0, pin.JointModelFreeFlyer(), pin.SE3.Identity(), "root")
        model.appendBodyToJoint(
            joint, pin.Inertia(1.0, np.zeros(3), np.eye(3)), pin.SE3.Identity()
        )
        errors = []
        for steps in (60, 120):
            data = model.createData()
            q = pin.neutral(model)
            q[2] = 2.0
            v = np.array([1.0, 0.0, 3.0, 0.0, -10.0, 0.0])
            for _ in range(steps):
                q, v = flight_check.midpoint_step(
                    model, data, q, v, np.zeros(6), 0.6 / steps
                )
            expected = [0.6, 0.0, 2.0 + 3.0 * 0.6 - 0.5 * 9.81 * 0.6**2]
            errors.append(np.linalg.norm(q[:3] - expected))
        self.assertLess(errors[1], errors[0] / 3.0)
        self.assertLess(errors[1], 0.01)

    def test_pd_tracks_the_saved_flight_without_limit_violations(self):
        result = flight_check.check(
            reference.ROOT
            / "docs/evidence/g1-trajopt-reference/backflip-momentum-50ms",
            1e-5,
            80.0,
            4.0,
        )
        self.assertLess(result["final_joint_position_error_rad"], 0.1)
        self.assertLess(result["final_root_rotation_error_rad"], 0.1)
        self.assertEqual(result["max_joint_limit_violation_rad"], 0.0)
        self.assertEqual(result["saturated_steps"], 0)
        self.assertTrue(result["numerical_momentum_screen_passed"])
        self.assertFalse(result["plant_validated"])

    def test_grounded_model_and_limit_contract(self):
        model, q, names, frames, _ = reference.build_model("sole")
        self.assertEqual(len(names), 23)
        self.assertEqual(model.nv, 29)
        data = model.createData()
        pin.framesForwardKinematics(model, data, q)
        for frame in frames:
            self.assertAlmostEqual(
                data.oMf[model.getFrameId(frame)].translation[2], 0.0
            )
        np.testing.assert_allclose(
            model.effortLimit[6:], [reference.native_torque_limit(n) for n in names]
        )

    def test_invalid_values_fail_bounds(self):
        for value in (np.nan, np.inf, -np.inf):
            self.assertEqual(
                reference.bound_violation(np.array([value]), [None], [None]), np.inf
            )
        self.assertEqual(reference.bound_violation(np.array([2.0]), [-1.0], [1.0]), 1.0)

    def test_mass_policy_accounts_for_four_unmodelled_sensor_frames(self):
        native, *_ = reference.build_model("sole", "rne")
        declared, *_ = reference.build_model("sole", "declared")
        self.assertAlmostEqual(pin.computeTotalMass(native), 38.13385728)
        self.assertAlmostEqual(pin.computeTotalMass(declared), 34.13385728)

    def test_warm_start_preserves_last_push_force_at_contact_transition(self):
        args = self.arguments()
        args.task = "backflip"
        args.dt_s = 0.05
        args.push_steps = args.flight_steps = args.landing_steps = 12
        args.warm_start = reference.ROOT / "docs/evidence/g1-trajopt-reference/backflip"
        problem, model, _, names, _, _ = reference.build_problem(args, self.api)
        reference.apply_warm_start(problem, model, names, args, self.api)
        violation = reference.bound_violation(
            problem.constraints(problem.x0), problem.clb, problem.cub
        )
        self.assertLess(violation, 1e-4)

    def test_flight_conservation_detects_spurious_horizontal_impulse(self):
        records = [
            {"time_s": 0.0, "momentum_world": [1.0, 0.0, 2.0, 0.0, -3.0, 0.0]},
            {"time_s": 0.1, "momentum_world": [1.0, 0.0, 2.0 - 0.981, 0.0, -3.0, 0.0]},
        ]
        self.assertTrue(
            reference.flight_momentum_errors(records, 1.0)[
                "flight_momentum_screen_passed"
            ]
        )
        records[-1]["momentum_world"][0] += 1.0
        self.assertFalse(
            reference.flight_momentum_errors(records, 1.0)[
                "flight_momentum_screen_passed"
            ]
        )

    def test_analytic_jacobian_matches_directional_difference(self):
        self.check_jacobian("body-euler")

    def test_world_integration_jacobian_matches_directional_difference(self):
        self.check_jacobian("world")

    def test_momentum_integration_jacobian_matches_directional_difference(self):
        self.check_jacobian("momentum")

    def test_refined_warm_start_can_preserve_nodal_dynamics(self):
        args = self.arguments()
        args.task = "backflip"
        args.integration = "momentum"
        args.push_steps = args.flight_steps = args.landing_steps = 20
        args.project_warm_start_dynamics = True
        args.warm_start = (
            reference.ROOT / "docs/evidence/g1-trajopt-reference/backflip-momentum-50ms"
        )
        problem, model, q, names, frames, _ = reference.build_problem(args, self.api)
        reference.apply_warm_start(problem, model, names, args, self.api)
        errors, _, _ = reference.audit(
            problem, model, q, frames, problem.x0, args, self.api
        )
        self.assertLess(errors["base_force_n"], 1e-8)
        self.assertLess(errors["base_torque_nm"], 1e-8)
        self.assertLess(errors["torque_limit_nm"], 1e-8)
        self.assertGreater(max(errors.values()), args.tolerance)

    def test_momentum_integration_accepts_ballistic_articulated_motion(self):
        model, q, *_ = reference.build_model("sole")
        data = model.createData()
        mass = pin.computeTotalMass(model)
        v = np.linspace(-0.3, 0.3, model.nv)
        v[4] = -10.0
        dt = 0.03
        gravity = np.array([0.0, 0.0, -9.81])
        a = pin.aba(model, data, q, v, np.zeros(model.nv))
        h = pin.computeCentroidalMomentum(model, data, q, v).vector.copy()
        com = pin.centerOfMass(model, data, q).copy()
        qn, _ = reference.world_step(model, q, v, a, dt)
        # Enforce the independent ballistic CoM path by translating the root.
        target_com = com + dt * h[:3] / mass + 0.5 * dt**2 * gravity
        qn[:3] += target_com - pin.centerOfMass(model, data, qn)
        target_h = h.copy()
        target_h[:3] += dt * mass * gravity
        centroidal_map = pin.computeCentroidalMap(model, data, qn).copy()
        vn = v + dt * a
        vn[:6] = np.linalg.solve(
            centroidal_map[:, :6], target_h - centroidal_map[:, 6:] @ vn[6:]
        )
        residual = reference.momentum_residual(model, data, q, v, a, qn, vn, dt, {})
        np.testing.assert_allclose(residual, 0.0, atol=1e-9)
        # An unforced horizontal kick must violate impulse balance.
        vn[0] += 1.0
        invalid = reference.momentum_residual(model, data, q, v, a, qn, vn, dt, {})
        self.assertGreater(np.linalg.norm(invalid[model.nv : model.nv + 3]), 10.0)

    def check_jacobian(self, integration):
        args = self.arguments()
        args.integration = integration
        problem, *_ = reference.build_problem(args, self.api)
        rng = np.random.default_rng(729)
        x = problem.x0 + rng.normal(size=problem.vars_dim) * 1e-3
        direction = rng.normal(size=problem.vars_dim)
        epsilon = 1e-6
        finite_difference = (
            problem.constraints(x + epsilon * direction)
            - problem.constraints(x - epsilon * direction)
        ) / (2 * epsilon)
        rows, cols = problem.jacobianstructure()
        analytic = np.zeros(problem.cons_dim)
        np.add.at(analytic, rows, problem.jacobian(x) * direction[cols])
        np.testing.assert_allclose(analytic, finite_difference, rtol=2e-5, atol=2e-5)

    def test_world_step_preserves_ballistic_motion_of_a_spinning_body(self):
        model = pin.Model()
        joint = model.addJoint(0, pin.JointModelFreeFlyer(), pin.SE3.Identity(), "root")
        model.appendBodyToJoint(
            joint, pin.Inertia(1.0, np.zeros(3), np.eye(3)), pin.SE3.Identity()
        )
        data = model.createData()
        q = pin.neutral(model)
        q[2] = 2.0
        v = np.array([1.0, 0.0, 3.0, 0.0, -10.0, 0.0])
        dt = 0.05
        for _ in range(12):
            a = pin.aba(model, data, q, v, np.zeros(6))
            q, world_velocity = reference.world_step(model, q, v, a, dt)
            rotation = pin.Quaternion(q[3:7]).matrix()
            v = np.concatenate(
                (rotation.T @ world_velocity[:3], rotation.T @ world_velocity[3:6])
            )
        duration = 12 * dt
        np.testing.assert_allclose(
            q[:3],
            [duration, 0.0, 2.0 + 3.0 * duration - 0.5 * 9.81 * duration**2],
            atol=1e-10,
        )
        np.testing.assert_allclose(
            world_velocity,
            [1.0, 0.0, 3.0 - 9.81 * duration, 0.0, -10.0, 0.0],
            atol=1e-10,
        )

    def test_static_kinematic_reference_fails_dynamics_audit(self):
        args = self.arguments()
        args.feasibility_only = True
        problem, model, q, _, frames, _ = reference.build_problem(args, self.api)
        x = problem.x0.copy()
        # Kinematically perfect stationary poses with no support cannot hover.
        for node in problem.nodes:
            for force_slice in node.forces_ids.values():
                x[force_slice] = 0.0
        self.assertEqual(problem.objective(x), 0.0)
        errors, _, _ = reference.audit(problem, model, q, frames, x, args, self.api)
        self.assertGreater(errors["base_force_n"], 100.0)
        self.assertLess(errors["position_m"], 1e-10)

    def test_last_terminal_contact_cannot_pull_on_ground(self):
        problem, _, _, _, frames, _ = reference.build_problem(
            self.arguments(), self.api
        )
        terminal = problem.nodes[-1]
        x = problem.x0.copy()
        x[terminal.forces_ids[frames[-1]]] = [0.0, 0.0, -100.0]
        constraints = problem.constraints(x)
        normal_row = terminal.c_friction_ids[frames[-1]].start + 4
        self.assertLess(constraints[normal_row], -99.0)
        rows, _ = problem.jacobianstructure()
        self.assertIn(normal_row, rows)


if __name__ == "__main__":
    unittest.main()
