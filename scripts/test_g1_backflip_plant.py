"""Headless checks for the optional optimization/contact benchmark."""

import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import g1_backflip_plant as plant
import g1_backflip_render as render
import mujoco
import numpy as np
from g1_backflip_search import Campaign


class ContactPlantTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = json.loads((plant.MODEL_MANIFEST).read_text())
        cls.model = plant.build_model(cls.manifest, 0.001, 300, 10)

    def test_free_base_has_no_actuator(self):
        model = self.model
        self.assertEqual(model.nv, 29)
        self.assertEqual(model.nu, 23)
        self.assertEqual(model.jnt_type[0], mujoco.mjtJoint.mjJNT_FREE)
        self.assertTrue(np.all(model.actuator_trnid[:, 0] > 0))
        np.testing.assert_allclose(
            model.actuator_forcerange[:, 1], self.manifest["torque_limits_nm"]
        )
        self.assertAlmostEqual(sum(model.body_mass), 38.13385728, places=5)

    def test_missing_inertia_policy_is_explicit(self):
        declared = dict(self.manifest, mass_policy="declared")
        model = plant.build_model(declared, 0.001, 300, 10)
        self.assertAlmostEqual(
            sum(self.model.body_mass) - sum(model.body_mass), 4.0, places=5
        )
        with self.assertRaises(ValueError):
            plant.build_model(dict(self.manifest, mass_policy="typo"), 0.001, 300, 10)
        with self.assertRaises(ValueError):
            plant.build_model(self.manifest, float("nan"), 300, 10)

        with self.assertRaisesRegex(ValueError, "time constants"):
            plant.build_model(self.manifest, 0.01, 300, 10)
        with self.assertRaisesRegex(ValueError, "checksum"):
            plant.build_model(dict(self.manifest, urdf_sha256="wrong"), 0.001, 300, 10)

    def test_static_stand_is_deterministic_and_unforced(self):
        model = self.model
        states = []
        for _ in range(2):
            data = mujoco.MjData(model)
            q = np.array(self.manifest["initial_q_xyzw"])
            data.qpos[:3] = q[:3]
            data.qpos[3:7] = q[[6, 3, 4, 5]]
            for index, name in enumerate(self.manifest["joint_names"]):
                data.qpos[model.jnt_qposadr[model.joint(name).id]] = q[index + 7]
            data.ctrl[:] = q[7:]
            for _ in range(1000):
                mujoco.mj_step(model, data)
            self.assertGreater(data.qpos[2], 0.7)
            self.assertLess(np.linalg.norm(data.qvel[:6]), 0.1)
            self.assertFalse(np.any(data.xfrc_applied))
            self.assertFalse(np.any(data.qfrc_applied))
            self.assertFalse(any(w.number for w in data.warning))
            states.append(data.qpos.copy())
        np.testing.assert_array_equal(*states)

    def test_saved_backflip_passes_and_matches_recording(self):
        evidence = plant.ROOT / "docs/evidence/g1-contact-backflip"
        seed = json.loads((evidence / "candidate.json").read_text())
        expected = json.loads((evidence / "step-125us/summary.json").read_text())
        with tempfile.TemporaryDirectory() as temporary:
            campaign = Campaign(
                Path(temporary),
                dt_s=seed["dt_s"],
                mass_policy=seed["mass_policy"],
                joint_limit_time_constant_s=seed["joint_limit_time_constant_s"],
                joint_limit_margin_rad=seed["joint_limit_margin_rad"],
            )
            campaign.early_balance = seed["early_balance_gains"]
            campaign.balance = seed["balance_gains"]
            campaign.landing_gains = seed["landing_gains"]
            campaign.recovery_s = seed["recovery_s"]
            result, history = campaign.rollout(seed["parameters"], record=True)
        self.assertTrue(result["passed"], result)
        digest = hashlib.sha256((json.dumps(history) + "\n").encode()).hexdigest()
        self.assertEqual(digest, expected["rollout_sha256"])

    def test_failed_rollout_cannot_be_rendered_as_success(self):
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            (directory / "summary.json").write_text(
                json.dumps({"passed": False, "backend": "mujoco"})
            )
            with self.assertRaisesRegex(ValueError, "passing"):
                render.render(directory, directory / "false-success.gif")
            self.assertFalse((directory / "false-success.gif").exists())


if __name__ == "__main__":
    unittest.main()
