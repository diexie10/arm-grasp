# -*- coding: utf-8 -*-
"""test_servo_controller.py — ServoController unit tests (pure math, no hardware)."""
import config
from servo_controller import ServoController, estimate_move_seconds, compute_wrist_correction


class TestAlignDelta:
    """align_delta clamps to ±max_step."""

    def test_small_error(self):
        sc = ServoController()
        dq0, dq12 = sc.align_delta(10, 10, max_step=5.0)
        assert abs(dq0) <= 5.0
        assert abs(dq12) <= 5.0

    def test_large_error_clamped(self):
        sc = ServoController()
        dq0, dq12 = sc.align_delta(999, -999, max_step=5.0)
        assert abs(dq0) <= 5.0 + 1e-9
        assert abs(dq12) <= 5.0 + 1e-9


class TestApplyJointDeltas:
    """apply_joint_deltas enforces J4 = -2·dq12 (when dq12 != 0)."""

    def test_j4_constraint(self):
        sc = ServoController()
        q0 = [0.0, 0.0, 0.0, 0.0, 0.0, 90.0]
        q_new = sc.apply_joint_deltas(q0, dq0=1.0, dq12=3.0)
        assert q_new[0] == 1.0
        assert q_new[1] == 3.0
        assert q_new[2] == 3.0
        assert q_new[3] == -6.0  # -2 * 3

    def test_j4_unchanged_when_dq12_zero(self):
        sc = ServoController()
        q0 = [0.0, 0.0, 0.0, 42.0, 0.0, 90.0]
        q_new = sc.apply_joint_deltas(q0, dq0=5.0, dq12=0.0)
        assert q_new[3] == 42.0  # unchanged


class TestComputeWristCorrection:
    """compute_wrist_correction quantizes to WRIST_QUANTUM_DEG and clamps to J4_SLACK_DEG."""

    def test_quantization(self):
        q_current = [0.0, 0.0, 0.0, 0.0, 0.0, 90.0]
        dq4, q4_target = compute_wrist_correction(10.0, q_current, center=0.0)
        if abs(dq4) > 1e-6:
            assert abs(dq4) % config.WRIST_QUANTUM_DEG < 1e-6 or \
                   abs(abs(dq4) % config.WRIST_QUANTUM_DEG - config.WRIST_QUANTUM_DEG) < 1e-6, \
                "dq4 not quantized to WRIST_QUANTUM_DEG"

    def test_slack_clamp(self):
        q_current = [0.0, 0.0, 0.0, 0.0, 0.0, 90.0]
        # Large error should be clamped to ±J4_SLACK_DEG from center
        dq4, q4_target = compute_wrist_correction(1000.0, q_current, center=0.0)
        assert abs(q4_target) <= config.J4_SLACK_DEG + 1e-6

    def test_no_correction_when_zero(self):
        q_current = [0.0, 0.0, 0.0, 5.0, 0.0, 90.0]
        dq4, q4_target = compute_wrist_correction(0.0, q_current, center=5.0)
        assert abs(dq4) < 1e-6
        assert q4_target == 5.0


class TestEstimateMoveSeconds:
    """estimate_move_seconds ≥ 0 and monotonic in displacement."""

    def test_non_negative(self):
        q_home = list(config.JOINT_HOME)
        t = estimate_move_seconds(q_home, q_home)
        assert t >= 0.0

    def test_monotonic(self):
        q_home = list(config.JOINT_HOME)
        # Small displacement
        q_small = list(q_home)
        q_small[0] += 5.0
        t_small = estimate_move_seconds(q_home, q_small)
        # Large displacement
        q_large = list(q_home)
        q_large[0] += 50.0
        t_large = estimate_move_seconds(q_home, q_large)
        assert t_large > t_small, "estimate_move_seconds not monotonic"
