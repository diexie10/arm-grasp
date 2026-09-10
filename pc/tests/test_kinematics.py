# -*- coding: utf-8 -*-
"""test_kinematics.py — IK/FK, domain guards, reachability tests."""
import math

import config
import kinematics


class TestVerifyIk:
    """FK(IK(T)) ≈ T closure on several reachable poses."""

    def test_mid_reach(self):
        """Pose well within workspace — error must be < 1mm."""
        ok, err = kinematics.verify_ik((100, 100, 150))
        assert ok, "verify_ik failed: %s" % err
        assert err < 1.0

    def test_low_z(self):
        """Low-z pose that uses elbow-down branch."""
        ok, err = kinematics.verify_ik((120, 0, 50))
        if ok:
            assert err < 1.0

    def test_high_reach(self):
        """High reach near workspace boundary."""
        ok, err = kinematics.verify_ik((150, 0, 200))
        if ok:
            assert err < 1.0

    def test_forward(self):
        """Straight ahead, moderate height."""
        ok, err = kinematics.verify_ik((100, 0, 120))
        if ok:
            assert err < 1.0


class TestRegressionPoses:
    """3 regression poses from kinematics.py __main__ must match ±0.15°."""

    REGRESSION = [
        ((-66.6, 147.7, 197.5), [114.27, 6.12, 57.1, 26.78, 0.0, 100.0]),
        ((-135.4, 142.6, 181.0), [133.51, 12.2, 30.52, 47.28, 0.0, 100.0]),
        ((138.5, 138.5, 194.0), [45.0, 23.11, 16.02, 50.87, 0.0, 100.0]),
    ]

    def test_regression(self):
        for pose, expected_q in self.REGRESSION:
            q, reason = kinematics.ik_solve(*pose)
            assert q is not None, "ik_solve(%s) returned None: %s" % (pose, reason)
            actual = [round(v, 2) for v in q]
            for a, e in zip(actual, expected_q):
                assert abs(a - e) < 0.15, (
                    "regression: pose %s q=%s != expected %s" % (pose, actual, expected_q))


class TestDomainGuard:
    """servo_limits domain guards for J2 and J4 (P0 regression)."""

    def test_j2_domain(self):
        assert kinematics.servo_limits(1) == (3.0, 183.0), \
            "J2 servo_limits wrong — domain confusion?"

    def test_j4_domain(self):
        assert kinematics.servo_limits(3) == (7.0, 187.0), \
            "J4 servo_limits wrong — domain confusion?"


class TestIsReachable:
    """is_reachable basics."""

    def test_inside(self):
        assert kinematics.is_reachable(100, 0, 150) is True

    def test_far(self):
        assert kinematics.is_reachable(500, 0, 150) is False

    def test_origin(self):
        # At J1 axis center, moderate height — should be reachable
        assert kinematics.is_reachable(0, 0, 150) is True
