# -*- coding: utf-8 -*-
"""test_imu_filter.py — Mahony AHRS filter tests (static level input)."""
from glove.imu_filter import MahonyFilter


class TestMahonyStaticLevel:
    """Static level input → roll & pitch < 0.5° after convergence."""

    def _run_static(self, steps=200):
        f = MahonyFilter()
        f.reset()
        for _ in range(steps):
            r, p, y = f.update(
                gyro_rad_s=[0.0, 0.0, 0.0],
                accel_m_s2=[0.0, 0.0, 9.81],
                dt=0.01)
        return r, p, y

    def test_converges(self):
        r, p, y = self._run_static(200)
        assert abs(r) < 0.5, "roll=%.3f > 0.5°" % r
        assert abs(p) < 0.5, "pitch=%.3f > 0.5°" % p

    def test_deterministic(self):
        """Two runs with same inputs must produce identical results."""
        r1, p1, y1 = self._run_static(200)
        r2, p2, y2 = self._run_static(200)
        assert r1 == r2 and p1 == p2 and y1 == y2, \
            "Non-deterministic: run1=(%.3f,%.3f,%.3f) run2=(%.3f,%.3f,%.3f)" % (r1,p1,y1,r2,p2,y2)
