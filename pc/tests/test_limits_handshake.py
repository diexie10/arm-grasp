# -*- coding: utf-8 -*-
"""test_limits_handshake.py — L-command handshake parser and integration tests."""
import pytest

import config
from arm_serial import ArmSerial


class TestDryRunQueryLimits:
    """ArmSerial(dry_run=True).query_limits() returns config values."""

    def test_query_returns_config(self):
        arm = ArmSerial(dry_run=True)
        result = arm.query_limits()
        assert result is not None
        assert result["min"] == [int(v) for v in config.SERVO_MIN]
        assert result["max"] == [int(v) for v in config.SERVO_MAX]
        assert result["home"] == [int(v) for v in config.HOME_SERVO]


class TestParserGoodString:
    """Unit test the parser on a well-formed L reply string."""

    def test_parse_ok(self):
        arm = ArmSerial(dry_run=True)
        # Patch _send to return a known good string
        arm._send = lambda cmd, timeout=None: (
            "OK L MIN 11 3 5 7 0 0 MAX 191 183 167 187 270 270 HOME 101 122 167 97 90 90"
        )
        result = arm.query_limits()
        assert result is not None
        assert result["min"] == [11, 3, 5, 7, 0, 0]
        assert result["max"] == [191, 183, 167, 187, 270, 270]
        assert result["home"] == [101, 122, 167, 97, 90, 90]


class TestParserMalformed:
    """Parser returns None on malformed / ERR strings."""

    def test_err_response(self):
        arm = ArmSerial(dry_run=True)
        arm._send = lambda cmd, timeout=None: "ERR unknown"
        assert arm.query_limits() is None

    def test_malformed_format(self):
        arm = ArmSerial(dry_run=True)
        arm._send = lambda cmd, timeout=None: "OK L MIN 11 3"
        assert arm.query_limits() is None

    def test_non_numeric(self):
        arm = ArmSerial(dry_run=True)
        arm._send = lambda cmd, timeout=None: "OK L MIN abc def ghi jkl mno pqr MAX 1 2 3 4 5 6 HOME 1 2 3 4 5 6"
        assert arm.query_limits() is None

    def test_none_response(self):
        arm = ArmSerial(dry_run=True)
        arm._send = lambda cmd, timeout=None: None
        assert arm.query_limits() is None


class TestMismatchRaisesSystemExit:
    """Mismatch between firmware and PC config raises SystemExit."""

    def test_mismatch_detected(self):
        arm = ArmSerial(dry_run=True)
        # Patch query_limits to return a wrong value
        original = arm.query_limits
        arm.query_limits = lambda: {
            "min": [99, 3, 5, 7, 0, 0],  # joint 0 wrong
            "max": [int(v) for v in config.SERVO_MAX],
            "home": [int(v) for v in config.HOME_SERVO],
        }
        with pytest.raises(SystemExit, match="固件与 PC config 限位不一致"):
            arm.verify_limits()

    def test_max_mismatch(self):
        arm = ArmSerial(dry_run=True)
        arm.query_limits = lambda: {
            "min": [int(v) for v in config.SERVO_MIN],
            "max": [999, 183, 167, 187, 270, 270],  # joint 0 max wrong
            "home": [int(v) for v in config.HOME_SERVO],
        }
        with pytest.raises(SystemExit, match="固件与 PC config 限位不一致"):
            arm.verify_limits()
