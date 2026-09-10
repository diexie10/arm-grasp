#!/usr/bin/env python3
"""test_mahony_parity.py — Prove C++ Mahony (mahony.h) matches Python exactly.

Compiles the REAL src/mahony.h via a tiny host C++ harness using clang++,
runs the same synthetic gyro/accel sequence, and reports max |diff| in
degrees.  Requires < 1e-3 deg match.
"""

import math
import os
import subprocess
import sys
import tempfile
import types

# ── Locate project paths ──────────────────────────────────────────────
TEST_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(TEST_DIR)
SRC_DIR = os.path.join(PROJECT_DIR, "src")
PC_DIR = os.path.join(os.path.dirname(PROJECT_DIR), "pc")

# Stub config module so imu_filter can import it
_config_mod = types.ModuleType("config")
_config_mod.GLOVE_MAHONY_KP = 0.5
_config_mod.GLOVE_MAHONY_KI = 0.0
sys.modules["config"] = _config_mod

sys.path.insert(0, PC_DIR)
from glove.imu_filter import MahonyFilter  # noqa: E402


# ── Synthetic sequence ────────────────────────────────────────────────

def generate_sequence(n_steps, dt):
    """Deterministic gyro/accel: gentle sinusoidal swing."""
    gyro, accel = [], []
    for i in range(n_steps):
        t = i * dt
        gx = 0.5 * math.sin(2.0 * math.pi * 0.3 * t)
        gy = 0.3 * math.sin(2.0 * math.pi * 0.2 * t)
        gz = 0.1 * math.sin(2.0 * math.pi * 0.1 * t)
        gyro.append((gx, gy, gz))

        roll_t = 0.3 * math.sin(2.0 * math.pi * 0.15 * t)
        pitch_t = 0.2 * math.sin(2.0 * math.pi * 0.25 * t)
        ax = -9.80665 * math.sin(pitch_t)
        ay = 9.80665 * math.sin(roll_t) * math.cos(pitch_t)
        az = 9.80665 * math.cos(roll_t) * math.cos(pitch_t)
        accel.append((ax, ay, az))
    return gyro, accel


# ── Python reference ──────────────────────────────────────────────────

def run_python(gyro, accel, dt):
    f = MahonyFilter()
    results = []
    for gx, gy, gz in gyro:
        ax, ay, az = accel[len(results)]
        roll, pitch, yaw = f.update((gx, gy, gz), (ax, ay, az), dt)
        results.append((roll, pitch, yaw))
    return results


# ── Compile & run the REAL mahony.h ───────────────────────────────────

HARNESS_CPP = r"""
#define _CRT_SECURE_NO_WARNINGS
#include "mahony.h"
#include <cstdio>
#include <cstdlib>

int main(int argc, char **argv) {
    if (argc < 3) { fprintf(stderr, "usage: mahony_test INPUT.TXT DT N\n"); return 1; }
    const char *inpath = argv[1];
    float dt = (float)atof(argv[2]);
    int n = atoi(argv[3]);

    FILE *fin = fopen(inpath, "r");
    if (!fin) { fprintf(stderr, "cannot open %s\n", inpath); return 1; }

    float kp = 0.5f, ki = 0.0f;
    MahonyState s;
    mahony_init(&s);

    printf("step,roll,pitch,yaw\n");
    for (int i = 0; i < n; i++) {
        float gx, gy, gz, ax, ay, az;
        if (fscanf(fin, "%f %f %f %f %f %f", &gx, &gy, &gz, &ax, &ay, &az) != 6) {
            fprintf(stderr, "bad input at step %d\n", i);
            fclose(fin);
            return 1;
        }
        float roll, pitch, yaw;
        mahony_update(&s, gx, gy, gz, ax, ay, az, dt, kp, ki,
                      &roll, &pitch, &yaw);
        printf("%d %.6f %.6f %.6f\n", i, roll, pitch, yaw);
    }
    fclose(fin);
    return 0;
}
"""


def run_c(gyro, accel, dt):
    """Compile mahony.h via clang++ and run the harness."""
    # Write harness
    harness_path = os.path.join(tempfile.gettempdir(), "mahony_harness.cpp")
    with open(harness_path, "w", encoding="utf-8", newline="\n") as f:
        f.write(HARNESS_CPP)

    obj_path = os.path.join(tempfile.gettempdir(), "mahony_harness.obj")
    exe_path = os.path.join(tempfile.gettempdir(), "mahony_harness")
    if sys.platform == "win32":
        obj_path = os.path.join(tempfile.gettempdir(), "mahony_harness.obj")
        exe_path += ".exe"

    # Step 1: compile to .obj (clang++ -c)
    compile_cmd = [
        "clang++", "-c", "-O2", "-std=c++14",
        "-I", SRC_DIR,
        "-D_CRT_SECURE_NO_WARNINGS",
        "-Wno-deprecated-declarations",
        "-o", obj_path,
        harness_path,
    ]
    print("  compile:", " ".join(compile_cmd))
    r = subprocess.run(compile_cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print("  COMPILE FAILED:\n" + r.stderr)
        return None

    # Step 2: link — use clang++ driver (it delegates to MSVC link.exe)
    link_cmd = [
        "clang++",
        "-o", exe_path,
        obj_path,
    ]
    print("  link:   ", " ".join(link_cmd))
    r = subprocess.run(link_cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print("  LINK FAILED:\n" + r.stderr)
        return None

    # Build input file: one line per sample "gx gy gz ax ay az"
    input_path = os.path.join(tempfile.gettempdir(), "mahony_input.txt")
    with open(input_path, "w") as f:
        for i in range(len(gyro)):
            gx, gy, gz = gyro[i]
            ax, ay, az = accel[i]
            f.write(f"{gx} {gy} {gz} {ax} {ay} {az}\n")

    r = subprocess.run(
        [exe_path, input_path, str(dt), str(len(gyro))],
        capture_output=True, text=True,
    )
    if r.returncode != 0:
        print("  RUN FAILED:\n" + r.stderr)
        return None

    results = []
    for line in r.stdout.strip().split("\n")[1:]:  # skip header
        parts = line.split()
        results.append((float(parts[1]), float(parts[2]), float(parts[3])))
    return results


# ── Main ──────────────────────────────────────────────────────────────

def main():
    dt = 0.01
    n_steps = 500
    print(f"Mahony parity: Python vs real mahony.h (clang++)")
    print(f"  {n_steps} steps, dt={dt}s, KP=0.5, KI=0.0")
    print()

    gyro, accel = generate_sequence(n_steps, dt)

    py = run_python(gyro, accel, dt)
    print("  Python reference: {} steps".format(len(py)))

    cc = run_c(gyro, accel, dt)
    if cc is None:
        print("\nFAILED — could not compile or run C++ harness.")
        sys.exit(1)
    print("  C++ harness:      {} steps".format(len(cc)))

    if len(cc) != len(py):
        print(f"\nFAILED — length mismatch: C++={len(cc)}, Python={len(py)}")
        sys.exit(1)

    max_diff = [0.0, 0.0, 0.0]
    for i, (p, c) in enumerate(zip(py, cc)):
        for axis in range(3):
            d = abs(p[axis] - c[axis])
            if d > max_diff[axis]:
                max_diff[axis] = d

    names = ["roll", "pitch", "yaw"]
    print("\nMax absolute difference (degrees):")
    for axis in range(3):
        print(f"  {names[axis]:6s}: {max_diff[axis]:.6e}")
    overall = max(max_diff)
    print(f"  OVERALL: {overall:.6e}")

    threshold = 1e-3
    if overall < threshold:
        print(f"\nPASS — max diff {overall:.2e} < {threshold}")
        return 0
    else:
        print(f"\nFAIL — max diff {overall:.2e} >= {threshold}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
