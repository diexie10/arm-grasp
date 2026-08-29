#!/usr/bin/env python3
"""Independent baud-rate auditor for arm-grasp.
Re-derives EVERY BRR/BaudRate constant from the RM0008 datasheet formula,
never trusting comments or previous claims.

Formula (oversampling by 16):
    USARTDIV = Mantissa + Fraction/16      (Mantissa = BRR[15:4], Fraction = BRR[3:0])
    baud     = fCK / (16 * USARTDIV)
Bus clocks (from firmware clock tree, HSE 8MHz x PLL9 = 72MHz SYSCLK):
    USART1 -> APB2 = 72 MHz
    USART2 -> APB1 = 36 MHz
    USART3 -> APB1 = 36 MHz
"""
import os, re

ROOT = r"C:\Users\diexie\Desktop\arm-grasp"
STD_BAUDS = [7200, 9600, 14400, 19200, 28800, 38400, 57600,
             76800, 115200, 230400, 460800, 921600]
TOL_PCT = 2.0   # UART tolerates ~2%; flag anything worse

EXTS = ('.c', '.h', '.bak', '.py')
SKIP_DIRS = {'Drivers', 'MDK-ARM', '.git', 'venv', '__pycache__', 'build'}

BRR_RE     = re.compile(r'(USART\d)\s*->\s*BRR\s*=\s*(0x[0-9A-Fa-f]+|\d+)\s*U?', re.I)
HALBR_RE   = re.compile(r'\.Init\.BaudRate\s*=\s*(\d+)')
PYSER_RE   = re.compile(r'[Ss]erial\s*\(|baudrate\s*=|BAUD[A-Z_]*\s*=')
NUM_RE     = re.compile(r'\b(115200|921600|460800|230400|57600|38400|28800|19200|14400|9600|7200)\b')


def decode_brr(val):
    mantissa = (val >> 4) & 0xFFF
    frac = val & 0xF
    return mantissa, frac, mantissa + frac / 16.0


def nearest_std(baud):
    return min(STD_BAUDS, key=lambda sb: abs(baud - sb))


def audit():
    files = []
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in filenames:
            if fn.endswith(EXTS):
                files.append(os.path.join(dirpath, fn))

    issues = []
    checked = 0
    print("=" * 100)
    print("INDEPENDENT BAUD AUDIT  (RM0008: USARTDIV=Mantissa+Fraction/16, baud=fCK/(16*USARTDIV))")
    print("Clock tree assumption: HSE=8MHz, PLLx9 -> SYSCLK=72MHz, APB1=36MHz, APB2=72MHz")
    print("=" * 100)

    for path in sorted(files):
        rel = os.path.relpath(path, ROOT)
        try:
            with open(path, encoding="utf-8", errors="replace") as fh:
                lines = fh.read().splitlines()
        except OSError as exc:
            print(f"[SKIP] {rel}: {exc}")
            continue

        for lineno, line in enumerate(lines, 1):
            # --- register-level BRR assignment ---
            m = BRR_RE.search(line)
            if m:
                checked += 1
                usart = m.group(1).upper()
                val = int(m.group(2), 0)
                mant, frac, usartdiv = decode_brr(val)
                fck = 72_000_000 if usart == "USART1" else 36_000_000
                baud = fck / (16.0 * usartdiv)
                near = nearest_std(baud)
                err_pct = (baud - near) / near * 100.0
                ok = abs(err_pct) <= TOL_PCT
                tag = "OK      " if ok else "MISMATCH"
                if not ok:
                    issues.append((rel, lineno, line.strip(), baud, near))
                print(f"\n[{tag}] {rel}:{lineno}")
                print(f"    code : {line.strip()}")
                print(f"    decode: mantissa={mant} frac={frac} -> USARTDIV={usartdiv}")
                print(f"    result: {usart} @ fCK={fck/1e6:.0f}MHz -> {baud:.1f} baud "
                      f"(nearest std {near}, error {err_pct:+.2f}%)")

            # --- HAL-style Init.BaudRate ---
            m = HALBR_RE.search(line)
            if m:
                checked += 1
                claimed = int(m.group(1))
                known = claimed in STD_BAUDS
                tag = "OK      " if known else "MISMATCH"
                if not known:
                    issues.append((rel, lineno, line.strip(), claimed, None))
                print(f"\n[{tag}] {rel}:{lineno}")
                print(f"    code : {line.strip()}")
                print(f"    result: HAL BaudRate={claimed} "
                      f"({'standard rate' if known else 'NOT a standard rate'})")

            # --- PC-side pyserial usage ---
            if path.endswith(".py") and PYSER_RE.search(line):
                nm = NUM_RE.search(line)
                if nm:
                    print(f"\n[PC-SIDE] {rel}:{lineno}")
                    print(f"    code : {line.strip()}")
                    print(f"    result: PC opens port at {nm.group(1)} baud "
                          f"(must equal firmware rate)")

    print("\n" + "=" * 100)
    print(f"CHECKED {checked} baud-related assignments across {len(files)} files")
    if issues:
        print(f"!!! {len(issues)} PROBLEM(S) FOUND:")
        for rel, lineno, src, baud, near in issues:
            extra = f" (nearest std {near})" if near else ""
            print(f"    - {rel}:{lineno}  computed={baud}{extra}\n        {src}")
    else:
        print("ALL CLEAR: every assignment decodes to a standard rate within tolerance.")
    print("=" * 100)


if __name__ == "__main__":
    audit()
