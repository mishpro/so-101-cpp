#!/usr/bin/env python3
"""Проверка cartesian_control на псевдо-порту: без железа, без робота.

Создаёт пару pty, подставляет путь slave-устройства в send_config.json и
запускает бинарник. Проверяются два сценария:

  1. URDF согласован с калибровкой — программа доходит до чтения сервоприводов;
  2. границы URDF разошлись с servo_limits.json — программа отказывается
     работать и называет сломанный сустав.
"""
import json
import os
import pty
import re
import shutil
import subprocess
import sys
from pathlib import Path

BUILD = Path(__file__).resolve().parents[2] / "build"
CONFIG = BUILD / "send_config.json"
URDF = Path(os.path.expanduser("~/develop/urdf/so101_irs_ksant.urdf"))


def run_with_port(port: str) -> tuple[int, str]:
    saved = CONFIG.read_text()
    data = json.loads(saved)
    data["port"] = port
    CONFIG.write_text(json.dumps(data, indent=2))
    try:
        with open(os.devnull) as devnull:
            finished = subprocess.run(
                [str(BUILD / "cartesian_control")],
                stdin=devnull, capture_output=True, text=True, timeout=30,
                cwd=BUILD,  # конфиг читается из текущего каталога
            )
    finally:
        CONFIG.write_text(saved)
    return finished.returncode, finished.stdout + finished.stderr


def break_limits() -> Path:
    """URDF с намеренно испорченной верхней границей shoulder_pan."""
    text = URDF.read_text()
    broken = re.sub(
        r'lower="-1\.91986" upper="2\.0096241445"',
        'lower="-1.91986" upper="1.91986"',
        text,
    )
    if broken == text:
        raise SystemExit("не нашлась строка <limit> shoulder_pan — тест устарел")
    path = Path("/tmp/opencode/so101_broken_limits.urdf")
    path.write_text(broken)
    return path


def main() -> int:
    if not (BUILD / "cartesian_control").exists():
        raise SystemExit("сначала соберите проект: cmake --build build")

    master, slave = pty.openpty()
    port = os.ttyname(slave)
    try:
        code, output = run_with_port(port)
        print("=== 1. границы URDF совпадают с калибровкой ===")
        print(output.strip()[:400])
        reached_servo_read = "angle read" in output or "outside saved limits" in output \
            or "Failed to read" in output
        print(f"код выхода: {code}; дошёл до чтения сервоприводов: {reached_servo_read}")

        # Теперь сломанный URDF.
        broken = break_limits()
        config_backup = CONFIG.read_text()
        data = json.loads(config_backup)
        data["port"] = port
        data["urdf_path"] = str(broken)
        CONFIG.write_text(json.dumps(data, indent=2))
        try:
            with open(os.devnull) as devnull:
                finished = subprocess.run(
                    [str(BUILD / "cartesian_control")],
                    stdin=devnull, capture_output=True, text=True, timeout=30,
                    cwd=BUILD,
                )
        finally:
            CONFIG.write_text(config_backup)

        text = finished.stdout + finished.stderr
        print("\n=== 2. границы URDF разошлись с калибровкой ===")
        print(text.strip()[:400])
        print(f"код выхода: {finished.returncode}")
    finally:
        os.close(master)
        os.close(slave)

    return 0


if __name__ == "__main__":
    sys.exit(main())
