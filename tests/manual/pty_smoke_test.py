#!/usr/bin/env python3
"""Ручные сценарии проверки на эмуляторе сервоприводов — без робота.

Поднимает псевдо-порт с эмулятором шести моторов (fake_servo_bus.py) и
проверяет то, что нельзя проверить обычным ctest: путь инициализации программ.

Сценарии:
  1. регистры совпадают с эталоном — программы стартуют штатно;
  2. эталонного файла servo_limits.json нет вовсе — программы всё равно
     работают, потому что границы берутся из регистров;
  3. регистры расходятся с URDF — печатается предупреждение, работа не
     прекращается (источник истины для углов — URDF);
  4. send --show-limits показывает оба источника и вердикт о расхождении;
  5. send --save-limits перезаписывает эталон значениями регистров;
  6. base_angle читает положения и печатает их в градусах.

Запуск: python3 tests/manual/pty_smoke_test.py
"""
from __future__ import annotations

import json
import os
import pathlib
import re
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from fake_servo_bus import FakeServoBus  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
BUILD = ROOT / "build"
URDF = pathlib.Path(os.path.expanduser("~/develop/urdf/so101_irs_ksant.urdf"))
REFERENCE = json.loads((ROOT / "servo_limits.json").read_text())["servos"]

failures: list[str] = []


def check(condition: bool, description: str, detail: str = "") -> None:
    mark = "ok  " if condition else "FAIL"
    print(f"  [{mark}] {description}")
    if detail:
        print(f"         {detail}")
    if not condition:
        failures.append(description)


def build_config(port: str, urdf: pathlib.Path | None = None) -> str:
    """Кладёт конфиг и эталон во временный каталог, чтобы не портить сборку."""
    workspace = pathlib.Path(tempfile.mkdtemp(prefix="so101-manual-"))
    config = json.loads((BUILD / "send_config.json").read_text())
    config["port"] = port
    if urdf is not None:
        config["urdf_path"] = str(urdf)
    (workspace / "send_config.json").write_text(json.dumps(config, indent=2))
    return str(workspace)


def run(binary: str, workspace: str, *arguments: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [str(BUILD / binary), *arguments],
        cwd=workspace,
        capture_output=True,
        text=True,
        timeout=60,
        stdin=subprocess.DEVNULL,
    )


def broken_urdf() -> pathlib.Path:
    """URDF с намеренно испорченной верхней границей shoulder_pan."""
    text = URDF.read_text()
    # Первый аргумент — паттерн (экранированные точки), второй — подстановка,
    # здесь экранировать ничего не нужно.
    broken = re.sub(
        r'lower="-1\.91986" upper="2\.0096241445"',
        'lower="-1.91986" upper="1.91986"',
        text,
    )
    if broken == text:
        raise SystemExit(
            "не нашлась строка <limit> shoulder_pan — генератор URDF изменился, "
            "обновите тест"
        )
    target = pathlib.Path(tempfile.mkdtemp(prefix="so101-urdf-")) / "broken.urdf"
    target.write_text(broken)
    return target


def scenario_registers_match_reference() -> None:
    print("\n1. Регистры совпадают с эталоном")
    with FakeServoBus([s["min"] for s in REFERENCE],
                      [s["max"] for s in REFERENCE]) as bus:
        workspace = build_config(bus.start())
        (pathlib.Path(workspace) / "servo_limits.json").write_text(
            (BUILD / "servo_limits.json").read_text()
        )
        result = run("base_angle", workspace)
        check(result.returncode == 0, "base_angle завершился успешно",
              result.stderr.strip()[:200])
        check("angle:" in result.stdout, "base_angle напечатал углы")
        check("Warning" not in result.stderr,
              "предупреждения о расхождении нет")


def scenario_without_reference_file() -> None:
    print("\n2. Эталонного файла нет в каталоге запуска")
    with FakeServoBus([s["min"] for s in REFERENCE],
                      [s["max"] for s in REFERENCE]) as bus:
        workspace = build_config(bus.start())
        # servo_limits.json намеренно не копируем.
        result = run("base_angle", workspace)
        check(result.returncode == 0,
              "base_angle работает без файла калибровки",
              result.stderr.strip()[:200])


def scenario_registers_disagree_with_urdf() -> None:
    print("\n3. Регистры расходятся с URDF")
    mins = [s["min"] for s in REFERENCE]
    maxs = [s["max"] for s in REFERENCE]
    maxs[0] -= 300  # shoulder_pan: 225.14 -> 208.88 градуса
    with FakeServoBus(mins, maxs) as bus:
        workspace = build_config(bus.start(), urdf=broken_urdf())
        (pathlib.Path(workspace) / "servo_limits.json").write_text(
            (BUILD / "servo_limits.json").read_text()
        )
        result = run("cartesian_control", workspace)
        output = result.stdout + result.stderr
        check("Warning" in output, "печатается предупреждение")
        check("shoulder_pan" in output, "назван расходящийся сустав")
        check("servo registers give" in output, "приведены обе величины")
        check("Cartesian control stopped" not in output,
              "работа не прекращена — источник истины для углов URDF")
        check("servo registers [deg]:" in result.stdout,
              "границы из регистров напечатаны при старте")


def scenario_show_limits_comparison() -> None:
    print("\n4. send --show-limits показывает оба источника")
    mins = [s["min"] for s in REFERENCE]
    maxs = list(s["max"] for s in REFERENCE)
    maxs[2] = maxs[2] - 300  # elbow_flex расходится с эталоном
    with FakeServoBus(mins, maxs) as bus:
        workspace = build_config(bus.start())
        (pathlib.Path(workspace) / "servo_limits.json").write_text(
            (BUILD / "servo_limits.json").read_text()
        )
        result = run("send", workspace, "--show-limits")
        output = result.stdout + result.stderr
        check(result.returncode == 0, "вывод завершился успешно",
              result.stderr.strip()[:200])
        check(output.count("|  registers |") == 6,
              "шесть строк с регистрами",
              f"найдено {output.count('|  registers |')}")
        check(output.count("|  reference |") == 6,
              "шесть строк с эталоном",
              f"найдено {output.count('|  reference |')}")
        check("joint 3:" in output, "назван расходящийся сустав номером")
        check("Warning" in output, "выведено предупреждение о расхождении")
        check("sync_urdf_limits.py" in output, "подсказано, что делать дальше")


def scenario_save_limits() -> None:
    print("\n5. send --save-limits обновляет эталон")
    mins = [s["min"] for s in REFERENCE]
    maxs = list(s["max"] for s in REFERENCE)
    maxs[5] -= 42  # gripper перенастроили
    with FakeServoBus(mins, maxs) as bus:
        workspace = build_config(bus.start())
        reference_path = pathlib.Path(workspace) / "servo_limits.json"
        reference_path.write_text((BUILD / "servo_limits.json").read_text())

        result = run("send", workspace, "--save-limits")
        check(result.returncode == 0, "команда завершилась успешно",
              result.stderr.strip()[:200])
        check(reference_path.exists(), "эталонный файл создан")
        saved = json.loads(reference_path.read_text())["servos"]
        check([s["max"] for s in saved] == maxs,
              "эталон записан по значениям регистров")

        follow_up = run("send", workspace, "--show-limits")
        check("OK: servo registers match" in follow_up.stdout,
              "после сохранения расхождений не осталось")


def main() -> int:
    if not (BUILD / "cartesian_control").exists():
        raise SystemExit("сначала соберите проект: cmake --build build")
    if not URDF.exists():
        raise SystemExit(f"не найден URDF: {URDF}")

    scenario_registers_match_reference()
    scenario_without_reference_file()
    scenario_registers_disagree_with_urdf()
    scenario_show_limits_comparison()
    scenario_save_limits()

    print()
    if failures:
        print(f"Провалено сценариев: {len(failures)}")
        for item in failures:
            print(f"  - {item}")
        return 1
    print("Все сценарии пройдены.")
    return 0


if __name__ == "__main__":
    sys.exit(main())