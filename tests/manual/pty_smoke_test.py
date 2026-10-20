#!/usr/bin/env python3
"""Ручные сценарии проверки на эмуляторе сервоприводов — без робота.

Поднимает псевдо-порт с эмулятором шести моторов (fake_servo_bus.py) и
проверяет то, что нельзя проверить обычным ctest: путь инициализации программ
и генератора URDF.

Сценарии:
  1. регистры совпадают с URDF — программы стартуют штатно;
  2. эталонного файла калибровки в проекте нет вовсе — программы всё равно
     работают, потому что границы берутся из регистров;
  3. регистры расходятся с URDF — печатается предупреждение, работа не
     прекращается (источник истины для углов — URDF);
  4. send --show-limits показывает регистры рядом с URDF и вердикт;
  5. маркер --show-limits не путает недоступный URDF минимум мотора;
  6. send --show-limits честно отказывается, если URDF недоступен;
  7. генератор URDF читает границы с шины и перегенерирует файл.

Запуск: python3 tests/manual/pty_smoke_test.py
"""
from __future__ import annotations

import json
import math
import os
import pathlib
import re
import subprocess
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from fake_servo_bus import FakeServoBus  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
# Генератору нужен pyserial, которого нет в системном Python. Если рядом с
# проектом есть .venv, берём его интерпретатор, иначе текущий.
VENV_PYTHON = ROOT / ".venv" / "bin" / "python"
PYTHON = VENV_PYTHON if VENV_PYTHON.exists() else pathlib.Path(sys.executable)
BUILD = ROOT / "build"
URDF = pathlib.Path(os.path.expanduser("~/develop/urdf/so101_irs_ksant.urdf"))
SOURCE_URDF = pathlib.Path(os.path.expanduser("~/develop/urdf/so101_new_calib.urdf"))
STEPS_PER_RADIAN = 4095.0 / (2.0 * math.pi)

# Настоящие минимумы моторов: ноль здесь маскировал бы дефект сравнения.
MOCK_MINS = [767, 812, 886, 900, 260, 1420]

failures: list[str] = []


def check(condition: bool, description: str, detail: str = "") -> None:
    mark = "ok  " if condition else "FAIL"
    print(f"  [{mark}] {description}")
    if detail:
        print(f"         {detail}")
    if not condition:
        failures.append(description)


def urdf_limits() -> list[tuple[int, int]]:
    """Выводит диапазоны моторов из span блоков <limit> — как это делает send.

    Минимумы берутся настоящие, а не нулевые. Ноль здесь был бы верен только
    арифметически, но он же и маскировал дефект: подразуваемый минимум URDF
    по построению равен нулю, поэтому с нулевыми минимумами на эмуляторе
    невозможно отличить корректную работу от сломанной. На живых моторах
    минимумы 767, 812, 886, и любой сравнитель по абсолютным отсчётам на них
    сразу рассыпается.
    """
    import xml.etree.ElementTree as ElementTree

    root = ElementTree.parse(URDF).getroot()
    by_name = {
        joint.get("name"): joint.find("limit")
        for joint in root.iter("joint")
        if joint.get("type") == "revolute"
    }
    order = ["shoulder_pan", "shoulder_lift", "elbow_flex",
             "wrist_flex", "wrist_roll", "gripper"]
    limits = []
    for index, name in enumerate(order):
        span = float(by_name[name].get("upper")) - float(by_name[name].get("lower"))
        limits.append((MOCK_MINS[index], MOCK_MINS[index] + round(span * STEPS_PER_RADIAN)))
    return limits


REFERENCE = urdf_limits()


def build_config(port: str, urdf: pathlib.Path | None = None) -> str:
    """Кладёт конфиг во временный каталог, чтобы не портить сборку."""
    workspace = pathlib.Path(tempfile.mkdtemp(prefix="so101-manual-"))
    config = json.loads((BUILD / "send_config.json").read_text())
    config["port"] = port
    if urdf is not None:
        config["urdf_path"] = str(urdf)
    (workspace / "send_config.json").write_text(json.dumps(config, indent=2))
    return str(workspace)


def run(binary: str, workspace: str, *arguments: str) -> subprocess.CompletedProcess:
    """Запускает программу в подготовленном каталоге.

    Порт берётся из send_config.json внутри workspace. Раньше base_angle этот
    файл не читал и молча открывал /dev/ttyACM0, то есть прогон «на эмуляторе»
    на самом деле обращался к живой руке. Теперь порт задаётся явно, а
    config_port проверяется, чтобы такая ошибка не могла вернуться молча.
    """
    config_port = json.loads(
        (pathlib.Path(workspace) / "send_config.json").read_text())["port"]
    if not config_port.startswith("/dev/pts/"):
        raise SystemExit(
            f"отказ: сценарий собран не на эмуляторе, порт {config_port}. "
            f"Прогонять на живой руке нельзя — часть сценариев пишет в моторы."
        )
    return subprocess.run(
        [str(BUILD / binary), *arguments],
        cwd=workspace,
        capture_output=True,
        text=True,
        timeout=60,
        stdin=subprocess.DEVNULL,
    )


def resized_urdf(span_steps: int) -> pathlib.Path:
    """URDF, где shoulder_pan имеет диапазон span_steps отсчётов."""
    text = URDF.read_text()
    match = re.search(r'lower="(-?[\d.]+)" upper="(-?[\d.]+)"', text)
    if match is None:
        raise SystemExit("не нашёлся блок <limit> shoulder_pan")
    lower = float(match.group(1))
    upper = lower + span_steps * 2.0 * math.pi / 4095.0
    broken = text.replace(
        f'lower="{match.group(1)}" upper="{match.group(2)}"',
        f'lower="{lower}" upper="{upper}"',
        1,
    )
    target = pathlib.Path(tempfile.mkdtemp(prefix="so101-urdf-")) / "resized.urdf"
    target.write_text(broken)
    return target


def scenario_registers_match_urdf() -> None:
    print("\n1. Регистры совпадают с URDF")
    with FakeServoBus([lo for lo, _ in REFERENCE], [hi for _, hi in REFERENCE]) as bus:
        workspace = build_config(bus.start())
        result = run("base_angle", workspace)
        check(result.returncode == 0, "base_angle завершился успешно",
              result.stderr.strip()[:200])
        check("angle:" in result.stdout, "base_angle напечатал углы")
        check("Warning" not in result.stderr, "предупреждения о расхождении нет")


def scenario_without_calibration_file() -> None:
    print("\n2. Файла калибровки в проекте нет")
    check(not (ROOT / "servo_limits.json").exists(),
          "servo_limits.json действительно удалён из проекта")
    with FakeServoBus([lo for lo, _ in REFERENCE], [hi for _, hi in REFERENCE]) as bus:
        workspace = build_config(bus.start())
        result = run("base_angle", workspace)
        check(result.returncode == 0, "base_angle работает без файла калибровки",
              result.stderr.strip()[:200])


def scenario_registers_disagree_with_urdf() -> None:
    print("\n3. Регистры расходятся с URDF")
    mins = [lo for lo, _ in REFERENCE]
    maxs = [hi for _, hi in REFERENCE]
    maxs[0] -= 300
    with FakeServoBus(mins, maxs) as bus:
        workspace = build_config(bus.start())
        result = run("cartesian_control", workspace)
        output = result.stdout + result.stderr
        check("Warning" in output, "печатается предупреждение")
        check("shoulder_pan" in output, "назван расходящийся сустав")
        check("servo registers give" in output, "приведены обе величины")
        check("Cartesian control stopped" not in output,
              "работа не прекращена — источник истины для углов URDF")
        check("servo registers [deg]:" in result.stdout,
              "границы из регистров напечатаны при старте")


def scenario_show_limits_matches_urdf() -> None:
    print("\n4. send --show-limits против URDF")
    mins = [lo for lo, _ in REFERENCE]
    maxs = [hi for _, hi in REFERENCE]
    maxs[2] -= 300  # elbow_flex расходится с URDF
    with FakeServoBus(mins, maxs) as bus:
        workspace = build_config(bus.start())
        result = run("send", workspace, "--show-limits")
        output = result.stdout + result.stderr
        check(result.returncode == 0, "вывод завершился успешно",
              result.stderr.strip()[:200])
        check(output.count("|") >= 6, "шесть строк таблицы")
        check("joint 3 (elbow_flex):" in output,
              "назван расходящийся сустав номером и именем")
        joint3_from_registers = (maxs[2] - mins[2]) * 360.0 / 4095.0
        joint3_from_urdf = REFERENCE[2][1] - REFERENCE[2][0]
        joint3_from_urdf = joint3_from_urdf * 360.0 / 4095.0
        check(f"{joint3_from_registers:.2f}" in output
              and f"{joint3_from_urdf:.2f}" in output,
              "приведены обе величины — из регистра и из URDF",
              f"{joint3_from_registers:.2f} и {joint3_from_urdf:.2f}")
        check(output.count("<- differs") == 1,
              "помечена ровно одна расходящаяся строка",
              f"меток {output.count('<- differs')}")
        check("sync_urdf_limits.py" in output, "подсказано, что делать дальше")


def scenario_marker_ignores_absolute_minimum() -> None:
    """Регрессия: маркер обязан смотреть на диапазон, а не на отсчёты.

    Минимумы мотора (767, 812, ...) абсолютным отсчётам URDF недоступны —
    там градусы. Раньше маркер сравнивал их напрямую и помечал все шесть
    строк, тогда как вердикт считал расхождение допустимым.
    """
    print("\n5. Маркер не путает минимум мотора с недоступным ему отсчётом")
    with FakeServoBus([lo for lo, _ in REFERENCE], [hi for _, hi in REFERENCE]) as bus:
        workspace = build_config(bus.start())
        result = run("send", workspace, "--show-limits")
        output = result.stdout + result.stderr
        check(result.returncode == 0, "вывод завершился успешно",
              result.stderr.strip()[:200])
        check("OK:" in output, "вердикт OK — диапазоны совпадают")
        check("<- differs" not in output,
              "ни одна строка не помечена, хотя минимумы ненулевые",
              f"меток {output.count('<- differs')}")
        check("0.50" in output or "0.5" in output,
              "допуск напечатан, иначе OK рядом с дельтой удивляет")


def scenario_show_limits_without_urdf() -> None:
    print("\n6. send --show-limits без доступного URDF")
    with FakeServoBus([lo for lo, _ in REFERENCE], [hi for _, hi in REFERENCE]) as bus:
        workspace = build_config(bus.start(), urdf=pathlib.Path("/no/such.urdf"))
        result = run("send", workspace, "--show-limits")
        output = result.stdout + result.stderr
        check(result.returncode != 0, "ненулевой код возврата")
        check("URDF comparison impossible" in output,
              "честно сказано, что сравнение невозможно")
        check(str(MOCK_MINS[0]) in output,
              "границы по регистрам всё равно показаны")


def scenario_generator_reads_bus() -> None:
    print("\n7. Генератор URDF читает границы с шины")
    mins = [lo for lo, _ in REFERENCE]
    maxs = [hi for _, hi in REFERENCE]
    maxs[5] -= 42  # gripper перенастроили
    workspace = pathlib.Path(tempfile.mkdtemp(prefix="so101-gen-"))
    output = workspace / "generated.urdf"

    with FakeServoBus(mins, maxs) as bus:
        result = subprocess.run(
            [str(PYTHON), str(ROOT / "tools" / "sync_urdf_limits.py"),
             "--port", bus.start(), "--baud", "1000000",
             "--source", str(SOURCE_URDF), "--output", str(output)],
            cwd=str(workspace),
            capture_output=True,
            text=True,
            timeout=120,
        )

    check(result.returncode == 0, "генератор завершился успешно",
          result.stderr.strip()[:300])
    check(output.exists(), "URDF записан")
    if not output.exists():
        return

    check(output.read_text() != URDF.read_text(),
          "URDF отличается от прежнего — диапазон gripper действительно изменился")

    import xml.etree.ElementTree as ElementTree
    root = ElementTree.parse(output).getroot()
    by_name = {joint.get("name"): joint.find("limit")
               for joint in root.iter("joint")
               if joint.get("type") == "revolute"}
    gripper = by_name["gripper"]
    span = float(gripper.get("upper")) - float(gripper.get("lower"))
    # Сравнивается ДИАПАЗОН, а не абсолютный отсчёт: в URDF градусы, и
    # абсолютного минимума мотора там нет вовсе.
    expected_span = maxs[5] - mins[5]
    check(round(span * STEPS_PER_RADIAN) == expected_span,
          "диапазон gripper в URDF совпадает с регистрами",
          f"{round(span * STEPS_PER_RADIAN)} против {expected_span}")

    # Перегенерация обязана быть идемпотентной: второй запуск с теми же
    # границами не должен ничего менять.
    check(run_generator_again(output) == output.read_text(),
          "повторная генерация с теми же границами идемпотентна")


def run_generator_again(previous: pathlib.Path) -> str:
    mins = [lo for lo, _ in REFERENCE]
    maxs = [hi for _, hi in REFERENCE]
    maxs[5] -= 42
    workspace = pathlib.Path(tempfile.mkdtemp(prefix="so101-gen2-"))
    output = workspace / "again.urdf"
    with FakeServoBus(mins, maxs) as bus:
        subprocess.run(
            [str(PYTHON), str(ROOT / "tools" / "sync_urdf_limits.py"),
             "--port", bus.start(), "--baud", "1000000",
             "--source", str(SOURCE_URDF), "--output", str(output)],
            cwd=str(workspace), capture_output=True, text=True, timeout=120,
        )
    return output.read_text() if output.exists() else previous.read_text() + " (нет файла)"


def main() -> int:
    if not (BUILD / "cartesian_control").exists():
        raise SystemExit("сначала соберите проект: cmake --build build")
    if not URDF.exists():
        raise SystemExit(f"не найден URDF: {URDF}")
    if not SOURCE_URDF.exists():
        raise SystemExit(f"не найден исходный URDF: {SOURCE_URDF}")

    scenario_registers_match_urdf()
    scenario_without_calibration_file()
    scenario_registers_disagree_with_urdf()
    scenario_show_limits_matches_urdf()
    scenario_marker_ignores_absolute_minimum()
    scenario_show_limits_without_urdf()
    scenario_generator_reads_bus()

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