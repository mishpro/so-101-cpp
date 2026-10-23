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
  7. генератор URDF читает границы с шины и перегенерирует файл;
  8. захват плоскости и рисование круга проходят весь путь по шине;
  9. расслабление моторов переживает потерю кадров;
 10. без нагрузки на моторах рисование продолжается: след без нажима;
  11. с ключом --no-contact нагрузка не читается вовсе — ни при захвате,
      ни при рисовании;
  12. круг с недостижимыми точками рисуется по приближению, и команда
      моторам на недостижимой точке отличается от предыдущей;
  13. касание не найдено: сказано почему и на какой высоте.

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
from fake_servo_bus import (  # noqa: E402
    REG_GOAL_POSITION,
    REG_TORQUE_ENABLE,
    FakeServoBus,
)

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


def run(binary: str, workspace: str, *arguments: str,
        stdin_data: str | None = None) -> subprocess.CompletedProcess:
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
        input=stdin_data if stdin_data is not None else None,
        stdin=subprocess.DEVNULL if stdin_data is None else None,
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


def steps_for_angles(angles_deg: list[float]) -> list[int]:
    """Переводит углы URDF в отсчёты мотора для эмулятора.

    Угол мотора отсчитывается от его нуля, а нуль мотора — это нижняя
    граница URDF: так же, как это делает send.
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
    limits = urdf_limits()
    steps = []
    for index, name in enumerate(order):
        # Границы в URDF в радианах, а углы мотора считаются в градусах.
        lower = math.degrees(float(by_name[name].get("lower")))
        motor_degrees = angles_deg[index] - lower
        steps.append(limits[index][0] +
                     round(motor_degrees * 4095.0 / 360.0))
    return steps


def scenario_draw_circle_on_fake_bus() -> None:
    print("\n8. Рисование круга на эмуляторе шины")
    limits = urdf_limits()
    mins = [item[0] for item in limits]
    maxs = [item[1] for item in limits]
    # Поза не вытянутая: у вытянутой руки вертикальный инструмент
    # недостижим (рабочая область кончается), и проверять было бы нечего.
    present = steps_for_angles([0.0, -30.0, 60.0, 30.0, 0.0, 0.0])
    # Мёртвая зона мотора: настоящий STS3215 до своей цели не доходит, и
    # из-за этого рисование останавливалось на первой точке. Зона
    # воспроизводится в отсчётах мотора.
    with FakeServoBus(mins, maxs, present, follow_goal=True,
                      deadband_steps=20) as bus:
        port = bus.start()
        workspace = build_config(port)
        plane_path = pathlib.Path(workspace) / "draw_plane.json"

        # Захват плоскости: оператор «нажимает Enter» (ввод подаётся через
        # stdin), программа должна записать файл плоскости.
        # Допуск наклона максимальный: эмулятор стоит в произвольной позе,
        # и проверить тут надо путь по шине, а не точность захвата.
        capture = run("draw_circle", workspace, "--set-plane",
                      "--plane", str(plane_path), "--tilt-tolerance", "90",
                      stdin_data="\n")
        check(capture.returncode == 0,
              "захват плоскости на эмуляторе завершился успешно",
              capture.stderr.strip()[:400])
        check(plane_path.exists(), "файл плоскости записан")
        if plane_path.exists():
            plane = json.loads(plane_path.read_text())
            check("tcp_height_m" in plane, "в файле плоскости есть высота TCP")
            check("tilt_deg" in plane, "в файле плоскости есть наклон")

        # После захвата моторы должны быть расслаблены: руку надо править
        # руками, пока программа не работает.
        relaxed = [bus.torque_state(servo) for servo in range(1, 7)]
        check(all(value == 0 for value in relaxed),
              "после захвата все моторы расслаблены", str(relaxed))

        # Рисование: центр окружности — в точке касания, так что окружность
        # рядом с текущей позой и IK обязан сойтись.
        if not plane_path.exists():
            check(False, "файл плоскости существует для рисования")
            return
        # Бумага там, где её оставил захват: после касания рука стоит на
        # цели, то есть на поверхности.
        bus.paper_steps = list(bus.goal_positions)
        plane = json.loads(plane_path.read_text())
        before = len(bus.writes_to(REG_GOAL_POSITION))
        draw = run("draw_circle", workspace,
                   "--center", f"{plane['contact_x_m']:.4f}",
                   f"{plane['contact_y_m']:.4f}",
                   "--radius", "0.02", "--points", "8",
                   "--plane", str(plane_path), "--no-confirm", stdin_data="\n")
        check(draw.returncode == 0, "рисование круга на эмуляторе завершилось",
              draw.stderr.strip()[:400])
        after = len(bus.writes_to(REG_GOAL_POSITION))
        check(after > before, "в моторы ушли цели по окружности",
              f"{before} -> {after}")
        relaxed = [bus.torque_state(servo) for servo in range(1, 7)]
        check(all(value == 0 for value in relaxed),
              "после рисования все моторы расслаблены", str(relaxed))
        # Мотор не дошёл до своей цели: именно это и ломало рисование.
        check(bus.present_positions != bus.goal_positions,
              "мотор остался на мёртвой зоне, а не в цели")


def scenario_torque_release_survives_lost_frames() -> None:
    print("\n9. Расслабление моторов переживает потерю кадров")
    limits = urdf_limits()
    mins = [item[0] for item in limits]
    maxs = [item[1] for item in limits]
    present = steps_for_angles([0.0, -30.0, 60.0, 30.0, 0.0, 0.0])
    with FakeServoBus(mins, maxs, present, follow_goal=True) as bus:
        # Шина теряет первые четыре кадра записи удержания: моторы их не
        # видят. Программа обязана это заметить (спросив у моторов) и
        # повторить команду.
        bus.drop_torque_writes = 4
        port = bus.start()
        workspace = build_config(port)
        plane_path = pathlib.Path(workspace) / "draw_plane.json"

        capture = run("draw_circle", workspace, "--set-plane",
                      "--plane", str(plane_path), "--tilt-tolerance", "90",
                      stdin_data="\n")
        check(capture.returncode == 0,
              "захват завершился успешно, несмотря на потерю кадров",
              capture.stderr.strip()[:400])
        relaxed = [bus.torque_state(servo) for servo in range(1, 7)]
        check(all(value == 0 for value in relaxed),
              "после захвата все моторы расслаблены", str(relaxed))
        torque_writes = len(bus.writes_to(REG_TORQUE_ENABLE))
        check(torque_writes > 6,
              "потерянные команды были повторены",
              f"записей удержания: {torque_writes}")


def scenario_draw_without_torque_signal() -> None:
    print("\n10. Без нагрузки на моторах рисование продолжается")
    limits = urdf_limits()
    mins = [item[0] for item in limits]
    maxs = [item[1] for item in limits]
    present = steps_for_angles([0.0, -30.0, 60.0, 30.0, 0.0, 0.0])
    with FakeServoBus(mins, maxs, present, follow_goal=True) as bus:
        # Моторы не отдают регистр нагрузки.
        bus.fail_load_reads = True
        port = bus.start()
        workspace = build_config(port)
        plane_path = pathlib.Path(workspace) / "draw_plane.json"

        capture = run("draw_circle", workspace, "--set-plane",
                      "--plane", str(plane_path), "--tilt-tolerance", "90",
                      stdin_data="\n")
        check(capture.returncode == 0,
              "захват плоскости без нагрузки завершился успешно",
              capture.stderr.strip()[:400])

        if not plane_path.exists():
            check(False, "файл плоскости существует для рисования")
            return
        plane = json.loads(plane_path.read_text())
        draw = run("draw_circle", workspace,
                   "--center", f"{plane['contact_x_m']:.4f}",
                   f"{plane['contact_y_m']:.4f}",
                   "--radius", "0.02", "--points", "8",
                   "--plane", str(plane_path), "--no-confirm", stdin_data="\n")
        # Нагрузка необязательна: без неё программа работает по высоте из
        # плоскости, нажим не проверяет, а точки считает «без нажима».
        check(draw.returncode == 0,
              "без нагрузки рисование всё равно завершилось",
              draw.stderr.strip()[:400])
        output = draw.stdout + draw.stderr
        done_line = next((line for line in draw.stdout.splitlines()
                          if line.startswith("Done:")), "")
        check(bool(done_line), "программа отчиталась о завершении",
              draw.stdout.strip()[-200:])
        check("Done: 8 points" in done_line,
              "все точки окружности пройдены", done_line)
        # Счётчик отчёта: точки без нажима. Он равен числу точек — нажима
        # не было ни на одной.
        check("points without press 8" in done_line,
              "все точки посчитаны «без нажима»", done_line)
        check("not readable" not in output.lower(),
              "программа не жаловалась на нечитаемую нагрузку",
              output.strip()[-200:])
        check("Drawing stopped" not in output,
              "рисование не прервано", output.strip()[-200:])
        # Нагрузку программа всё-таки пыталась читать: отказ мотора виден
        # на шине, и он не помешал рисованию.
        check(bus.load_reads > 0,
              "программа читала нагрузку и получила отказ мотора",
              f"чтений нагрузки: {bus.load_reads}")
        relaxed = [bus.torque_state(servo) for servo in range(1, 7)]
        check(all(value == 0 for value in relaxed),
              "после рисования все моторы расслаблены", str(relaxed))


def scenario_draw_without_contact_option() -> None:
    """Рисование с ключом --no-contact: нагрузка не читается вовсе.

    Отдельный случай, а не продолжение десятого: там мотор отказывает в
    ответе, а здесь программа сама не спрашивает. Нагрузка читаться может,
    но не читается — и это видно на шине по числу чтений регистра.
    """
    print("\n11. Рисование без чтения нагрузки (--no-contact)")
    limits = urdf_limits()
    mins = [item[0] for item in limits]
    maxs = [item[1] for item in limits]
    present = steps_for_angles([0.0, -30.0, 60.0, 30.0, 0.0, 0.0])
    with FakeServoBus(mins, maxs, present, follow_goal=True,
                      deadband_steps=20) as bus:
        port = bus.start()
        workspace = build_config(port)
        plane_path = pathlib.Path(workspace) / "draw_plane.json"

        capture = run("draw_circle", workspace, "--set-plane",
                      "--plane", str(plane_path), "--tilt-tolerance", "90",
                      stdin_data="\n")
        check(capture.returncode == 0,
              "захват плоскости завершился успешно", capture.stderr.strip()[:400])
        if not plane_path.exists():
            check(False, "файл плоскости существует для рисования")
            return

        # Бумага там, где рука стояла после захвата.
        bus.paper_steps = list(bus.goal_positions)
        plane = json.loads(plane_path.read_text())

        # Захват с тем же ключом: высота касанием не уточняется, и нагрузка
        # не читается вовсе. Флаг --no-contact действует и на захват, а не
        # только на рисование: иначе «--set-plane --no-contact» всё равно
        # читал бы нагрузку при уточнении высоты бумаги.
        plane_without = pathlib.Path(workspace) / "draw_plane_no_contact.json"
        reads_before_capture = bus.load_reads
        capture_without = run("draw_circle", workspace, "--set-plane",
                              "--plane", str(plane_without),
                              "--tilt-tolerance", "90", "--no-contact",
                              stdin_data="\n")
        check(capture_without.returncode == 0,
              "захват без нагрузки завершился успешно",
              capture_without.stderr.strip()[:400])
        check(bus.load_reads == reads_before_capture,
              "при захвате нагрузка не читалась ни разу",
              f"чтений нагрузки: {bus.load_reads - reads_before_capture}")

        # Захват нагрузку читал: уточнял высоту бумаги касанием. Считаем
        # эти чтения отдельно, чтобы видеть, читал ли их сам рисунок.
        reads_before_draw = bus.load_reads

        draw = run("draw_circle", workspace,
                   "--center", f"{plane['contact_x_m']:.4f}",
                   f"{plane['contact_y_m']:.4f}",
                   "--radius", "0.02", "--points", "8",
                   "--plane", str(plane_path), "--no-confirm", "--no-contact",
                   stdin_data="\n")
        check(draw.returncode == 0,
              "рисование без нагрузки завершилось успешно",
              draw.stderr.strip()[:400])
        done_line = next((line for line in draw.stdout.splitlines()
                          if line.startswith("Done:")), "")
        check("Done: 8 points" in done_line,
              "все точки окружности пройдены", done_line)
        # Нажим не проверялся, поэтому ни одна точка не «с отрывом» по
        # нагрузке, но все посчитаны «без нажима»: проверять было нечем.
        check("points without press 8" in done_line,
              "все точки посчитаны «без нажима»", done_line)
        check(bus.load_reads == reads_before_draw,
              "нагрузка не читалась ни разу за рисование",
              f"чтений нагрузки: {bus.load_reads - reads_before_draw}")
        relaxed = [bus.torque_state(servo) for servo in range(1, 7)]
        check(all(value == 0 for value in relaxed),
              "после рисования все моторы расслаблены", str(relaxed))


def scenario_draw_with_unreachable_points() -> None:
    """Круг с недостижимыми точками: недостижимое рисуется по приближению.

    Недостижимая точка — не повод бросить круг: программа говорит о ней и
    идёт к следующей. Но раньше моторам уходила поза предыдущей точки: рука
    не двигалась вовсе, и точка выпадала из круга дыркой. Теперь моторам
    уходит лучшее приближение, и точка дорисовывается — насколько позволяет
    досягаемость.
    """
    print("\n12. Круг с недостижимыми точками рисуется по приближению")
    limits = urdf_limits()
    mins = [item[0] for item in limits]
    maxs = [item[1] for item in limits]
    present = steps_for_angles([0.0, -30.0, 60.0, 30.0, 0.0, 0.0])
    with FakeServoBus(mins, maxs, present, follow_goal=True,
                      deadband_steps=20) as bus:
        port = bus.start()
        workspace = build_config(port)
        plane_path = pathlib.Path(workspace) / "draw_plane.json"

        capture = run("draw_circle", workspace, "--set-plane",
                      "--plane", str(plane_path), "--tilt-tolerance", "90",
                      stdin_data="\n")
        check(capture.returncode == 0, "захват плоскости завершился успешно",
              capture.stderr.strip()[:400])
        if not plane_path.exists():
            check(False, "файл плоскости существует для рисования")
            return

        bus.paper_steps = list(bus.goal_positions)
        # Центр окружности унесён вбок: часть точек заведомо вне досягаемости,
        # часть — рядом с рукой. Подстройка выключена, чтобы команды шли
        # ровно по одной на точку.
        draw = run("draw_circle", workspace, "--center", "0.26", "0.05",
                   "--radius", "0.05", "--points", "8",
                   "--plane", str(plane_path), "--no-confirm", "--no-compensate",
                   stdin_data="\n")
        output = draw.stdout + draw.stderr
        check(draw.returncode == 0,
              "рисование не остановилось на недостижимых точках",
              draw.stderr.strip()[:400])
        check("недостижимы" in draw.stdout,
              "программа сказала, какие точки недостижимы",
              draw.stdout.strip()[-300:])
        done_line = next((line for line in draw.stdout.splitlines()
                          if line.startswith("Done:")), "")
        check(bool(done_line), "программа отчиталась о завершении", done_line)
        check("unreachable points 2" in done_line,
              "недостижимые точки посчитаны отдельно", done_line)
        # Недостижимая точка не выпадает из круга: моторам ушло лучшее
        # приближение, и рука дошла до него. Раньше моторам уходила поза
        # предыдущей точки, рука не двигалась, и точка пропускалась.
        check("Done: 8 points" in done_line,
              "все точки пройдены, недостижимые — по приближению", done_line)
        check("точка пропущена" not in draw.stdout,
              "недостижимые точки не выпали из круга",
              draw.stdout.strip()[-300:])
        check("Drawing stopped" not in output,
              "рисование не прервано", output.strip()[-200:])
        relaxed = [bus.torque_state(servo) for servo in range(1, 7)]
        check(all(value == 0 for value in relaxed),
              "после рисования все моторы расслаблены", str(relaxed))

        # Команда моторам на недостижимой точке отличается от команды
        # предыдущей точки.
        #
        # Отдельный прогон без поиска касания и без подстройки: тогда на
        # каждую точку уходит ровно одна команда, и по шине видно, что на
        # недостижимой точке команда не копия предыдущей.
        before = len(bus.writes_to(REG_GOAL_POSITION))
        draw2 = run("draw_circle", workspace, "--center", "0.26", "0.05",
                    "--radius", "0.05", "--points", "8",
                    "--plane", str(plane_path), "--no-confirm", "--no-contact",
                    "--no-compensate", stdin_data="\n")
        check(draw2.returncode == 0,
              "рисование без нагрузки завершилось успешно",
              draw2.stderr.strip()[:400])
        unreachable = [int(number) for number
                       in re.findall(r"точка (\d+): IK не сошёлся",
                                     draw2.stdout)]
        check(bool(unreachable),
              "программа назвала недостижимые точки",
              str(unreachable))
        # По шесть записей цели на команду: по одной на мотор.
        goals = bus.writes_to(REG_GOAL_POSITION)[before:]
        commands = [goals[index * 6:(index + 1) * 6]
                    for index in range(len(goals) // 6)]
        check(len(commands) == 8,
              "на каждую точку ушла ровно одна команда",
              f"команд: {len(commands)}")
        for number in unreachable:
            if number < 2 or number > len(commands):
                continue
            previous = commands[number - 2]
            current = commands[number - 1]
            check(previous != current,
                  f"на недостижимой точке {number} команда отличается от "
                  "предыдущей",
                  f"{previous} против {current}")



def scenario_draw_without_contact_found() -> None:
    """Касание не найдено: программа говорит почему и на какой высоте.

    Бумага, которая не даёт реакции, — это не поломка программы, а
    неверная плоскость или неверные пороги. Но сказать об этом надо
    правильно: с пустой причиной («касание не найдено ()») оператор не
    поймёт, куда смотреть. И уровень нагрузки на свободной высоте нужен
    для того же: если он велик, «свободная» высота свободна не была.
    """
    print("\n13. Касание не найдено: сказано почему и на какой высоте")
    limits = urdf_limits()
    mins = [item[0] for item in limits]
    maxs = [item[1] for item in limits]
    present = steps_for_angles([0.0, -30.0, 60.0, 30.0, 0.0, 0.0])
    with FakeServoBus(mins, maxs, present, follow_goal=True,
                      deadband_steps=20) as bus:
        port = bus.start()
        workspace = build_config(port)
        plane_path = pathlib.Path(workspace) / "draw_plane.json"

        capture = run("draw_circle", workspace, "--set-plane",
                      "--plane", str(plane_path), "--tilt-tolerance", "90",
                      stdin_data="\n")
        check(capture.returncode == 0, "захват плоскости завершился успешно",
              capture.stderr.strip()[:400])
        if not plane_path.exists():
            check(False, "файл плоскости существует для рисования")
            return
        plane = json.loads(plane_path.read_text())

        # Бумага ниже любой цели: нагрузка всегда нулевая, касание не
        # находится никогда.
        bus.paper_steps = list(mins)
        draw = run("draw_circle", workspace, "--center",
                   f"{plane['contact_x_m']:.4f}", f"{plane['contact_y_m']:.4f}",
                   "--radius", "0.02", "--points", "4",
                   "--plane", str(plane_path), "--no-confirm", stdin_data="\n")
        check(draw.returncode == 0, "рисование продолжилось без касания",
              draw.stderr.strip()[:400])
        check("касание не найдено" in draw.stdout,
              "программа сказала, что касания нет", draw.stdout.strip()[-300:])
        # Причина непустая: сколько шагов спуска бумага не отвечала.
        reason = next((line for line in draw.stdout.splitlines()
                       if "касание не найдено" in line), "")
        check("()" not in reason and "шагов спуска" in reason,
              "причина названа, а не оставлена пустой", reason)
        # Уровень нагрузки на свободной высоте: по нему видно, что «свободная»
        # высота была свободна.
        check("the load at the free height was" in draw.stdout,
              "назван уровень нагрузки на свободной высоте",
              draw.stdout.strip()[-300:])
        # Нажим по нагрузке не проверяется, но сигнал читается: это не то же
        # самое, что «нагрузка недоступна».
        check("нагрузка доступна, нажим по ней не проверяется" in draw.stdout,
              "нажим не проверяется, а сигнал читается — сказано отдельно",
              draw.stdout.strip()[-300:])
        check("нагрузка недоступна" not in draw.stdout,
              "сообщения о недоступной нагрузке тут быть не должно",
              draw.stdout.strip()[-300:])
        done_line = next((line for line in draw.stdout.splitlines()
                          if line.startswith("Done:")), "")
        check("Done: 4 points" in done_line,
              "все точки пройдены по высоте плоскости", done_line)
        check("points without press 4" in done_line,
              "точки посчитаны «без нажима»", done_line)

        # Тот же случай в захвате: высота касанием не уточняется, и сообщение
        # говорит почему и на какой высоте.
        capture_without = run("draw_circle", workspace, "--set-plane",
                              "--plane", str(pathlib.Path(workspace) /
                                             "draw_plane_no_paper.json"),
                              "--tilt-tolerance", "90", stdin_data="\n")
        check(capture_without.returncode == 0,
              "захват без бумаги завершился успешно",
              capture_without.stderr.strip()[:400])
        check("Высота бумаги касанием не уточнена" in capture_without.stdout,
              "захват сказал, что высота не уточнена",
              capture_without.stdout.strip()[-300:])
        check("шагов спуска" in capture_without.stdout,
              "причина названа и при захвате",
              capture_without.stdout.strip()[-300:])
        check("the load at the free height was" in capture_without.stdout,
              "уровень на свободной высоте назван и при захвате",
              capture_without.stdout.strip()[-300:])


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
    scenario_draw_circle_on_fake_bus()
    scenario_torque_release_survives_lost_frames()
    scenario_draw_without_torque_signal()
    scenario_draw_without_contact_option()
    scenario_draw_with_unreachable_points()
    scenario_draw_without_contact_found()

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