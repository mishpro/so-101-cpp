#!/usr/bin/env python3
"""Создаёт URDF с границами суставов, взятыми из регистров сервоприводов.

so101_irs_ksant.urdf — копия исходного URDF, в которой у каждого
revolute-сустава атрибуты lower/upper блока <limit> заменены на реальный
диапазон мотора, прочитанный с шины. Получается единый источник знаний об
углах суставов для конкретного экземпляра робота: и Pinocchio, и код берут
границы из одного файла.

Границы читаются напрямую из регистров 0x09 и 0x0B каждого из шести моторов.
Ничего не записывается: скрипт только спрашивает диапазон, момент и ток не
трогает, робот при этом стоит как стоял.

Соответствие «servo id -> сустав» берётся из блоков <transmission> самого URDF
(<actuator name="motorN"><joint name="..."/>), а не дублируется здесь.

Привязка диапазона: lower сохраняется (соответствие «минимум мотора ↔ lower
URDF» не меняется), upper = lower + (max − min)·2π/4095 — ровно та формула,
по которой servo_bus.cpp переводит отсчёты мотора в градусы.

Всё, кроме шести строк <limit .../>, копируется байт-в-байт.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import xml.etree.ElementTree as ElementTree
from pathlib import Path

try:
    import serial
except ImportError:  # pragma: no cover - зависит от окружения
    serial = None

# Feetech STS3215: мотор считает угол как steps * 360 / 4095 (не 4096 — именно
# так считает servo_bus.cpp в read_joint_angle/write_joint_angle). Число шагов на
# полный оборот здесь 4095, иначе URDF разошёлся бы с прошивкой на 0.06 %.
COUNTS_PER_REVOLUTION = 4095.0

# Границы диапазона в регистрах STS3215: 0x09 и 0x0B — те же адреса, что
# использует servo_bus.cpp, поэтому скрипт видит ровно то, что видят
# программы управления.
REG_MIN_POSITION_LIMIT = 0x09
REG_MAX_POSITION_LIMIT = 0x0B

# Бодовая скорость шины сервоприводов: 1 Мбит/с, как в servo_bus.cpp.
# Это НЕ поле speed из send_config.json — там скорость МОТОРА (регистр
# GOAL_VELOCITY). На 600 бод, взятых из конфига, сервоприводы молчат.
BUS_BAUD = 1000000

JOINT_OPEN_RE = re.compile(r'<joint\s+name="([^"]+)"[^>]*\btype="revolute"')
JOINT_CLOSE = "</joint>"
LIMIT_RE = re.compile(
    r'\blower="[-+0-9.eE]+"\s+upper="[-+0-9.eE]+"'
    r'|\bupper="[-+0-9.eE]+"\s+lower="[-+0-9.eE]+"'
)


def expand_home(path: Path) -> Path:
    """Разворачивает ведущий ~ в пути."""
    text = str(path)
    if not text.startswith("~"):
        return path
    home = Path.home()
    if str(home) == "~":
        return path
    return Path(home / text[2:])


def checksum(packet: bytes, first: int, last: int) -> int:
    """Контрольная сумма пакета: ~(сумма байт [first..last]) & 0xFF."""
    return (~sum(packet[first:last + 1])) & 0xFF


def read_register(link, servo_id: int, address: int, timeout: float) -> int:
    """Читает 16-битный регистр мотора.

    Кадры повторяют servo_bus.cpp побайтово: запрос длиной 8 байт, где байт 3
    и байт 6 — константы протокола (длина пакета и длина читаемых данных), а
    адрес целиком помещается в байт 5. Ошибка проверяется по байту 4 ответа.
    """
    request = bytes([0xFF, 0xFF, servo_id, 0x04, 0x02, address, 0x02, 0x00])
    request = request[:7] + bytes([checksum(request, 2, 6)])
    link.reset_input_buffer()
    link.write(request)
    link.flush()

    response = link.read(8)
    if len(response) != 8:
        raise OSError(
            f"мотор {servo_id}: ответ на чтение регистра 0x{address:02X} "
            f"обрезан ({len(response)} из 8 байт). Молчание сервоприводов почти "
            f"всегда означает неверную бодовую скорость: нужна {BUS_BAUD} бод, "
            f"а поле speed из send_config.json относится к скорости мотора."
        )
    if response[0] != 0xFF or response[1] != 0xFF or response[2] != servo_id:
        raise OSError(
            f"мотор {servo_id}: ответ не адресован этому мотору: "
            f"{response.hex(' ')}"
        )
    if checksum(response, 2, 6) != response[7]:
        raise OSError(
            f"мотор {servo_id}: не сходится контрольная сумма ответа: "
            f"{response.hex(' ')}"
        )
    if response[4] != 0x00:
        raise OSError(
            f"мотор {servo_id}: сообщил об ошибке {response[4]} при чтении "
            f"регистра 0x{address:02X}"
        )
    return response[5] | (response[6] << 8)


def read_servo_limits_from_bus(
    port: str, baud: int, timeout: float
) -> dict[int, tuple[int, int]]:
    """Считывает границы всех шести моторов как {servo_id: (min, max)}.

    Момоторы на шине включены, но скрипт ничего в них не пишет: только чтение.
    """
    if serial is None:
        raise RuntimeError(
            "нужен pyserial: установите его (pip install pyserial) "
            "или запустите скрипт в окружении с ним"
        )

    try:
        link = serial.Serial(port, baud, timeout=timeout)
    except Exception as error:  # pyserial бросает разные исключения
        raise OSError(f"не удалось открыть {port} на скорости {baud}: {error}") from error

    limits: dict[int, tuple[int, int]] = {}
    try:
        for servo_id in range(1, 7):
            minimum = read_register(link, servo_id, REG_MIN_POSITION_LIMIT, timeout)
            maximum = read_register(link, servo_id, REG_MAX_POSITION_LIMIT, timeout)
            if not 0 <= minimum < maximum <= 0xFFFF:
                raise ValueError(
                    f"мотор {servo_id}: некорректный диапазон "
                    f"{minimum}..{maximum}"
                )
            limits[servo_id] = (minimum, maximum)
    finally:
        link.close()
    return limits


def read_servo_to_joint(source: Path) -> dict[int, str]:
    """Достаёт {servo_id: имя сустава} из блоков <transmission> исходного URDF."""
    root = ElementTree.parse(source).getroot()
    mapping: dict[int, str] = {}

    for transmission in root.findall("transmission"):
        actuator = transmission.find("actuator")
        joint = transmission.find("joint")
        if actuator is None or joint is None:
            continue
        name = actuator.get("name", "")
        joint_name = joint.get("name")
        if not name or not joint_name:
            continue
        match = re.fullmatch(r"motor(\d+)", name)
        if match is None:
            continue
        mapping[int(match.group(1))] = joint_name

    if len(mapping) != 6:
        found = ", ".join(f"motor{n}->{v}" for n, v in sorted(mapping.items()))
        raise ValueError(
            f"{source}: ожидалось 6 transmission с actuator motor1..motor6, "
            f"разобрано {len(mapping)} ({found})"
        )
    return mapping


class JointLimit:
    """Границы одного сустава: значения как в URDF и как как float."""

    def __init__(self, lower_text: str, upper_text: str) -> None:
        self.lower_text = lower_text
        self.upper_text = upper_text
        self.lower = float(lower_text)
        self.upper = float(upper_text)


def read_joint_limits(source: Path) -> dict[str, JointLimit]:
    """Читает текущие границы revolute-суставов из исходного URDF."""
    result: dict[str, JointLimit] = {}
    current: str | None = None

    for line in source.read_text(encoding="utf-8").splitlines():
        opened = JOINT_OPEN_RE.search(line)
        if opened is not None:
            current = opened.group(1)
            continue
        if current is None:
            continue
        if "<limit" in line:
            lower = re.search(r'\blower="([^"]+)"', line)
            upper = re.search(r'\bupper="([^"]+)"', line)
            if lower is None or upper is None:
                raise ValueError(f"{source}: у сустава {current} нет lower/upper")
            result[current] = JointLimit(lower.group(1), upper.group(1))
        if JOINT_CLOSE in line:
            current = None

    if len(result) != 6:
        raise ValueError(
            f"{source}: ожидалось 6 revolute-суставов с <limit>, разобрано {len(result)}"
        )
    return result


def sync(source: Path, limits: dict[int, tuple[int, int]]) -> tuple[str, list[dict]]:
    """Возвращает новый текст URDF и таблицу изменений."""
    servo_to_joint = read_servo_to_joint(source)
    joint_to_servo = {joint: servo for servo, joint in servo_to_joint.items()}
    old_limits = read_joint_limits(source)
    spans: dict[str, float] = {}

    for servo_id, (minimum, maximum) in limits.items():
        joint_name = servo_to_joint[servo_id]
        spans[joint_name] = (
            (maximum - minimum) * 2.0 * math.pi / COUNTS_PER_REVOLUTION
        )

    changes: list[dict] = []
    pending: str | None = None
    out: list[str] = []

    for line in source.read_text(encoding="utf-8").splitlines(keepends=True):
        opened = JOINT_OPEN_RE.search(line)
        if opened is not None:
            pending = opened.group(1)
        if pending is not None and "<limit" in line and pending in spans:
            old = old_limits[pending]
            upper = old.lower + spans[pending]
            # lower переносится исходной строкой, чтобы побайтово не менять формат.
            replacement = f'lower="{old.lower_text}" upper="{upper:.10f}"'
            new_line, count = LIMIT_RE.subn(replacement, line)
            if count != 1:
                raise ValueError(
                    f"{source}: у сустава {pending} не найдена ровно одна пара lower/upper"
                )
            changes.append(
                {
                    "servo_id": joint_to_servo[pending],
                    "joint": pending,
                    "old_lower": old.lower,
                    "old_upper": old.upper,
                    "new_lower": old.lower,
                    "new_upper": upper,
                    "old_deg": math.degrees(old.upper - old.lower),
                    "new_deg": math.degrees(upper - old.lower),
                }
            )
            out.append(new_line)
            if JOINT_CLOSE in line:
                pending = None
            continue
        if JOINT_CLOSE in line:
            pending = None
        out.append(line)

    if len(changes) != 6:
        raise ValueError(f"{source}: изменено {len(changes)} блоков <limit>, ожидалось 6")
    return "".join(out), changes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        default="~/develop/urdf/so101_new_calib.urdf",
        help="исходный URDF, из которого берутся lower и длина блоков <joint>",
    )
    parser.add_argument(
        "--output",
        default="~/develop/urdf/so101_irs_ksant.urdf",
        help="куда записать результат",
    )
    parser.add_argument(
        "--config",
        default="send_config.json",
        help="откуда взять порт по умолчанию",
    )
    parser.add_argument("--port", help="порт сервоприводов (перекрывает конфиг)")
    parser.add_argument(
        "--baud",
        type=int,
        default=BUS_BAUD,
        help="бодовая скорость шины (по умолчанию %(default)s; поле speed из "
             "конфига — это скорость мотора, а не шины)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.05,
        help="сколько ждать ответ мотора, секунд (по умолчанию %(default)s)",
    )
    args = parser.parse_args()

    source = expand_home(Path(args.source))
    output = expand_home(Path(args.output))

    if not source.is_file():
        print(f"Не найден исходный URDF: {source}", file=sys.stderr)
        return 1

    # Из конфига берётся только порт: скорость шины фиксирована и от настроек
    # мотора не зависит.
    port = args.port
    if port is None:
        try:
            config = json.loads(expand_home(Path(args.config)).read_text("utf-8"))
        except (OSError, ValueError) as error:
            print(f"Не удалось прочитать {args.config}: {error}", file=sys.stderr)
            return 1
        port = config.get("port")
    if not port:
        print("Не задан порт: укажите --port или исправьте конфиг", file=sys.stderr)
        return 1

    try:
        limits = read_servo_limits_from_bus(port, args.baud, args.timeout)
        text, changes = sync(source, limits)
    except (OSError, RuntimeError, ValueError, KeyError,
            ElementTree.ParseError) as error:
        print(f"Ошибка: {error}", file=sys.stderr)
        return 1

    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8")

    print(f"{'id':>3}  {'сустав':<14}{'было,°':>9}{'стало,°':>10}{'lower':>11}{'upper':>14}")
    for change in sorted(changes, key=lambda c: c["servo_id"]):
        print(
            f"{change['servo_id']:>3}  {change['joint']:<14}"
            f"{change['old_deg']:>9.2f}{change['new_deg']:>10.2f}"
            f"{change['new_lower']:>11.6f}{change['new_upper']:>14.10f}"
        )
    print(f"\nЗаписано: {output}")
    print(
        "ВНИМАНИЕ: после этого запустите ./build/send --show-limits и сверьте диапазоны,\n"
        "          либо просто запустите cartesian_control — он проверит согласованность сам."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())