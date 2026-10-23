#!/usr/bin/env python3
"""Эмулятор шины Feetech на псевдо-порту.

Разбирает ровно тот протокол, который реализует servo_bus.cpp, — и точно так же,
как ожидает ответы.

Запрос чтения (8 байт), как собирает read_register:
    FF FF id 04 02 addr 02 cks,  cks = ~(сумма[2..6]) & 0xFF
    байт 3 (0x04) и байт 6 (0x02) — константы протокола: длина пакета и длина
    читаемых данных. Адрес целиком помещается в байт 5.
Ответ, который проверяет read_register:
    FF FF id ?? 00 data_lo data_hi cks, cks = ~(сумма[2..6]) & 0xFF
    проверяются только байты 0,1,2 и контрольная сумма; код ошибки — байт 4,
    значение читается из байт 5 и 6.

Запрос записи (write 9 байт), как собирает write_register:
    FF FF id 05 03 addr value_lo value_hi cks, cks = ~(сумма[2..7]) & 0xFF

Ответ на запись не читается, поэтому эмулятор его просто поглощает.

Нужен, чтобы проверить путь инициализации программ без робота: после перехода на
регистры именно чтение регистров решает, запустится программа или нет.
"""
from __future__ import annotations

import os
import pty
import select
import threading

REG_MIN_POSITION_LIMIT = 0x09
REG_MAX_POSITION_LIMIT = 0x0B
REG_GOAL_POSITION = 0x2A
REG_GOAL_VELOCITY = 0x2E
REG_PRESENT_POSITION = 0x38
REG_PRESENT_SPEED = 0x3A
REG_PRESENT_CURRENT = 0x3C
REG_PRESENT_LOAD = 0x3E
REG_TORQUE_ENABLE = 0x28

# Байт команды в пакете: 0x02 — чтение, 0x03 — запись.
INST_READ = 0x02
INST_WRITE = 0x03


def checksum(packet: bytes, first: int, last: int) -> int:
    return (~sum(packet[first:last + 1])) & 0xFF


class FakeServoBus:
    """Обслуживает шесть моторов на slave-конце псевдо-порта."""

    def __init__(self, min_positions, max_positions, present_positions=None,
                 follow_goal=False, deadband_steps=0, paper_steps=None):
        if not (len(min_positions) == len(max_positions) == 6):
            raise ValueError("ожидаются ровно шесть моторов")
        self.min_positions = [int(value) for value in min_positions]
        self.max_positions = [int(value) for value in max_positions]
        if present_positions is None:
            present_positions = [
                low + (high - low) // 2
                for low, high in zip(self.min_positions, self.max_positions)
            ]
        self.present_positions = [int(value) for value in present_positions]
        self.goal_positions = list(self.present_positions)
        # follow_goal: мотор мгновенно приходит к заданной цели. Обычно
        # этого не бывает — эмулятор стоит на месте, — но программам, которые
        # ждут фактического достижения точки (рисование круга), без этого
        # ждать бессмысленно: они упрутся в таймаут.
        self.follow_goal = follow_goal
        # deadband_steps: мёртвая зона мотора. Настоящий STS3215 до своей
        # цели не доходит — останавливается на зоне раньше, — и из-за этого
        # рисование круга останавливалось на первой точке. Здесь зона
        # воспроизводится в отсчётах мотора.
        self.deadband_steps = int(deadband_steps)
        # paper_steps: уровень бумаги по каждому мотору. Пока цель мотора
        # ниже бумаги, бумага давит в ответ, и это видно в регистре
        # нагрузки. По умолчанию бумага там, где рука стояла изначально.
        self.paper_steps = (list(paper_steps) if paper_steps is not None
                             else list(self.present_positions))
        # fail_load_reads: моторы не отвечают на чтение нагрузки. Настоящий
        # STS3215 так умеет. Нагрузка нужна только для уточнения высоты
        # бумаги и для проверки нажима, поэтому её отсутствие — не поломка:
        # программа продолжает рисовать по высоте из плоскости и считает
        # точки «без нажима».
        self.fail_load_reads = False
        # Сколько раз читался регистр нагрузки: по нему видно, пыталась ли
        # программа нагрузку читать.
        self.load_reads = 0
        self.writes: list[tuple[int, int, int]] = []
        # Состояние удержания каждого мотора: сервопривод по умолчанию
        # держит позицию, и программа читает это состояние обратно, чтобы
        # убедиться, что команда дошла.
        self.torque_switches = [1] * 6
        # Сколько ещё кадров записи переключателя момента шина «потеряет».
        # Настоящий STS3215 теряет кадры, если следующий приходит, пока
        # предыдущий не переварен, — программа обязана это переживать.
        self.drop_torque_writes = 0
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._master = -1
        self._slave = -1

# --- жизненный цикл -------------------------------------------------

    def start(self) -> str:
        """Поднимает пару pty и возвращает путь slave-устройства."""
        master, slave = pty.openpty()
        self._master = master
        # Slave держим открытым в родителе: закрытие последнего slave-дескриптора
        # удаляет узел /dev/pts/N, и программа не сможет его открыть.
        self._slave = slave

        # Общаемся через master: программа открывает slave, поэтому всё, что она
        # пишет, приходит на master, и наши ответы уходят туда же.
        self._stop.clear()
        self._thread = threading.Thread(target=self._serve, args=(master,),
                                        daemon=True)
        self._thread.start()
        return os.ttyname(slave)

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        for attribute in ("_master", "_slave"):
            descriptor = getattr(self, attribute)
            if descriptor >= 0:
                os.close(descriptor)
                setattr(self, attribute, -1)

    def __enter__(self) -> "FakeServoBus":
        return self

    def __exit__(self, *_exception) -> None:
        self.stop()

    # --- протокол -------------------------------------------------------

    def _serve(self, master_fd: int) -> None:
        # select, а не busy-loop: в неблокирующем режиме os.read возвращается
        # мгновенно и не отпускает GIL, из-за чего поток съедает время
        # основного процесса и тот не успевает ничего записать в порт.
        while not self._stop.is_set():
            try:
                readable, _, _ = select.select([master_fd], [], [], 0.05)
            except (OSError, ValueError):
                return
            if not readable:
                continue
            try:
                data = os.read(master_fd, 64)
            except BlockingIOError:
                continue
            except OSError:
                return
            if not data:
                continue
            reply = self._handle(data)
            if reply:
                try:
                    os.write(master_fd, reply)
                except OSError:
                    return

    def _handle(self, data: bytes) -> bytes | None:
        # В одном чтении может оказаться несколько пакетов: команды четырём
        # моторам подряд уходят в порт без пауз между ними. Раньше эмулятор
        # обрабатывал только первый пакет и выбрасывал остальные — поток
        # рассыпался, и следующий запрос читался по сдвигу.
        replies = b""
        offset = 0
        while offset < len(data) - 7:
            packet = self._frame(data, offset)
            if packet is None:
                offset += 1
                continue
            # Читание или запись определяется байтом команды, а не длиной
            # пакета: запись одного байта (переключатель момента) — тоже
            # восемь байт, как и чтение. По длине эмулятор считал такую
            # запись чтением, отвечал на неё, и поток рассыпался.
            if packet[4] == INST_READ:
                replies += self._handle_read(packet)
            else:
                self._handle_write(packet)
            offset += len(packet)
        return replies or None

    @staticmethod
    def _frame(data: bytes, offset: int) -> bytes | None:
        if data[offset] != 0xFF or data[offset + 1] != 0xFF:
            return None
        length = data[offset + 3] if offset + 3 < len(data) else 0
        total = length + 4
        if length not in (0x04, 0x05) or offset + total > len(data):
            return None
        packet = data[offset:offset + total]
        # Контрольная сумма лежит последним байтом и покрывает всё, кроме
        # заголовка и самой себя: байты [2 .. total-2].
        if checksum(packet, 2, total - 2) != packet[total - 1]:
            return None
        return packet

    def _handle_read(self, packet: bytes) -> bytes:
        servo_id = packet[2]
        # В запросе чтения адрес занимает только байт 5: байт 6 у read_register
        # задан константой 0x02 (длина читаемых данных), а не старшим байтом
        # адреса. Это видно по layout самого servo_bus.cpp.
        address = packet[5]
        index = servo_id - 1
        value = self._register(index, address) if 0 <= index < 6 else 0
        error = 0x00
        if address in (REG_PRESENT_LOAD, REG_PRESENT_CURRENT):
            self.load_reads += 1
        if self.fail_load_reads and address in (REG_PRESENT_LOAD,
                                                REG_PRESENT_CURRENT):
            # Мотор отвечает ошибкой: регистр он не отдаёт.
            error = 0x01
        reply = bytes([0xFF, 0xFF, servo_id, 0x00, error,
                        value & 0xFF, (value >> 8) & 0xFF, 0x00])
        return reply[:7] + bytes([checksum(reply, 2, 6)])

    def _handle_write(self, packet: bytes) -> None:
        servo_id = packet[2]
        address = packet[5]
        # Запись одного байта (переключатель момента) — восемь байт, и данные
        # занимают один байт; запись двух байт (цель, скорость) — девять, и
        # старший байт данных второй. Считать всегда два байта нельзя: в
        # восьмибайтовом пакете восьмой — контрольная сумма.
        if len(packet) == 8:
            value = packet[6]
        else:
            value = packet[6] | (packet[7] << 8)
        self.writes.append((servo_id, address, value))
        if 0 <= servo_id - 1 < 6 and address == REG_TORQUE_ENABLE:
            if self.drop_torque_writes > 0:
                # Кадр потерян: запись в порт удалась, а мотор её не увидел.
                self.drop_torque_writes -= 1
            else:
                self.torque_switches[servo_id - 1] = value
        if 0 <= servo_id - 1 < 6 and address == REG_GOAL_POSITION:
            self.goal_positions[servo_id - 1] = value
            if self.follow_goal:
                # Мотор идёт к цели, но останавливается на мёртвой зоне
                # раньше: до цели он не доходит.
                self.present_positions[servo_id - 1] = self._with_deadband(
                    servo_id - 1, value)
        return None

    def _with_deadband(self, index: int, goal: int) -> int:
        """Положение мотора с учётом мёртвой зоны: до цели он не доходит."""
        if self.deadband_steps <= 0:
            return goal
        present = self.present_positions[index]
        step = goal - present
        if abs(step) <= self.deadband_steps:
            return present
        return goal - (self.deadband_steps if step > 0 else -self.deadband_steps)

    def _register(self, index: int, address: int) -> int:
        if address == REG_MIN_POSITION_LIMIT:
            return self.min_positions[index]
        if address == REG_MAX_POSITION_LIMIT:
            return self.max_positions[index]
        if address == REG_PRESENT_POSITION:
            return self.present_positions[index]
        if address == REG_GOAL_POSITION:
            return self.goal_positions[index]
        if address == REG_GOAL_VELOCITY:
            return 0
        if address == REG_TORQUE_ENABLE:
            return self.torque_switches[index]
        if address == REG_PRESENT_LOAD:
            return self._load(index)
        if address == REG_PRESENT_CURRENT:
            return self._load(index)
        return 0

    def _load(self, index: int) -> int:
        """Нагрузка на мотор: бумага давит в ответ, когда цель ниже бумаги."""
        depth = self.paper_steps[index] - self.goal_positions[index]
        if depth <= 0:
            return 0
        return min(1000, depth * 4)

    # --- проверки -------------------------------------------------------

    def goal_position(self, servo_id: int) -> int:
        return self.goal_positions[servo_id - 1]

    def writes_to(self, address: int) -> list[tuple[int, int]]:
        return [(servo_id, value)
                for servo_id, written_address, value in self.writes
                if written_address == address]

    def torque_switch(self, servo_id: int) -> int | None:
        """Последнее записанное значение переключателя момента (1 — держит)."""
        values = [value for written_servo, address, value in self.writes
                  if address == REG_TORQUE_ENABLE and written_servo == servo_id]
        return values[-1] if values else None

    def torque_state(self, servo_id: int) -> int:
        """Действующее состояние удержания мотора (1 — держит позицию)."""
        return self.torque_switches[servo_id - 1]