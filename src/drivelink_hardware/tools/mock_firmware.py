#!/usr/bin/env python3
import argparse
import os
import select
import struct
import termios
import time

START = b"\xAA\xAA\xAA"

MSG_MOTOR_CONFIG = 0x21
MSG_MOTOR_RESP = 0x61
MSG_MOTOR_SETPOINT_STREAM = 0x80
MSG_MOTOR_TELEMETRY_STREAM = 0x81

MSG_IMU_CONFIG = 0x22
MSG_IMU_RESP = 0x42
MSG_IMU_STREAM = 0x82
MSG_TIME_SYNC_REQ = 0x20
MSG_TIME_SYNC_RESP = 0x60

SUB_MOTOR_SET_MODE = 0x04
SUB_MOTOR_SET_SETPOINT = 0x06
SUB_MOTOR_SET_TELEMETRY_GLOBAL = 0x09

SUB_IMU_SET_STREAM_ENABLED = 0x01
SUB_IMU_SET_STREAM_PERIOD_MS = 0x02
SUB_IMU_SET_STREAM_FIELD_MASK = 0x03

SETPOINT_KIND_VEL = 1


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


def make_frame(msg_type: int, payload: bytes) -> bytes:
    header = bytes([msg_type, len(payload)])
    csum = crc16(header + payload)
    return START + header + payload + struct.pack('<H', csum)


def configure_serial(fd: int, baud: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    speed_map = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
    }
    speed = speed_map.get(baud, termios.B115200)
    attrs[4] = speed
    attrs[5] = speed
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


class MockFirmware:
    def __init__(self, port: str, baud: int):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        configure_serial(self.fd, baud)

        self.rx = bytearray()

        self.motor_positions = {1: 0.0, 2: 0.0}
        self.motor_velocities = {1: 0.0, 2: 0.0}

        self.motor_stream_enabled = True
        self.imu_stream_enabled = True
        self.imu_period_s = 0.01
        self.motor_period_s = 0.1

        self.last_motor_tx = time.monotonic()
        self.last_imu_tx = time.monotonic()
        self.last_time_sync_tx = time.monotonic()
        self.time_sync_period_s = 1.0
        self.time_sync_request_id = 1

    def close(self) -> None:
        os.close(self.fd)

    def send(self, msg_type: int, payload: bytes) -> None:
        frame = make_frame(msg_type, payload)
        offset = 0
        retries = 0
        while offset < len(frame):
            try:
                n = os.write(self.fd, frame[offset:])
                if n <= 0:
                    break
                offset += n
            except BlockingIOError:
                retries += 1
                if retries > 10:
                    return
                select.select([], [self.fd], [], 0.01)

    def send_motor_response(self, version: int, subcmd: int, motor_id: int, result: int = 0) -> None:
        payload = struct.pack('<BBBBB', version, subcmd, motor_id, result, 0)
        self.send(MSG_MOTOR_RESP, payload)

    def send_imu_response(self, version: int, subcmd: int, result: int = 0) -> None:
        payload = struct.pack('<BBB', version, subcmd, result)
        self.send(MSG_IMU_RESP, payload)

    def send_motor_telemetry(self) -> None:
        now_us = int(time.time() * 1e6)
        dt = self.motor_period_s
        for motor_id in self.motor_positions:
            self.motor_positions[motor_id] += self.motor_velocities[motor_id] * dt

        payload = bytearray(struct.pack('<BBQ', 1, 2, now_us))
        for motor_id in (1, 2):
            payload.extend(struct.pack('<BBff', motor_id, 0x03, self.motor_positions[motor_id], self.motor_velocities[motor_id]))
        self.send(MSG_MOTOR_TELEMETRY_STREAM, bytes(payload))

    def send_imu_telemetry(self) -> None:
        now_us = int(time.time() * 1e6)
        field_mask = 0x0026  # quaternion + linear_accel + raw_gyro
        quat = (16384, 0, 0, 0)  # w, x, y, z
        lin_acc = (0, 0, 100)
        raw_gyro = (0, 0, 0)
        payload = struct.pack('<BHQhhhhhhhhhh', 1, field_mask, now_us, *quat, *lin_acc, *raw_gyro)
        self.send(MSG_IMU_STREAM, payload)

    def send_time_sync_request(self) -> None:
        now_us = int(time.time() * 1e6)
        payload = struct.pack('<BBHq', 1, 0, self.time_sync_request_id, now_us)
        self.send(MSG_TIME_SYNC_REQ, payload)
        self.time_sync_request_id = (self.time_sync_request_id + 1) & 0xFFFF

    def parse_frames(self):
        while True:
            start = self.rx.find(START)
            if start < 0:
                if len(self.rx) > 2:
                    del self.rx[:-2]
                return
            if start > 0:
                del self.rx[:start]

            if len(self.rx) < 7:
                return

            msg_type = self.rx[3]
            length = self.rx[4]
            frame_len = 3 + 1 + 1 + length + 2
            if len(self.rx) < frame_len:
                return

            payload = bytes(self.rx[5:5 + length])
            recv_crc = struct.unpack('<H', self.rx[5 + length:7 + length])[0]
            calc_crc = crc16(bytes([msg_type, length]) + payload)
            del self.rx[:frame_len]

            if recv_crc != calc_crc:
                continue

            self.handle_frame(msg_type, payload)

    def handle_frame(self, msg_type: int, payload: bytes) -> None:
        if msg_type == MSG_MOTOR_CONFIG:
            if len(payload) < 3:
                return
            version, subcmd, motor_id = struct.unpack('<BBB', payload[:3])

            if subcmd == SUB_MOTOR_SET_SETPOINT and len(payload) >= 8:
                kind = payload[3]
                value = struct.unpack('<f', payload[4:8])[0]
                if kind == SETPOINT_KIND_VEL:
                    self.motor_velocities[motor_id] = value
                print(f"MOTOR_SET_SETPOINT motor={motor_id} kind={kind} value={value:.4f}")

            if subcmd == SUB_MOTOR_SET_MODE and len(payload) >= 4:
                mode = payload[3]
                print(f"MOTOR_SET_MODE motor={motor_id} mode={mode}")

            if subcmd == SUB_MOTOR_SET_TELEMETRY_GLOBAL and len(payload) >= 5:
                enabled = payload[3] != 0
                self.motor_stream_enabled = enabled
                print(f"MOTOR_TELEMETRY_GLOBAL enabled={enabled}")

            self.send_motor_response(version, subcmd, motor_id, 0)
            return

        if msg_type == MSG_MOTOR_SETPOINT_STREAM:
            if len(payload) < 2:
                return
            _, count = struct.unpack('<BB', payload[:2])
            offset = 2
            for _ in range(count):
                if offset + 6 > len(payload):
                    break
                motor_id, kind = struct.unpack('<BB', payload[offset:offset + 2])
                value = struct.unpack('<f', payload[offset + 2:offset + 6])[0]
                if kind == SETPOINT_KIND_VEL:
                    self.motor_velocities[motor_id] = value
                print(f"SETPOINT_STREAM motor={motor_id} kind={kind} value={value:.4f}")
                offset += 6
            return

        if msg_type == MSG_IMU_CONFIG:
            if len(payload) < 2:
                return
            version, subcmd = struct.unpack('<BB', payload[:2])
            if subcmd == SUB_IMU_SET_STREAM_ENABLED and len(payload) >= 3:
                self.imu_stream_enabled = payload[2] != 0
                print(f"IMU_STREAM_ENABLED {self.imu_stream_enabled}")
            elif subcmd == SUB_IMU_SET_STREAM_PERIOD_MS and len(payload) >= 4:
                period_ms = struct.unpack('<H', payload[2:4])[0]
                self.imu_period_s = max(0.005, period_ms / 1000.0)
                print(f"IMU_STREAM_PERIOD_MS {period_ms}")
            elif subcmd == SUB_IMU_SET_STREAM_FIELD_MASK and len(payload) >= 4:
                field_mask = struct.unpack('<H', payload[2:4])[0]
                print(f"IMU_STREAM_FIELD_MASK 0x{field_mask:04x}")
            self.send_imu_response(version, subcmd, 0)
            return

        if msg_type == MSG_TIME_SYNC_RESP:
            if len(payload) != 30:
                return
            version, status, request_id = struct.unpack('<BBH', payload[:4])
            client_send_us, server_rx_us, server_tx_us = struct.unpack('<qqq', payload[4:28])
            print(
                f"TIME_SYNC_RESPONSE ver={version} status={status} req={request_id} "
                f"client_send_us={client_send_us} server_rx_us={server_rx_us} server_tx_us={server_tx_us}")
            return

    def spin(self) -> None:
        print("mock_firmware ready")
        while True:
            now = time.monotonic()
            if self.motor_stream_enabled and (now - self.last_motor_tx) >= self.motor_period_s:
                self.send_motor_telemetry()
                self.last_motor_tx = now

            if self.imu_stream_enabled and (now - self.last_imu_tx) >= self.imu_period_s:
                self.send_imu_telemetry()
                self.last_imu_tx = now

            if (now - self.last_time_sync_tx) >= self.time_sync_period_s:
                self.send_time_sync_request()
                self.last_time_sync_tx = now

            rlist, _, _ = select.select([self.fd], [], [], 0.01)
            if self.fd in rlist:
                try:
                    chunk = os.read(self.fd, 4096)
                except BlockingIOError:
                    chunk = b""
                if chunk:
                    self.rx.extend(chunk)
                    self.parse_frames()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', required=True)
    parser.add_argument('--baud', type=int, default=115200)
    args = parser.parse_args()

    fw = MockFirmware(args.port, args.baud)
    try:
        fw.spin()
    except KeyboardInterrupt:
        pass
    finally:
        fw.close()


if __name__ == '__main__':
    main()
