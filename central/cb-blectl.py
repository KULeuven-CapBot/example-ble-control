#!/usr/bin/env python

import asyncio
from typing import List, Dict, Optional, Tuple, assert_never
from argparse import ArgumentParser, Namespace
from enum import StrEnum

from bleak import BleakScanner, BleakClient
from bleak.backends.device import BLEDevice
from bleak.uuids import normalize_uuid_32
from bleak.exc import BleakDeviceNotFoundError

# -------------------------------------------------------------------------- #
#                                   Logging                                  #
# -------------------------------------------------------------------------- #


class Log:
    _info: bool

    def __init__(self) -> None:
        self._info = False

    def enable_info(self, en: bool) -> None:
        self._info = en

    def info(self, msg: str) -> None:
        if self._info:
            print(f"[INFO] {msg}")

    def warn(self, msg: str) -> None:
        print(f"[WARNING] {msg}")

    def error(self, msg: str) -> None:
        print(f"[ERROR] {msg}")

    def print(self, msg: str) -> None:
        print(msg)


log = Log()

# -------------------------------------------------------------------------- #
#                             BLE related logic                              #
# -------------------------------------------------------------------------- #


class CapBotUuid(StrEnum):
    service = normalize_uuid_32(0x00000030)  # CapBot control service
    drive = normalize_uuid_32(0x00000031)  # Drive characteristic
    speed = normalize_uuid_32(0x00000032)  # Motor speed characteristic
    angle = normalize_uuid_32(0x00000033)  # Motor angle characteristic
    vcap = normalize_uuid_32(0x00000034)  # Capacitor voltage characteristic


class CapBotMotors:
    """
    Structure to hold an integer for each motor
    """
    front_left: int
    front_right: int
    back_left: int
    back_right: int

    def __init__(self, fl: int, fr: int, bl: int, br: int):
        self.front_left = fl
        self.front_right = fr
        self.back_left = bl
        self.back_right = br

    def __str__(self) -> str:
        return f"{{front_left: {self.front_left}, front_right: {self.front_right}, back_left: {self.back_left}, back_right: {self.back_right}}}"


class CapBotSensors:
    """
    Structure to hold a robot's sensor values
    """

    address: str
    voltage: float
    angles: CapBotMotors
    speeds: CapBotMotors

    def __init__(self, address: str):
        self.address = address
        self.voltage = 0
        self.angles = CapBotMotors(0, 0, 0, 0)
        self.speeds = CapBotMotors(0, 0, 0, 0)

    def __str__(self) -> str:
        return f"Robot: {self.address}:\n\tVcap: {self.voltage}V\n\tSpeeds: {self.speeds}\n\tAngles: {self.angles}"

    def set_voltage(self, v: float) -> None:
        self.voltage = v

    def set_angles(self, a: CapBotMotors) -> None:
        self.angles = a

    def set_speeds(self, s: CapBotMotors) -> None:
        self.speeds = s


async def scan() -> List[BLEDevice]:
    devices: List[BLEDevice] = await BleakScanner(
        service_uuids=[CapBotUuid.service]
    ).discover(timeout=5)
    bots: List[BLEDevice] = []
    for device in devices:
        log.info(f"Found device: {device.address} - {device.name}")
        if await is_robot(device):
            bots.append(device)
    return bots


async def find(addr: str) -> Optional[BLEDevice]:
    device = await BleakScanner(
        service_uuids=[CapBotUuid.service]
    ).find_device_by_address(addr, timeout=5)
    if device is not None and await is_robot(device):
        return device
    else:
        return None


async def is_robot(device: BLEDevice) -> bool:
    return CapBotUuid.service in device.details["props"]["UUIDs"]


async def connect(device: BLEDevice) -> BleakClient:
    client = BleakClient(device, timeout=5)
    log.info(f"Connecting to bot: {device.address}")
    try:
        await client.connect()
    except BleakDeviceNotFoundError:
        log.error(f"Could not find bot: {device.address}")
    except TimeoutError:
        log.error(f"Could not connect to bot: {device.address}")
    return client


async def read_voltage(client: BleakClient) -> float:
    if not client.is_connected:
        await client.connect()
    raw = await client.read_gatt_char(CapBotUuid.vcap)
    return int.from_bytes(raw, "little") / 1000.0


async def read_angle(client: BleakClient) -> CapBotMotors:
    if not client.is_connected:
        await client.connect()
    raw = await client.read_gatt_char(CapBotUuid.angle)
    dat = CapBotMotors(
        fl=int.from_bytes(raw[0:4], "little", signed=True),
        fr=int.from_bytes(raw[4:8], "little", signed=True),
        bl=int.from_bytes(raw[8:12], "little", signed=True),
        br=int.from_bytes(raw[12:16], "little", signed=True),
    )
    return dat


async def read_speed(client: BleakClient) -> CapBotMotors:
    if not client.is_connected:
        await client.connect()
    raw = await client.read_gatt_char(CapBotUuid.speed)
    dat = CapBotMotors(
        fl=int.from_bytes(raw[0:4], "little", signed=True),
        fr=int.from_bytes(raw[4:8], "little", signed=True),
        bl=int.from_bytes(raw[8:12], "little", signed=True),
        br=int.from_bytes(raw[12:16], "little", signed=True),
    )
    return dat


# -------------------------------------------------------------------------- #
#                           Command Line Interface                           #
# -------------------------------------------------------------------------- #


def cli_scan() -> None:
    """
    # Scan for available robots

    Scans for available BLE devices, filter out robots and print their address to stdout
    """
    bots = asyncio.run(scan())

    if len(bots) == 0:
        log.print(f"No robots found")
        exit(1)
    else:
        log.print("Found robots:")
        for bot in bots:
            log.print(f"\t{bot.address}")
        exit(0)


def cli_sense(addr: str | None) -> None:
    """
    # Read the different sensors on the robot
    """

    async def sense(addr: Optional[str]) -> Optional[CapBotSensors]:
        # Determine robot device
        if addr is not None:
            # Search for bot with given address
            robot = await find(addr)
        else:
            # Use first bot in scan result
            log.info("No address specified, scanning for available robots...")
            devices = await scan()
            if len(devices) == 0:
                robot = None
            else:
                robot = devices[0]

        if robot is not None:
            client = await connect(robot)
            sensed = CapBotSensors(client.address)
            sensed.set_voltage(await read_voltage(client))
            sensed.set_angles(await read_angle(client))
            sensed.set_speeds(await read_speed(client))
            return sensed
        else:
            log.error("No suitable robots found")
            return None

    sensors = asyncio.run(sense(addr))

    if sensors is not None:
        log.print(str(sensors))
        exit(0)
    else:
        exit(1)


def cli_drive(addr: str | None) -> None:
    """
    # Drive a given robot
    """
    raise NotImplementedError


class Command(StrEnum):
    Scan = "scan"
    Drive = "drive"
    Sense = "sense"

    def exec(this, args: Namespace) -> None:
        log.enable_info(args.verbose)
        match this:
            case Command.Scan:
                if args.address is not None:
                    log.warn(f"Address argument is unused for scan subcommand")
                cli_scan()
            case Command.Drive:
                cli_drive(args.address)
            case Command.Sense:
                cli_sense(args.address)
            case unknown:  # Default case: matches everything not specified above
                assert_never(unknown)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("command", choices=[cmd.lower() for cmd in Command])
    parser.add_argument("-a", "--address", type=str, required=False)
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()
    Command(args.command).exec(args)
