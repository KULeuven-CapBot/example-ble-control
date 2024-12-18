#!/usr/bin/env python

import asyncio
import sys

from typing import List, NoReturn, Optional, assert_never
from enum import StrEnum

from argparse import ArgumentParser, Namespace

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
    SERVICE = normalize_uuid_32(0x00000030)  # CapBot control service
    DRIVE = normalize_uuid_32(0x00000031)  # Drive characteristic
    SPEED = normalize_uuid_32(0x00000032)  # Motor speed characteristic
    ANGLE = normalize_uuid_32(0x00000033)  # Motor angle characteristic
    VCAP = normalize_uuid_32(0x00000034)  # Capacitor voltage characteristic


class CapBotMotors:
    """
    Structure to hold an integer for each motor
    """

    def __init__(self, fl: int, fr: int, bl: int, br: int):
        self.front_left: int = fl
        self.front_right: int = fr
        self.back_left: int = bl
        self.back_right: int = br

    def __str__(self) -> str:
        return (
            "{"
            f"front_left: {self.front_left}, "
            f"front_right: {self.front_right}, "
            f"back_left: {self.back_left}, "
            f"back_right: {self.back_right}"
            "}"
        )


class CapBotSensors:
    """
    Structure to hold a robot's sensor values
    """

    def __init__(self, address: str):
        self.address: str = address
        self.voltage: float = 0
        self.angles: CapBotMotors = CapBotMotors(0, 0, 0, 0)
        self.speeds: CapBotMotors = CapBotMotors(0, 0, 0, 0)

    def __str__(self) -> str:
        return (
            f"Robot: {self.address}:\n"
            f"\tVcap: {self.voltage}V\n"
            f"\tSpeeds: {self.speeds}\n"
            f"\tAngles: {self.angles}"
        )

    def set_voltage(self, v: float) -> None:
        self.voltage = v

    def set_angles(self, a: CapBotMotors) -> None:
        self.angles = a

    def set_speeds(self, s: CapBotMotors) -> None:
        self.speeds = s


async def scan() -> List[BLEDevice]:
    devices: List[BLEDevice] = await BleakScanner(
        service_uuids=[CapBotUuid.SERVICE]
    ).discover(timeout=5)
    bots: List[BLEDevice] = []
    for device in devices:
        log.info(f"Found device: {device.address} - {device.name}")
        if await is_robot(device):
            bots.append(device)
    return bots


async def find(addr: str) -> Optional[BLEDevice]:
    device = await BleakScanner(
        service_uuids=[CapBotUuid.SERVICE]
    ).find_device_by_address(addr, timeout=5)
    if device is not None and await is_robot(device):
        return device
    return None


async def is_robot(device: BLEDevice) -> bool:
    return CapBotUuid.SERVICE in device.details["props"]["UUIDs"]


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
    raw = await client.read_gatt_char(CapBotUuid.VCAP)
    return int.from_bytes(raw, "little") / 1000.0


async def read_angle(client: BleakClient) -> CapBotMotors:
    if not client.is_connected:
        await client.connect()
    raw = await client.read_gatt_char(CapBotUuid.ANGLE)
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
    raw = await client.read_gatt_char(CapBotUuid.SPEED)
    dat = CapBotMotors(
        fl=int.from_bytes(raw[0:4], "little", signed=True),
        fr=int.from_bytes(raw[4:8], "little", signed=True),
        bl=int.from_bytes(raw[8:12], "little", signed=True),
        br=int.from_bytes(raw[12:16], "little", signed=True),
    )
    return dat


async def set_motors(client: BleakClient) -> None:
    if not client.is_connected:
        await client.connect()
    # Modify GATT service to accept 4-wheel individual drive
    await client.write_gatt_char(CapBotUuid.DRIVE, b"\x02", False)


# -------------------------------------------------------------------------- #
#                           Command Line Interface                           #
# -------------------------------------------------------------------------- #


def cli_scan() -> NoReturn:
    """
    # Scan for available robots

    Scans for available BLE devices, filter out robots and print their address
    to stdout
    """
    bots = asyncio.run(scan())

    if len(bots) == 0:
        log.print("No robots found")
        sys.exit(1)
    else:
        log.print("Found robots:")
        for bot in bots:
            log.print(f"\t{bot.address}")
        sys.exit(0)


def cli_sense(addr: Optional[str]) -> NoReturn:
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

        log.error("No suitable robots found")
        return None

    sensors = asyncio.run(sense(addr))

    if sensors is not None:
        log.print(str(sensors))
        sys.exit(0)
    else:
        sys.exit(1)


def cli_drive(addr: Optional[str]) -> NoReturn:
    """
    # Drive a given robot
    """

    async def drive(addr: Optional[str]) -> None:
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
            await set_motors(client)
        else:
            log.error("No suitable robots found")

    asyncio.run(drive(addr))
    sys.exit(0)


class Command(StrEnum):
    SCAN = "scan"
    DRIVE = "drive"
    SENSE = "sense"

    def exec(self, args: Namespace) -> None:
        log.enable_info(args.verbose)
        match self:
            case Command.SCAN:
                if args.address is not None:
                    log.warn("Address argument is unused for scan subcommand")
                cli_scan()
            case Command.DRIVE:
                cli_drive(args.address)
            case Command.SENSE:
                cli_sense(args.address)
            case unknown:  # Default: matches everything not specified above
                assert_never(unknown)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("command", choices=[cmd.lower() for cmd in Command])
    parser.add_argument("-a", "--address", type=str, required=False)
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()
    Command(args.command).exec(args)
