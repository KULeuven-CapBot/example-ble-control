#!/usr/bin/env python

"""Control CapBots using a custom BLE GATT service."""

import asyncio
import sys
from argparse import ArgumentParser, Namespace
from enum import StrEnum
from typing import List, NoReturn, Optional, assert_never

from bleak import BleakClient, BleakScanner
from bleak.backends.device import BLEDevice
from bleak.exc import BleakDeviceNotFoundError
from bleak.uuids import normalize_uuid_32

# -------------------------------------------------------------------------- #
#                                   Logging                                  #
# -------------------------------------------------------------------------- #


class Log:
    """Rudimentary logging interface."""

    def __init__(self) -> None:
        """Construct a simple logging interface."""
        self._info = False

    def enable_info(self, en: bool) -> None:
        """Enable printing info to stdout."""
        self._info = en

    def info(self, msg: str) -> None:
        """Log info message."""
        if self._info:
            print(f"[INFO] {msg}")

    def warn(self, msg: str) -> None:
        """Log warning."""
        print(f"[WARNING] {msg}")

    def error(self, msg: str) -> None:
        """Log error."""
        print(f"[ERROR] {msg}")

    def print(self, msg: str) -> None:
        """Unconditionally log an arbitrary message."""
        print(msg)


log = Log()

# -------------------------------------------------------------------------- #
#                             BLE related logic                              #
# -------------------------------------------------------------------------- #


class CapBotUuid(StrEnum):
    """Enum with different UUIDs for our "Robot Control Service"."""

    SERVICE = normalize_uuid_32(0x00000030)  # Robot Control Service
    DRIVE = normalize_uuid_32(0x00000031)  # Motor drive characteristic
    SPEED = normalize_uuid_32(0x00000032)  # Motor speed characteristic
    ANGLE = normalize_uuid_32(0x00000033)  # Motor angle characteristic
    # Voltage measurement characteristic
    VOLTAGE = normalize_uuid_32(0x00000034)


class CapBotMotors:
    """Structure to hold an integer for each motor."""

    def __init__(
        self,
        front_left: int,
        front_right: int,
        back_left: int,
        back_right: int,
    ):
        """Create robot "struct" with given values."""
        self.front_left: int = front_left
        self.front_right: int = front_right
        self.back_left: int = back_left
        self.back_right: int = back_right

    def __repr__(self) -> str:
        return (
            "{"
            f"front_left: {self.front_left}, "
            f"front_right: {self.front_right}, "
            f"back_left: {self.back_left}, "
            f"back_right: {self.back_right}"
            "}"
        )


class CapBotSensors:
    """Structure to hold a robot's sensor values."""

    def __init__(self, address: str):
        """Create a sensor "struct" for a robot with given address."""
        self.address: str = address
        self.voltage: float = 0
        self.angles: CapBotMotors = CapBotMotors(0, 0, 0, 0)
        self.speeds: CapBotMotors = CapBotMotors(0, 0, 0, 0)

    def __repr__(self) -> str:
        return (
            f"Robot: {self.address}:\n"
            f"\tVcap: {self.voltage}V\n"
            f"\tSpeeds: {self.speeds}\n"
            f"\tAngles: {self.angles}"
        )


async def scan() -> List[BLEDevice]:
    """Scan for BLE devices and filter out robots."""
    devices: List[BLEDevice] = await BleakScanner.discover(
        timeout=5, service_uuids=[CapBotUuid.SERVICE]
    )
    bots: List[BLEDevice] = []
    for device in devices:
        log.info(f"Found device: {device.address} - {device.name}")
        bots.append(device)
    return bots


async def find(addr: str) -> Optional[BLEDevice]:
    """Search for a robot with given address."""
    device = await BleakScanner.find_device_by_address(
        addr, timeout=5, service_uuids=[CapBotUuid.SERVICE]
    )
    if device is not None:
        return device
    return None


async def connect(device: BLEDevice) -> BleakClient:
    """Connect to the given device."""
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
    """Read the robot's capacitor voltage."""
    if not client.is_connected:
        await client.connect()
    raw = await client.read_gatt_char(CapBotUuid.VOLTAGE)
    log.info(f"Raw voltage data: {raw.hex()}")
    if len(raw) != 2:
        log.error("Received malformed voltage data")
    return int.from_bytes(raw, "little") / 1000.0


async def read_angle(client: BleakClient) -> CapBotMotors:
    """Get the angle of the robot's wheels."""
    if not client.is_connected:
        await client.connect()
    raw = await client.read_gatt_char(CapBotUuid.ANGLE)
    log.info(f"Raw angle data: {raw.hex()}")

    if len(raw) != 16:
        log.error("Received malformed angle data")
    dat = CapBotMotors(
        front_left=int.from_bytes(raw[0:4], "little", signed=True),
        front_right=int.from_bytes(raw[4:8], "little", signed=True),
        back_left=int.from_bytes(raw[8:12], "little", signed=True),
        back_right=int.from_bytes(raw[12:16], "little", signed=True),
    )
    return dat


async def read_speed(client: BleakClient) -> CapBotMotors:
    """Get the speed of the robot's wheels."""
    if not client.is_connected:
        await client.connect()
    raw = await client.read_gatt_char(CapBotUuid.SPEED)
    log.info(f"Raw speed data: {raw.hex()}")
    if len(raw) != 4:
        log.error("Received malformed speed data")
    dat = CapBotMotors(
        front_left=int.from_bytes(raw[0:1], "little", signed=True),
        front_right=int.from_bytes(raw[1:2], "little", signed=True),
        back_left=int.from_bytes(raw[2:3], "little", signed=True),
        back_right=int.from_bytes(raw[3:4], "little", signed=True),
    )
    return dat


async def set_motors(client: BleakClient, speeds: CapBotMotors, duration: int) -> None:
    """Set the target speed of the robot's wheels."""
    if not client.is_connected:
        await client.connect()
    raw = (
        (speeds.front_left).to_bytes(1, "little", signed=True)
        + (speeds.front_right).to_bytes(1, "little", signed=True)
        + (speeds.back_left).to_bytes(1, "little", signed=True)
        + (speeds.back_right).to_bytes(1, "little", signed=True)
        + duration.to_bytes(4, "little", signed=False)
    )
    log.info(f"Raw drive data: {raw.hex()}")
    assert len(raw) == 8
    await client.write_gatt_char(CapBotUuid.DRIVE, raw, False)


# -------------------------------------------------------------------------- #
#                           Command Line Interface                           #
# -------------------------------------------------------------------------- #


def cli_scan() -> NoReturn:
    """
    Scan for available robots.

    Scans for available BLE devices, filters out devices that expose a "robot
    control service" and prints their address to stdout
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
    """Read the different sensors on the robot."""

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
            sensed.voltage = await read_voltage(client)
            sensed.angles = await read_angle(client)
            sensed.speeds = await read_speed(client)
            await client.disconnect()
            return sensed

        log.error("No suitable robots found")
        return None

    sensors = asyncio.run(sense(addr))

    if sensors is not None:
        log.print(str(sensors))
        sys.exit(0)
    else:
        sys.exit(1)


def cli_drive(addr: Optional[str], speeds: List[int], duration: int) -> NoReturn:
    """Drive a robot."""

    async def drive(addr: Optional[str], speeds: CapBotMotors, duration: int) -> None:
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
            await set_motors(client, speeds, duration)
            await client.disconnect()
        else:
            log.error("No suitable robots found")

    asyncio.run(
        drive(
            addr,
            CapBotMotors(speeds[0], speeds[1], speeds[2], speeds[3]),
            duration,
        )
    )
    sys.exit(0)


class Command(StrEnum):
    """Possible subcommands of our command line interface."""

    SCAN = "scan"
    DRIVE = "drive"
    SENSE = "sense"

    def exec(self, cli_args: Namespace) -> None:
        """Execute the given subcommand."""
        log.enable_info(cli_args.verbose)
        match self:
            case Command.SCAN:
                if cli_args.address is not None:
                    log.warn("Address argument is unused when scanning")
                cli_scan()
            case Command.DRIVE:
                cli_drive(cli_args.address, cli_args.speed, cli_args.duration)
            case Command.SENSE:
                cli_sense(cli_args.address)
            case unknown:  # Default: matches everything not specified above
                assert_never(unknown)


if __name__ == "__main__":
    parser = ArgumentParser(description="BLE based controller for CapBots")
    parser.add_argument("-v", "--verbose", action="store_true", help="show verbose output")
    parser.add_argument("-a", "--address", type=str, required=False)
    subparsers = parser.add_subparsers(help="subcommand help", dest="command")

    scan_parser = subparsers.add_parser(
        Command.SCAN.lower(), description="Scan for available robots"
    )

    sense_parser = subparsers.add_parser(
        Command.SENSE.lower(), description="Request info from a robot"
    )

    drive_parser = subparsers.add_parser(
        Command.DRIVE.lower(), description="Send drive command to a robot"
    )
    drive_parser.add_argument("speed", nargs=4, type=int)
    drive_parser.add_argument(
        "-d",
        "--duration",
        type=int,
        help="time before stopping again (ms)",
        required=True,
    )

    args = parser.parse_args()
    Command(args.command).exec(args)
