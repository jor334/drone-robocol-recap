"""Command and utility helpers for DroneKit missions."""

import math
import time

from pymavlink import mavutil


def wait_until(condition, description, timeout=120):
    """Wait until ``condition`` is true or raise ``TimeoutError``."""
    start = time.time()
    while not condition():
        if time.time() - start > timeout:
            raise TimeoutError(f"Condition not met: {description}")
        time.sleep(0.5)


def distance_m(a, b):
    """Approximate horizontal distance in meters between nearby GPS positions."""
    north = (b.lat - a.lat) * 111_320
    east = (b.lon - a.lon) * 111_320 * math.cos(math.radians(a.lat))
    return math.hypot(north, east)


def go_to(vehicle, destination, description, speed=3, tolerance=1.0, timeout=60):
    """Send the vehicle to ``destination`` and wait until it is within ``tolerance`` meters."""
    vehicle.simple_goto(destination, groundspeed=speed)
    wait_until(
        lambda: distance_m(vehicle.location.global_relative_frame, destination) < tolerance,
        description,
        timeout,
    )


def turn_to(vehicle, heading, speed=30, tolerance=5, timeout=30):
    """Turn to an absolute heading in degrees (0 = north, 180 = south)."""
    message = vehicle.message_factory.command_long_encode(
        0,
        0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,
        speed,
        1,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(message)
    # Normalize the angular difference so 359° and 1° are treated as nearby.
    wait_until(
        lambda: abs((vehicle.heading - heading + 180) % 360 - 180) < tolerance,
        f"heading {heading}°",
        timeout,
    )
