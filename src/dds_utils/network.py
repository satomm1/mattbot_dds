import hashlib
import os
import socket

from cyclonedds.domain import DomainParticipantQos
from cyclonedds.util import duration

from .config import PARTICIPANT_LEASE_DURATION_MS


class RobotIdError(RuntimeError):
    """ROBOT_ID is missing, empty, or not a base-10 integer."""


def parse_robot_id_int(value) -> int:
    """Parse a robot id value to ``int``; raises ``ValueError`` if invalid."""
    if value is None:
        raise ValueError("robot id is None")
    s = str(value).strip()
    if not s:
        raise ValueError("robot id is empty")
    return int(s)


def require_robot_id_int() -> int:
    """
    Read ``ROBOT_ID`` from the environment and return it as ``int``.

    Raises:
        RobotIdError: if unset, whitespace-only, or not a base-10 integer.
    """
    raw = os.environ.get("ROBOT_ID")
    if raw is None or str(raw).strip() == "":
        raise RobotIdError(
            "ROBOT_ID environment variable must be set to a non-empty integer agent id"
        )
    try:
        return parse_robot_id_int(raw)
    except ValueError as exc:
        raise RobotIdError(f"ROBOT_ID must be a base-10 integer, got {raw!r}") from exc


def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    finally:
        s.close()


def get_robot_id():
    return os.environ.get("ROBOT_ID")


def hash_robot_id(robot_id):
    return int(hashlib.sha256(str(robot_id).encode()).hexdigest(), 16)


def make_participant_qos(lease_duration_ms=PARTICIPANT_LEASE_DURATION_MS):
    qos_profile = DomainParticipantQos()
    qos_profile.lease_duration = duration(milliseconds=lease_duration_ms)
    return qos_profile
