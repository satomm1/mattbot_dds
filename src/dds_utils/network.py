import hashlib
import os
import socket

from cyclonedds.domain import DomainParticipantQos
from cyclonedds.util import duration

from .config import PARTICIPANT_LEASE_DURATION_MS


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
