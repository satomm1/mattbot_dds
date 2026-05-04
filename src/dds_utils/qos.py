from cyclonedds.util import duration
from cyclonedds.core import Qos, Policy

from .config import PARTICIPANT_LEASE_DURATION_MS

reliable_qos = Qos(
    Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=10)),
    Policy.Durability.TransientLocal,
    Policy.History.KeepLast(depth=1),
)

best_effort_qos = Qos(
    Policy.Reliability.BestEffort,
    Policy.Durability.Volatile,
    Policy.Liveliness.ManualByParticipant(
        lease_duration=duration(milliseconds=PARTICIPANT_LEASE_DURATION_MS)
    ),
)
