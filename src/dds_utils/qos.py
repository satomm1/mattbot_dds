from cyclonedds.util import duration
from cyclonedds.core import Qos, Policy

from .config import PARTICIPANT_LEASE_DURATION_MS

reliable_qos = Qos(
    Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=10)),
    Policy.Durability.TransientLocal,
    Policy.History.KeepLast(depth=1),
)

# reliable_qos + Lifespan: directed one-shot replies (ledger sync responses). Late-joining readers
# (e.g. the requester after a restart) must not receive a stale reply from the writer history.
# Durability stays TransientLocal so the writer matches the TransientLocal DataTopic readers.
reliable_lifespan_qos = Qos(
    Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=10)),
    Policy.Durability.TransientLocal,
    Policy.History.KeepLast(depth=1),
    Policy.Lifespan(duration(seconds=5)),
)

# Entry/init need longer blocking time and are sensitive to discovery timing.
entry_init_reliable_qos = Qos(
    Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=500)),
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

# Live camera previews: drop late/lost frames instead of Reliable retransmit backlog.
image_qos = Qos(
    Policy.Reliability.BestEffort,
    Policy.Durability.Volatile,
    Policy.History.KeepLast(depth=1),
)
