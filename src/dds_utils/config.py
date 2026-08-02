"""Timing and system-wide defaults for DDS nodes."""

PARTICIPANT_LEASE_DURATION_MS = 30000

HEARTBEAT_PERIOD = 10  # seconds
AIR_QUALITY_PUBLISH_PERIOD_S = 10.0
HEARTBEAT_TIMEOUT = 31  # seconds (unchanged from prior hardcoded value)

LOCATION_PERIOD = 0.5  # seconds

INTER_DDS_WRITE_SLEEP_S = 0.01

INIT_RECENT_THRESHOLD_S = 10
INIT_MAX_RETRIES = 10
INIT_RETRY_SLEEP_S = 1.0
# Allow DDS discovery/matching before the first enter sample.
INIT_DISCOVERY_GRACE_S = 3.0
# Heartbeat nodes start after entry_exit has had time to join the swarm.
HEARTBEAT_STARTUP_DELAY_S = 6.0

POSITION_INIT_RECENT_THRESHOLD_S = 5.0

DEFAULT_AGENT_TYPE = "robot"
