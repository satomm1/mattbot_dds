"""Timing and system-wide defaults for DDS nodes."""

PARTICIPANT_LEASE_DURATION_MS = 30000

HEARTBEAT_PERIOD = 10  # seconds
HEARTBEAT_TIMEOUT = 31  # seconds (unchanged from prior hardcoded value)

LOCATION_PERIOD = 0.5  # seconds

INTER_DDS_WRITE_SLEEP_S = 0.01

INIT_RECENT_THRESHOLD_S = 10
INIT_MAX_RETRIES = 10

POSITION_INIT_RECENT_THRESHOLD_S = 5.0

DEFAULT_AGENT_TYPE = "robot"
