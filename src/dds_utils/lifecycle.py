"""Explicit DDS entity teardown for orderly process shutdown."""


def dispose_entity(entity) -> None:
    """Call ``close()`` on a Cyclone entity if the binding exposes it."""
    if entity is None:
        return
    close = getattr(entity, "close", None)
    if callable(close):
        try:
            close()
        except Exception:
            pass


def dispose_participant(participant) -> None:
    """
    Best-effort explicit DomainParticipant shutdown.

    Prefer calling this from ``rospy.on_shutdown`` after writers have flushed any
    final messages (e.g. exit DDS samples).
    """
    dispose_entity(participant)
