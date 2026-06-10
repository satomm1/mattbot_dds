import rospy


class DdsLogger:
    """Prefixed rospy logging for mattbot_dds nodes (grep-friendly: [DDS component])."""

    __slots__ = ("_tag",)

    def __init__(self, component):
        self._tag = f"[DDS {component}]"

    def info(self, msg, *args):
        rospy.loginfo("%s " + msg, self._tag, *args)

    def debug(self, msg, *args):
        rospy.logdebug("%s " + msg, self._tag, *args)

    def warn(self, msg, *args):
        rospy.logwarn("%s " + msg, self._tag, *args)
