import rospy
from std_msgs.msg import Float64MultiArray
import tf
import sys
import time

from cyclonedds.topic import Topic
from cyclonedds.pub import Publisher, DataWriter

from dds_utils import (
    DEFAULT_AGENT_TYPE,
    HEARTBEAT_PERIOD,
    HEARTBEAT_TOPIC,
    Heartbeat,
    ROS_TOPIC_TRANSFORMATION_MATRIX_ABS,
    RobotIdError,
    TransformMixin,
    best_effort_qos,
    create_domain_participant,
    dispose_participant,
    get_local_ip,
    require_robot_id_int,
)


class HeartbeatPublisher(TransformMixin):
    def __init__(self):
        rospy.init_node("dds_heartbeat_publisher", anonymous=True)

        self.init_transform_state()

        try:
            self.my_id_int = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)
        self.my_id = str(self.my_id_int)

        self.my_ip = get_local_ip()

        self.participant = create_domain_participant(domain_qos=False)
        self.publisher = Publisher(self.participant)

        self.heartbeat_topic = Topic(self.participant, HEARTBEAT_TOPIC, Heartbeat)
        self.heartbeat_writer = DataWriter(self.publisher, self.heartbeat_topic, qos=best_effort_qos)

        self.trans_listener = tf.TransformListener()

        rospy.Subscriber(ROS_TOPIC_TRANSFORMATION_MATRIX_ABS, Float64MultiArray, self.transformation_callback)

    def run(self):
        while not rospy.is_shutdown():

            # Get current position of the agent
            location_valid = False
            try:
                (translation, rotation) = self.trans_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
                x = translation[0]
                y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                theta = euler[2]

                transformed_point = self.transform_point([x, y, theta], forward=True)
                x, y, theta = transformed_point

                location_valid = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                # Location not available yet (not yet localized)
                x = 0.0
                y = 0.0
                theta = 0.0
                location_valid = False

            # Create a heartbeat message
            heartbeat = Heartbeat(
                self.my_id_int, int(time.time()), DEFAULT_AGENT_TYPE, self.my_ip, location_valid, x, y, theta, []
            )

            # Publish the heartbeat message
            self.heartbeat_writer.write(heartbeat)
            # print("Heartbeat sent")

            # Sleep for HEARTBEAT_PERIOD seconds
            rospy.sleep(HEARTBEAT_PERIOD)

    def shutdown(self):
        rospy.loginfo("Shutting down DDS heartbeat publisher...")
        self.heartbeat_writer = None
        self.publisher = None
        dispose_participant(self.participant)
        self.participant = None


if __name__ == "__main__":
    heartbeat_publisher = HeartbeatPublisher()
    time.sleep(11)
    rospy.on_shutdown(heartbeat_publisher.shutdown)
    heartbeat_publisher.run()
