import rospy
from std_msgs.msg import Float64MultiArray
import tf

from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.pub import Publisher, DataWriter
import time
import os

from dds_utils import (
    DEFAULT_AGENT_TYPE,
    HEARTBEAT_PERIOD,
    HEARTBEAT_TOPIC,
    Heartbeat,
    ROS_TOPIC_TRANSFORMATION_MATRIX_ABS,
    TransformMixin,
    best_effort_qos,
    get_local_ip,
)


class HeartbeatPublisher(TransformMixin):
    def __init__(self):
        rospy.init_node("dds_heartbeat_publisher", anonymous=True)

        self.init_transform_state()

        # Get robot ID, Hash, and IP Address
        self.my_id = os.environ.get("ROBOT_ID")

        self.my_ip = get_local_ip()

        # Create a DomainParticipant, Subscriber, and Publisher
        self.participant = DomainParticipant()
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
                int(self.my_id), int(time.time()), DEFAULT_AGENT_TYPE, self.my_ip, location_valid, x, y, theta, []
            )

            # Publish the heartbeat message
            self.heartbeat_writer.write(heartbeat)
            print("Heartbeat sent")

            # Sleep for HEARTBEAT_PERIOD seconds
            rospy.sleep(HEARTBEAT_PERIOD)

    def shutdown(self):
        rospy.loginfo("Shutting down DDS heartbeat publisher...")


if __name__ == "__main__":
    heartbeat_publisher = HeartbeatPublisher()
    time.sleep(11)
    rospy.on_shutdown(heartbeat_publisher.shutdown)
    heartbeat_publisher.run()
