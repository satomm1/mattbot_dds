import rospy
import tf

from cyclonedds.domain import DomainParticipant, DomainParticipantQos
from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.util import duration
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import sequence
from cyclonedds.core import Qos, Policy, Listener
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsParticipant

from dataclasses import dataclass
import time
import os

HEARTBEAT_PERIOD = 10    # seconds
HEARTBEAT_TIMEOUT = 31  # seconds

@dataclass
class Heartbeat(IdlStruct):
    """
    Represents a heartbeat message from an agent.
    
    Attributes:
        agent_id (int): The ID of the agent sending the heartbeat.
        timestamp (int): The timestamp of the heartbeat message.
    """
    agent_id: int
    timestamp: int
    location_valid: bool
    x: float
    y: float
    theta: float

class HeartbeatPublisher:
    def __init__(self):
        rospy.init_node('dds_heartbeat', anonymous=True)

        # Get robot ID, Hash, and IP Address
        self.my_id = os.environ.get('ROBOT_ID')

        # Reliable qos
        self.reliable_qos = Qos(
            Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=1)),
            Policy.Durability.TransientLocal,
            Policy.History.KeepLast(depth=1)
        )

        self.best_effort_qos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.Liveliness.ManualByParticipant(lease_duration=duration(milliseconds=30000))
            # Policy.Deadline(duration(milliseconds=1000))
            # Policy.History.KeepLast(depth=1)
        )

        self.lease_duration_ms = 30000
        qos_profile = DomainParticipantQos()
        qos_profile.lease_duration = duration(milliseconds=self.lease_duration_ms)

        # Create a DomainParticipant, Subscriber, and Publisher
        self.participant = DomainParticipant()
        self.publisher = Publisher(self.participant)

        self.heartbeat_topic = Topic(self.participant, 'HeartbeatTopic', Heartbeat)
        self.heartbeat_writer = DataWriter(self.publisher, self.heartbeat_topic, qos=self.best_effort_qos)

        self.trans_listener = tf.TransformListener()

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
                location_valid = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                # Location not available yet (not yet localized)
                x = 0.0
                y = 0.0
                theta = 0.0
                location_valid = False

            # Create a heartbeat message
            heartbeat = Heartbeat(int(self.my_id), int(time.time()), location_valid, x, y, theta)

            # Publish the heartbeat message
            self.heartbeat_writer.write(heartbeat)
            print("Heartbeat sent")

            # Sleep for HEARTBEAT_PERIOD seconds
            rospy.sleep(HEARTBEAT_PERIOD)

    def shutdown(self):
        rospy.loginfo("Shutting down DDS heartbeat publisher...")

if __name__ == '__main__':
    time.sleep(HEARTBEAT_PERIOD)
    heartbeat_publisher = HeartbeatPublisher()
    rospy.on_shutdown(heartbeat_publisher.shutdown)
    heartbeat_publisher.run()
