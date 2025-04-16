import rospy
from std_msgs.msg import Float64MultiArray
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
import numpy as np

from dds_utils import Location

LOCATION_PERIOD = 0.75    # seconds

class LocationPublisher:
    def __init__(self):
        rospy.init_node('dds_location', anonymous=True)

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

        self.location_topic = Topic(self.participant, 'LocationTopic' + str(self.my_id), Location)
        self.location_writer = DataWriter(self.publisher, self.location_topic, qos=self.best_effort_qos)

        self.trans_listener = tf.TransformListener()

        self.R = None
        self.t = None
        transformation_subscriber = rospy.Subscriber('transformation_matrix', Float64MultiArray, self.transformation_callback)

    def transformation_callback(self, data):
        # Get the transformation matrix
        transformation_matrix = data.data

        # Reshape the transformation matrix
        self.R = np.array(transformation_matrix[:4]).reshape(2, 2)
        self.t = np.array(transformation_matrix[4:])

    def transform_point(self, point, forward=True):
        if self.R is None:
            return point

        point_xy = np.array([point[0], point[1]])
        if forward:
            new_point_xy = self.R @ point_xy + self.t
            new_point_theta = point[2] + np.arctan2(self.R[1, 0], self.R[0, 0])
            return np.concatenate((new_point_xy, [new_point_theta]))
        else:
            new_point_xy = self.R.T @ (point_xy - self.t)
            new_point_theta = point[2] - np.arctan2(self.R[1, 0], self.R[0, 0])
            return np.concatenate((new_point_xy, [new_point_theta]))

    def run(self):
        while not rospy.is_shutdown():

            # Get current position of the agent
            try:
                (translation, rotation) = self.trans_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
                x = translation[0]
                y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                theta = euler[2]

                transformed_point = self.transform_point([x, y, theta])
                x_new, y_new, theta_new = transformed_point
                location = Location(int(self.my_id), int(time.time()), x_new, y_new, theta_new)
                self.location_writer.write(location)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                # Location not available yet (not yet localized)
                pass

            # Sleep for LOCATION_PERIOD seconds
            rospy.sleep(LOCATION_PERIOD)

    def shutdown(self):
        rospy.loginfo("Shutting down DDS location publisher...")

if __name__ == '__main__':
    location_publisher = LocationPublisher()
    rospy.on_shutdown(location_publisher.shutdown)
    location_publisher.run()
