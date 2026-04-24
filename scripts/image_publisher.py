import rospy
from rospy_message_converter import message_converter
import tf
import rospkg
from sensor_msgs.msg import Image

from cyclonedds.domain import DomainParticipant, DomainParticipantQos
from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.util import duration
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import sequence
from cyclonedds.core import Qos, Policy, Listener
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsParticipant

import time
import os
import hashlib
import socket
import json
import requests
import numpy as np

from dds_utils import ImageMessage, reliable_qos, best_effort_qos


class ImagePublisher:

    def __init__(self):
        rospy.init_node('dds_image_publisher', anonymous=True)

        self.my_id = os.environ.get('ROBOT_ID')

        self.lease_duration_ms = 30000
        qos_profile = DomainParticipantQos()
        qos_profile.lease_duration = duration(milliseconds=self.lease_duration_ms)

        # Create a DomainParticipant, Subscriber, and Publisher
        self.participant = DomainParticipant(qos=qos_profile)
        self.publisher = Publisher(self.participant)

        # Create my image topic
        self.image_topic = Topic(self.participant, 'ImageTopic' + str(self.my_id), ImageMessage)
        self.image_writer = DataWriter(self.publisher, self.image_topic, qos=reliable_qos)

        # Create a subscriber for the image topic
        self.image_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        # Convert ROS Image message to DDS ImageMessage
        image_message = ImageMessage(
            agent_id=int(self.my_id),
            timestamp=int(time.time()),  # Convert to milliseconds
            data=msg.data,
            width=msg.width,
            height=msg.height,
            encoding=msg.encoding
        )

        # Publish the image message
        self.image_writer.write(image_message)
        rospy.loginfo(f"Published image from agent {self.my_id} at timestamp {image_message.timestamp}")

    def run(self):
        rospy.loginfo("Image Publisher is running...")
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down Image Publisher...")
        self.image_subscriber.unregister()
        rospy.loginfo("Image Publisher shutdown complete.")


if __name__ == '__main__':
    image_publisher = ImagePublisher()
    rospy.on_shutdown(image_publisher.shutdown)
    image_publisher.run()