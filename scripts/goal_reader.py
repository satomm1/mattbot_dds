import rospy
from rospy_message_converter import message_converter
import tf
import rospkg
from mattbot_image_detection.msg import DetectedObject
from mattbot_dds.msg import AgentSubscription, AgentPath, AgentLocation
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Pose2D

from cyclonedds.domain import DomainParticipant
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
import hashlib
import socket
import json
import requests

##################################################
# This script is used to process and send DataMessages.
# The agent_entry_exit handles entry/exit/determining
# what agents to subscribe to, while this script handles
# the actual communication of data between agents.
##################################################

@dataclass
class DataMessage(IdlStruct):
    message_type: str
    sending_agent: int
    timestamp: int
    data: str

@dataclass
class Location(IdlStruct):
    """
    Represents the location of an agent.

    Attributes:
        agent_id (int): The ID of the agent.
        timestamp (int): The timestamp of the location message.
        x (float): The x-coordinate of the agent.
        y (float): The y-coordinate of the agent.
        theta (float): The orientation of the agent.
    """
    agent_id: int
    timestamp: int
    x: float
    y: float
    theta: float

class SelfDataListener(Listener):

    def __init__(self, my_id, topic_id):
        super().__init__()
        self.my_id = my_id
        self.topic_id = topic_id
        self.goal_pub = rospy.Publisher('/external_goal', Pose2D, queue_size=10)

    def on_data_available(self, reader):
        for sample in reader.read():
            
            sending_agent = sample.sending_agent
            if sending_agent == int(self.my_id):
                # Ignore messages from me
                continue

            message_type = sample.message_type
            timestamp = sample.timestamp
            data = json.loads(sample.data)

            # Process the message
            if message_type == "goal":
                print(f"Received goal message from agent {sending_agent}: x={data['x']}, y={data['y']}, theta={data['theta']}")
                goal_msg = Pose2D()
                goal_msg.x = data['x']
                goal_msg.y = data['y']
                goal_msg.theta = data['theta']
                self.goal_pub.publish(goal_msg)

class GoalReader:

    def __init__(self):
        
        rospy.init_node('dds_goal_reader', anonymous=True)

        # Get robot ID, Hash, and IP Address
        self.my_id = os.environ.get('ROBOT_ID')
        self.agents_subscribed = set()

        # Reliable qos
        self.reliable_qos = Qos(
            Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=10)),
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

        # Create a DomainParticipant, Subscriber, and Publisher
        self.participant = DomainParticipant()
        self.subscriber = Subscriber(self.participant)
        self.publisher = Publisher(self.participant)

        # Create my data topic
        self.data_topic = Topic(self.participant, 'DataTopic' + str(self.my_id), DataMessage)
        self.data_listener = SelfDataListener(self.my_id, self.my_id)
        self.data_reader = DataReader(self.subscriber, self.data_topic, listener=self.data_listener, qos=self.reliable_qos)

    def run(self):
        while not rospy.is_shutdown():
            time.sleep(1)

    def shutdown(self):
        print("Shutting down DDS Goal Reader")


if __name__ == '__main__':
    time.sleep(5)  # Wait
    goal_reader = GoalReader()
    rospy.on_shutdown(goal_reader.shutdown)
    goal_reader.run()