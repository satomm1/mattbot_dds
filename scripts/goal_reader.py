import rospy
from rospy_message_converter import message_converter
import tf
import rospkg
from mattbot_image_detection.msg import DetectedObject
from mattbot_dds.msg import AgentSubscription, AgentPath, AgentLocation
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Pose2D
from std_msgs.msg import Float64MultiArray, UInt32

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
import numpy as np

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
        self.send_unknown_images_pub = rospy.Publisher('/send_unknown_images', UInt32, queue_size=10)

        self.R = None
        self.t = None

        print("Created listener for topic", topic_id)

    def on_data_available(self, reader):
        for sample in reader.read():
            
            sending_agent = sample.sending_agent
            if sending_agent == int(self.my_id):
                # Ignore messages from me
                continue

            message_type = sample.message_type
            timestamp = sample.timestamp
            data = json.loads(sample.data)

            print("Received message from agent", sending_agent, "of type", message_type)

            # Process the message
            if message_type == "goal":

                # Transform the goal point to this occupancy grid
                x, y, theta = self.transform_point([data['x'], data['y'], data['theta']], forward=False)

                print(f"Received goal message from agent {sending_agent}: x={x}, y={y}, theta={theta}")
                goal_msg = Pose2D()
                goal_msg.x = x
                goal_msg.y = y
                goal_msg.theta = theta
                self.goal_pub.publish(goal_msg)
            elif message_type == "send_unknown_images":
                # Publish to topic to let the image detection node know to send unknown images
                # We need to send the agent id to which the images should be sent
                msg = UInt32()
                msg.data = sending_agent
                self.send_unknown_images_pub.publish(msg)
                

    def update_transformation_matrix(self, R, t):   
        
        self.R = R
        self.t = t

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

        self.R = None
        self.t = None
        transformation_subscriber = rospy.Subscriber('/transformation_matrix', Float64MultiArray, self.transformation_callback)

    def transformation_callback(self, data):

        # Get the transformation matrix
        transformation_matrix = data.data

        # Reshape the transformation matrix
        self.R = np.array(transformation_matrix[:4]).reshape(2, 2)
        self.t = np.array(transformation_matrix[4:])

        self.data_listener.update_transformation_matrix(self.R, self.t)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def shutdown(self):
        print("Shutting down DDS Goal Reader")


if __name__ == '__main__':
    goal_reader = GoalReader()
    rospy.on_shutdown(goal_reader.shutdown)
    goal_reader.run()