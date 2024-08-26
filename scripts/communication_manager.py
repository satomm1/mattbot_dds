import rospy
from rospy_message_converter import message_converter
import tf
import rospkg
from mattbot_image_detection.msg import DetectedObject
from mattbot_dds.msg import AgentSubscription, AgentPath, AgentLocation
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose

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

    def on_data_available(self, reader):
        for sample in reader.read():
            
            sending_agent = sample.sending_agent
            if sending_agent == int(self.my_id):
                # Ignore messages from me
                continue

            message_type = sample.message_type
            timestamp = sample.timestamp
            data = json.loads(sample.data)

            if self.topic_id == self.my_id:  # This is my topic, just a check
                # Process the message
                if message_type == "goal":
                    pass

class OtherDataListener(Listener):

    def __init__(self, my_id, topic_id, object_publisher, path_publisher):
        super().__init__()
        self.my_id = my_id
        self.topic_id = topic_id
        self.object_publisher = object_publisher
        self.path_publisher = path_publisher

    def on_data_available(self, reader):
        for sample in reader.read():
            
            sending_agent = sample.sending_agent

            # Check if the message is from the agent
            if sending_agent == self.topic_id:
                message_type = sample.message_type
                timestamp = sample.timestamp
                data = json.loads(sample.data)
                if message_type == "detected_object":
                    new_object = message_converter.convert_dictionary_to_ros_message('mattbot_image_detection/DetectedObject', data)
                    self.object_publisher.publish(new_object)
                    print("Received object from agent " + str(self.topic_id))
                elif message_type == "path":
                    new_path = message_converter.convert_dictionary_to_ros_message('nav_msgs/Path', data)
                    new_agent_path = AgentPath()
                    new_agent_path.agentID.data = self.topic_id
                    new_agent_path.path = new_path
                    self.path_publisher.publish(new_agent_path)
                    print("Received path from agent " + str(self.topic_id))
            else:
                # This was a message to the agent, we can safely ignore
                continue

class LocationListener(Listener):
    """
    Listener class that handles location data for agents.

    Attributes:
        my_id (int): The ID of the listener.
        agent_ids (list): List of agent IDs.
        location (tuple): Tuple to store agent location.

    Methods:
        on_data_available(reader): Callback method called when data is available.
    """

    def __init__(self, my_id, topic_id, location_publisher):
        super().__init__()
        self.my_id = my_id
        self.topic_id = topic_id
        self.location_publisher = location_publisher

    def on_data_available(self, reader):
        for sample in reader.read():

            # Ignore messages from self
            if sample.agent_id == int(self.my_id):
                continue

            agent_location = AgentLocation()
            agent_location.agentID.data = sample.agent_id

            agent_pose = Pose()
            agent_pose.position.x = sample.x
            agent_pose.position.y = sample.y

            quaternion = tf.transformations.quaternion_from_euler(0, 0, sample.theta)
            agent_pose.orientation.x = quaternion[0]
            agent_pose.orientation.y = quaternion[1]
            agent_pose.orientation.z = quaternion[2]
            agent_pose.orientation.w = quaternion[3]

            agent_location.pose = agent_pose
            self.location_publisher.publish(agent_location)

class CommManager:

    def __init__(self):
        
        rospy.init_node('dds_comm_manager', anonymous=True)

        # Get robot ID, Hash, and IP Address
        self.my_id = os.environ.get('ROBOT_ID')
        self.agents_subscribed = set()

        # Reliable qos
        self.reliable_qos = Qos(
            Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=1)),
            Policy.Durability.TransientLocal,
            Policy.History.KeepLast(depth=1)
        )

        self.best_effort_qos = Qos(
            Policy.Reliability.BestEffort,
            Policy.Durability.Volatile,
            Policy.Deadline(duration(milliseconds=1000))
            # Policy.History.KeepLast(depth=1)
        )

        # Create a DomainParticipant, Subscriber, and Publisher
        self.participant = DomainParticipant()
        self.subscriber = Subscriber(self.participant)
        self.publisher = Publisher(self.participant)

        # Create my data topic
        self.data_topic = Topic(self.participant, 'DataTopic' + str(self.my_id), DataMessage)
        self.data_writer = DataWriter(self.publisher, self.data_topic, qos=self.reliable_qos)
        self.data_listener = SelfDataListener(self.my_id, self.my_id)
        self.other_agent_data_listeners = {}
        self.data_reader = DataReader(self.subscriber, self.data_topic, listener=self.data_listener, qos=self.reliable_qos)
        self.other_agent_data_readers = {}
        self.other_agent_location_listeners = {}
        self.other_agent_location_readers = {}

        self.object_publisher = rospy.Publisher('/object_from_agent', DetectedObject, queue_size=10)
        self.path_publisher = rospy.Publisher('/path_from_agent', AgentPath, queue_size=10)
        self.location_publisher = rospy.Publisher('/agent_location', AgentLocation, queue_size=10)

        self.cone_subscriber = rospy.Subscriber('/new_cone_map', DetectedObject, self.cone_callback, queue_size=10)
        self.agent_subscriber = rospy.Subscriber('/agent_to_subscribe', AgentSubscription, self.agent_subscription_callback, queue_size=10)
        self.path_subscriber = rospy.Subscriber('/cmd_smoothed_path', Path, self.path_callback, queue_size=10)
    
    def cone_callback(self, msg):
        cone_message = DataMessage(
            message_type="detected_object",
            sending_agent=int(self.my_id),
            timestamp=int(time.time()),
            data=json.dumps(message_converter.convert_ros_message_to_dictionary(msg))
        )
        self.data_writer.write(cone_message)

    def path_callback(self, msg):
        path_message = DataMessage(
            message_type="path",
            sending_agent=int(self.my_id),
            timestamp=int(time.time()),
            data=json.dumps(message_converter.convert_ros_message_to_dictionary(msg))
        )
        self.data_writer.write(path_message)

    def agent_subscription_callback(self, msg):
        agents = msg.agentIDs.data
        for agent in agents:
            if agent not in self.agents_subscribed:
                self.agents_subscribed.add(agent)

                topic_name = 'DataTopic' + str(agent)
                topic = Topic(self.participant, topic_name, DataMessage)
                self.other_agent_data_listeners[agent] = OtherDataListener(self.my_id, agent, self.object_publisher, self.path_publisher)
                self.other_agent_data_readers[agent] = DataReader(self.subscriber, topic, listener=self.other_agent_data_listeners[agent], qos=self.reliable_qos)

                location_topic_name = 'LocationTopic' + str(agent)
                location_topic = Topic(self.participant, location_topic_name, Location)
                self.other_agent_location_listeners[agent] = LocationListener(self.my_id, agent, self.location_publisher)
                self.other_agent_location_readers[agent] = DataReader(self.subscriber, location_topic, listener=self.other_agent_location_listeners[agent], qos=self.best_effort_qos)

                print("Subscribed to agent " + str(agent))

        agents_to_remove = []
        for agent in self.agents_subscribed:
            if agent not in agents:
                agents_to_remove.append(agent)
                
                self.other_agent_data_readers[agent] = None
                self.other_agent_data_readers.pop(agent)
                self.other_agent_data_listeners[agent] = None
                self.other_agent_data_listeners.pop(agent)    

                self.other_agent_location_readers[agent] = None
                self.other_agent_location_readers.pop(agent)
                self.other_agent_location_listeners[agent] = None
                self.other_agent_location_listeners.pop(agent)

                print("Unsubscribed from agent " + str(agent))
        for agent in agents_to_remove:
            self.agents_subscribed.remove(agent)

    def run(self):
        while not rospy.is_shutdown():
            time.sleep(1)

    def shutdown(self):
        print("Shutting down DDS Communication Manager")


if __name__ == '__main__':
    time.sleep(5)  # Wait
    manager = CommManager()
    rospy.on_shutdown(manager.shutdown)
    manager.run()