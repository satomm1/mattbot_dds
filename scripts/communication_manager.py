import rospy
from rospy_message_converter import message_converter
import tf
import rospkg
from mattbot_image_detection.msg import DetectedObject
from mattbot_dds.msg import AgentSubscription

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

    def __init__(self, my_id, topic_id):
        super().__init__()
        self.my_id = my_id
        self.topic_id = topic_id

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
            else:
                # This was a message to the agent, we can safely ignore
                continue

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

        self.object_publisher = rospy.Publisher('/object_from_agent', DetectedObject, queue_size=10)

        self.cone_subscriber = rospy.Subscriber('/new_cone_map', DetectedObject, self.cone_callback, queue_size=10)
        self.agent_subscriber = rospy.Subscriber('/agent_to_subscribe', AgentSubscription, self.agent_subscription_callback, queue_size=10)
    
    def cone_callback(self, msg):
        print(msg)
        cone_message = DataMessage(
            message_type="detected_object",
            sending_agent=int(self.my_id),
            timestamp=int(time.time()),
            data=json.dumps(message_converter.convert_ros_message_to_dictionary(msg))
        )
        self.data_writer.write(cone_message)

    def agent_subscription_callback(self, msg):
        agents = msg.agentIDs.data
        for agent in agents:
            if agent not in self.agents_subscribed:
                self.agents_subscribed.add(agent)
                topic_name = 'DataTopic' + str(agent)
                topic = Topic(self.participant, topic_name, DataMessage)
                self.other_agent_data_listeners[agent] = OtherDataListener(self.my_id, agent)
                self.other_agent_data_readers[agent] = DataReader(self.subscriber, topic, listener=self.other_agent_data_listeners[agent], qos=self.reliable_qos)
                print("Subscribed to agent " + str(agent))

        agents_to_remove = []
        for agent in self.agents_subscribed:
            if agent not in agents:
                agents_to_remove.append(agent)
                self.other_agent_data_readers[agent] = None
                self.other_agent_data_readers.pop(agent)
                self.other_agent_data_listeners[agent] = None
                self.other_agent_data_listeners.pop(agent)    
                print("Unsubscribed from agent " + str(agent))
        for agent in agents_to_remove:
            self.agents_subscribed.remove(agent)

    def run(self):
        while not rospy.is_shutdown():
            time.sleep(1)

    def shutdown(self):
        print("Shutting down DDS Communication Manager")


if __name__ == '__main__':
    
    manager = CommManager()
    rospy.on_shutdown(manager.shutdown)
    manager.run()