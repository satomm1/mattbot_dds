import rospy
from std_msgs.msg import Float64MultiArray, Int16MultiArray
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

import time
import os
import numpy as np
import socket
import hashlib

from dds_utils import Heartbeat, best_effort_qos

HEARTBEAT_PERIOD = 10    # seconds
HEARTBEAT_TIMEOUT = 31  # seconds
AGENT_TYPE = "robot"

class HeartbeatListener(Listener):
    """
    Listener class that handles heartbeat data from agents.

    Attributes:
        heartbeats (dict): A dictionary to store the heartbeats of agents.
        my_id (int): The ID of the current agent.
        agents (dict): A dictionary to store information about all agents in the environment.
    """

    def __init__(self, my_id):
        super().__init__()
        self.heartbeats = dict()
        self.new_heartbeats = dict()
        self.my_id = my_id

        self.R = None
        self.t = None

    def on_data_available(self, reader):
        """
        Callback method called when data is available in the reader.

        Args:
            reader (DataReader): The DataReader object.

        Returns:
            None
        """
        for sample in reader.read():

            # Skip messages from self
            if sample.agent_id == int(self.my_id):
                continue

            self.new_heartbeats[sample.agent_id] = sample.timestamp
            self.heartbeats[sample.agent_id] = sample.timestamp

    def get_heartbeats(self):
        """
        Get a copy of the heartbeats dictionary.

        Returns:
            dict: A copy of the heartbeats dictionary.
        """
        returned_heartbeats = self.new_heartbeats.copy()
        self.new_heartbeats.clear()
        return returned_heartbeats

def hash_func(robot_id):
    """
    Hashes the given robot ID using SHA-256 algorithm.

    Parameters:
    robot_id (str): The robot ID to be hashed.

    Returns:
    int: The hashed robot ID as an integer.

    """
    return int(hashlib.sha256(robot_id.encode()).hexdigest(), 16)


class HeartbeatSubscriber:
    def __init__(self):
        rospy.init_node('dds_heartbeat_subscriber', anonymous=True)

        # Get robot ID, Hash, and IP Address
        self.my_id = os.environ.get('ROBOT_ID')

        self.my_hash = hash_func(self.my_id)

        # Get IP Address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # This doesn't have to be reachable; it just has to be a valid address
        s.connect(("8.8.8.8", 80))
        self.my_ip = s.getsockname()[0]
        s.close()
        print(f"My IP address is {self.my_ip}")

        # Dictionary to store agents in the environment
        self.agents = dict()
        self.exited_agents = set()
        self.prev_exited_agents = set()

        # ROS Publisher for publishing active agents and exited agents
        self.active_agents_pub = rospy.Publisher('/heartbeat_agents', Int16MultiArray, queue_size=10)
        self.active_agents_sub = rospy.Subscriber('/entry_agents', Int16MultiArray, self.active_agents_callback)
        self.exited_agents_sub = rospy.Subscriber('/exited_agents', Int16MultiArray, self.exited_agents_callback)

        self.lease_duration_ms = 30000
        qos_profile = DomainParticipantQos()
        qos_profile.lease_duration = duration(milliseconds=self.lease_duration_ms)

        # Create a DomainParticipant, Subscriber
        self.participant = DomainParticipant()
        self.subscriber = Subscriber(self.participant)

        self.heartbeat_topic = Topic(self.participant, 'HeartbeatTopic', Heartbeat)
        self.heartbeat_listener = HeartbeatListener(self.my_id)
        self.heartbeat_reader = DataReader(self.subscriber, self.heartbeat_topic, listener=self.heartbeat_listener, qos=best_effort_qos)

        # self.trans_listener = tf.TransformListener()
        # self.R = None
        # self.t = None
        # transformation_subscriber = rospy.Subscriber('/transformation_matrix', Float64MultiArray, self.transformation_callback)

    def active_agents_callback(self, data):
        # Get the list of active agents
        active_agents = data.data

        # Update the agents dictionary with the new active agents
        for agent_id in active_agents:
            if agent_id not in self.agents:
                self.agents[agent_id] = {'timestamp': int(time.time())}

            if agent_id in self.exited_agents:
                self.exited_agents.remove(agent_id)

            if agent_id in self.prev_exited_agents:
                self.prev_exited_agents.remove(agent_id)

    def exited_agents_callback(self, data):
        # Get the list of exited agents
        exited_agents = data.data
        self.exited_agents = set(exited_agents)

        # Remove the exited agents from the agents dictionary
        for agent_id in exited_agents:
            if agent_id not in self.prev_exited_agents:
                self.prev_exited_agents.add(agent_id)
                if agent_id in self.agents:
                    self.agents.pop(agent_id)

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

        last_time = int(time.time())
        prev_exited_agents = set()
        while not rospy.is_shutdown():
            current_time = int(time.time())

            if current_time - last_time >= HEARTBEAT_PERIOD:
                last_time = current_time

                # Update agents with new heartbeats
                heartbeats = self.heartbeat_listener.get_heartbeats()

                update_to_active_agents = False
                for agent_id in heartbeats.keys():
                    if agent_id in self.agents:
                        self.agents[agent_id]['timestamp'] = heartbeats[agent_id]
                    else:
                        print(f'Detected heartbeat from unknown agent {agent_id}')
                        self.agents[agent_id] = {'timestamp': heartbeats[agent_id]}
                        update_to_active_agents = True

                        if agent_id in self.prev_exited_agents:
                            self.prev_exited_agents.remove(agent_id)
                            self.exited_agents.remove(agent_id)

                # Check Periodically for Dead Agents
                dead_agents = []
                for agent_id, agent_info in self.agents.items():
                    
                    # skip self
                    if agent_id == int(self.my_id):
                        continue

                    time_difference = current_time - agent_info['timestamp']
                    if time_difference > HEARTBEAT_TIMEOUT:
                        print(f"Agent {agent_id} has timed out")
                        dead_agents.append(agent_id)
                
                for agent_id in dead_agents:
                    self.agents.pop(agent_id)

                if update_to_active_agents or dead_agents:
                    # Publish our record of active agents
                    active_agents = Int16MultiArray(data=list(self.agents.keys()))
                    self.active_agents_pub.publish(active_agents)

            # Sleep for a short duration to avoid busy waiting
            time.sleep(1)
    def shutdown(self):
        rospy.loginfo("Shutting down DDS heartbeat subscriber...")

if __name__ == '__main__':
    heartbeat_subscriber = HeartbeatSubscriber()
    time.sleep(10)
    rospy.on_shutdown(heartbeat_subscriber.shutdown)
    heartbeat_subscriber.run()
