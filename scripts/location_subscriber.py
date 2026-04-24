import rospy
from std_msgs.msg import Float64MultiArray, Int16MultiArray
from mattbot_dds.msg import AgentLocation
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

from dds_utils import Location, best_effort_qos

class LocationListener(Listener):
    """
    Listener class that handles location data for agents.

    Attributes:
        my_id (int): The ID of the listener.
        agent_ids (list): List of agent IDs.
        locations (dict): Dictionary to store agent locations.

    Methods:
        on_data_available(reader): Callback method called when data is available.
        get_locations(): Returns the locations dictionary.
        set_agent_ids(agent_ids): Sets the agent IDs and updates the locations dictionary.
    """

    def __init__(self, my_id, agent_id):
        super().__init__()
        self.my_id = my_id
        self.agent_id = agent_id
        self.locations = (None, None, None)

        self.agent_location_publisher = rospy.Publisher('/agent_location', AgentLocation, queue_size=10)

        self.R = None
        self.t = None

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

    def update_transformation(self, R, t):
        self.R = R
        self.t = t

    def on_data_available(self, reader):
        """
        Callback method called when data is available.

        Args:
            reader: The data reader object.

        Returns:
            None
        """
        for sample in reader.read():

            # Skip messages from self
            if sample.agent_id == int(self.my_id):
                continue

            if sample.x is not None and sample.y is not None and sample.theta is not None:
                x, y, theta = self.transform_point((sample.x, sample.y, sample.theta), forward=False)
                self.locations = (x, y, theta)

                agent_location = AgentLocation()
                agent_location.agentID.data = int(sample.agent_id)
                agent_location.pose.position.x = x
                agent_location.pose.position.y = y
                agent_location.pose.position.z = 0.0

                quaternion = tf.transformations.quaternion_from_euler(0, 0, theta)
                agent_location.pose.orientation.x = quaternion[0]
                agent_location.pose.orientation.y = quaternion[1]
                agent_location.pose.orientation.z = quaternion[2]
                agent_location.pose.orientation.w = quaternion[3]

                agent_location.isStatic.data = sample.static

                self.agent_location_publisher.publish(agent_location)

    def get_locations(self):
        """
        Returns the location.

        Returns:
            tuple: The location of the agent.
        """
        return self.locations


class LocationSubscriber:
    def __init__(self):
        rospy.init_node('dds_location_subscriber', anonymous=True)

        # Get robot ID, Hash, and IP Address
        self.my_id = os.environ.get('ROBOT_ID')

        self.lease_duration_ms = 30000
        qos_profile = DomainParticipantQos()
        qos_profile.lease_duration = duration(milliseconds=self.lease_duration_ms)

        # Create a DomainParticipant, Subscriber, and Publisher
        self.participant = DomainParticipant(qos=qos_profile)
        self.subscriber = Subscriber(self.participant)

        self.location_listeners = dict()
        self.location_readers = dict()
        
        self.R = None
        self.t = None
        transformation_subscriber = rospy.Subscriber('transformation_matrix', Float64MultiArray, self.transformation_callback)

        self.subscribed_agents = set()
        self.agents_to_subscribe = set()
        self.agents_to_subscribe_subscriber = rospy.Subscriber('/agents_to_subscribe', Int16MultiArray, self.agents_to_subscribe_callback)

    def agents_to_subscribe_callback(self, data):
        # Get the list of agents to subscribe to
        agents_to_subscribe = data.data

        self.agents_to_subscribe = set(agents_to_subscribe)

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
                new_agents = self.agents_to_subscribe - self.subscribed_agents
                old_agents = self.subscribed_agents - self.agents_to_subscribe

                for agent_id in new_agents:
                    print(f"    Subscribed to agent {agent_id} location")
                    new_location_topic = Topic(self.participant, 'LocationTopic' + str(agent_id), Location)
                    self.location_listeners[agent_id] = LocationListener(self.my_id, agent_id)
                    self.location_listeners[agent_id].update_transformation(self.R, self.t)
                    self.location_readers[agent_id] = DataReader(self.subscriber, new_location_topic, listener=self.location_listeners[agent_id], qos=best_effort_qos)

                for agent_id in old_agents:
                    print(f"    Unsubscribed from agent {agent_id} location")
                    self.location_readers[agent_id] = None
                    self.location_listeners[agent_id] = None
                    self.location_readers.pop(agent_id)
                    self.location_listeners.pop(agent_id)

                self.subscribed_agents = self.agents_to_subscribe

            except Exception as e:
                pass

            rospy.sleep(1)

    def shutdown(self):
        rospy.loginfo("Shutting down DDS location publisher...")

if __name__ == '__main__':
    location_subscriber = LocationSubscriber()
    rospy.on_shutdown(location_subscriber.shutdown)
    location_subscriber.run()
