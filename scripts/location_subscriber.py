import rospy
from std_msgs.msg import Float64MultiArray, Int16MultiArray
from mattbot_dds.msg import AgentLocation
import tf
import sys

from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.core import Listener

import numpy as np

from dds_utils import (
    Location,
    DdsLogger,
    ROS_TOPIC_AGENTS_TO_SUBSCRIBE,
    ROS_TOPIC_TRANSFORMATION_MATRIX,
    RobotIdError,
    TransformMixin,
    best_effort_qos,
    create_domain_participant,
    dispose_participant,
    location_topic_name,
    require_robot_id_int,
)

_log = DdsLogger("location_subscriber")


class LocationListener(Listener, TransformMixin):
    """
    Listener class that handles location data for agents.

    Attributes:
        my_id_int (int): The ID of the listener.
        agent_ids (list): List of agent IDs.
        locations (dict): Dictionary to store agent locations.

    Methods:
        on_data_available(reader): Callback method called when data is available.
        get_locations(): Returns the locations dictionary.
        set_agent_ids(agent_ids): Sets the agent IDs and updates the locations dictionary.
    """

    def __init__(self, my_id_int, agent_id):
        super().__init__()
        self.init_transform_state()
        self.my_id_int = my_id_int
        self.agent_id = agent_id
        self.locations = (None, None, None)

        self.agent_location_publisher = rospy.Publisher("/agent_location", AgentLocation, queue_size=10)

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
            if sample.agent_id == self.my_id_int:
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


class LocationSubscriber(TransformMixin):
    def __init__(self):
        rospy.init_node("dds_location_subscriber", anonymous=True)

        self.init_transform_state()

        try:
            self.my_id_int = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)
        self.my_id = str(self.my_id_int)

        self.participant = create_domain_participant(domain_qos=True)
        self.subscriber = Subscriber(self.participant)

        self.location_listeners = dict()
        self.location_readers = dict()

        rospy.Subscriber(ROS_TOPIC_TRANSFORMATION_MATRIX, Float64MultiArray, self.transformation_callback)

        self.subscribed_agents = set()
        self.agents_to_subscribe = set()
        self.agents_to_subscribe_subscriber = rospy.Subscriber(
            ROS_TOPIC_AGENTS_TO_SUBSCRIBE, Int16MultiArray, self.agents_to_subscribe_callback
        )

    def agents_to_subscribe_callback(self, data):
        # Get the list of agents to subscribe to
        agents_to_subscribe = data.data

        self.agents_to_subscribe = set(agents_to_subscribe)

    def run(self):
        while not rospy.is_shutdown():

            # Get current position of the agent
            try:
                new_agents = self.agents_to_subscribe - self.subscribed_agents
                old_agents = self.subscribed_agents - self.agents_to_subscribe

                for agent_id in new_agents:
                    _log.info("Subscribed to agent %s location", agent_id)
                    new_location_topic = Topic(self.participant, location_topic_name(agent_id), Location)
                    self.location_listeners[agent_id] = LocationListener(self.my_id_int, agent_id)
                    self.location_listeners[agent_id].update_transformation(self.R, self.t)
                    self.location_readers[agent_id] = DataReader(
                        self.subscriber, new_location_topic, listener=self.location_listeners[agent_id], qos=best_effort_qos
                    )

                for agent_id in old_agents:
                    _log.info("Unsubscribed from agent %s location", agent_id)
                    self.location_readers[agent_id] = None
                    self.location_listeners[agent_id] = None
                    self.location_readers.pop(agent_id)
                    self.location_listeners.pop(agent_id)

                self.subscribed_agents = self.agents_to_subscribe

            except Exception as e:
                pass

            rospy.sleep(1)

    def shutdown(self):
        _log.debug("Shutting down")
        self.location_readers.clear()
        self.location_listeners.clear()
        self.subscriber = None
        dispose_participant(self.participant)
        self.participant = None


if __name__ == "__main__":
    location_subscriber = LocationSubscriber()
    rospy.on_shutdown(location_subscriber.shutdown)
    location_subscriber.run()
