import rospy
from std_msgs.msg import Int16MultiArray
import sys
import time

from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.core import Listener

from dds_utils import (
    HEARTBEAT_PERIOD,
    HEARTBEAT_TIMEOUT,
    HEARTBEAT_TOPIC,
    Heartbeat,
    ROS_TOPIC_ENTRY_AGENTS,
    ROS_TOPIC_EXITED_AGENTS,
    ROS_TOPIC_HEARTBEAT_AGENTS,
    RobotIdError,
    best_effort_qos,
    create_domain_participant,
    dispose_participant,
    get_local_ip,
    require_robot_id_int,
)


class HeartbeatListener(Listener):
    """
    Listener class that handles heartbeat data from agents.

    Attributes:
        heartbeats (dict): A dictionary to store the heartbeats of agents.
        my_id_int (int): The ID of the current agent.
        agents (dict): A dictionary to store information about all agents in the environment.
    """

    def __init__(self, my_id_int):
        super().__init__()
        self.heartbeats = dict()
        self.new_heartbeats = dict()
        self.my_id_int = my_id_int

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
            if sample.agent_id == self.my_id_int:
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


class HeartbeatSubscriber:
    def __init__(self):
        rospy.init_node("dds_heartbeat_subscriber", anonymous=True)

        try:
            self.my_id_int = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)
        self.my_id = str(self.my_id_int)

        self.my_ip = get_local_ip()
        print(f"My IP address is {self.my_ip}")

        # Dictionary to store agents in the environment
        self.agents = dict()
        self.exited_agents = set()
        self.prev_exited_agents = set()

        # ROS Publisher for publishing active agents and exited agents
        self.active_agents_pub = rospy.Publisher(ROS_TOPIC_HEARTBEAT_AGENTS, Int16MultiArray, queue_size=10)
        self.active_agents_sub = rospy.Subscriber(ROS_TOPIC_ENTRY_AGENTS, Int16MultiArray, self.active_agents_callback)
        self.exited_agents_sub = rospy.Subscriber(ROS_TOPIC_EXITED_AGENTS, Int16MultiArray, self.exited_agents_callback)

        self.participant = create_domain_participant(domain_qos=False)
        self.subscriber = Subscriber(self.participant)

        self.heartbeat_topic = Topic(self.participant, HEARTBEAT_TOPIC, Heartbeat)
        self.heartbeat_listener = HeartbeatListener(self.my_id_int)
        self.heartbeat_reader = DataReader(
            self.subscriber, self.heartbeat_topic, listener=self.heartbeat_listener, qos=best_effort_qos
        )

    def active_agents_callback(self, data):
        # Get the list of active agents
        active_agents = data.data

        # Update the agents dictionary with the new active agents
        for agent_id in active_agents:
            if agent_id not in self.agents:
                self.agents[agent_id] = {"timestamp": int(time.time())}

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

    def run(self):

        last_time = int(time.time())
        while not rospy.is_shutdown():
            current_time = int(time.time())

            if current_time - last_time >= HEARTBEAT_PERIOD:
                last_time = current_time

                # Update agents with new heartbeats
                heartbeats = self.heartbeat_listener.get_heartbeats()

                update_to_active_agents = False
                for agent_id in heartbeats.keys():
                    if agent_id in self.agents:
                        self.agents[agent_id]["timestamp"] = heartbeats[agent_id]
                    else:
                        print(f"Detected heartbeat from unknown agent {agent_id}")
                        self.agents[agent_id] = {"timestamp": heartbeats[agent_id]}
                        update_to_active_agents = True

                        if agent_id in self.prev_exited_agents:
                            self.prev_exited_agents.remove(agent_id)
                            self.exited_agents.remove(agent_id)

                # Check Periodically for Dead Agents
                dead_agents = []
                for agent_id, agent_info in self.agents.items():

                    # skip self
                    if agent_id == self.my_id_int:
                        continue

                    time_difference = current_time - agent_info["timestamp"]
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
        self.heartbeat_reader = None
        self.subscriber = None
        dispose_participant(self.participant)
        self.participant = None


if __name__ == "__main__":
    heartbeat_subscriber = HeartbeatSubscriber()
    time.sleep(10)
    rospy.on_shutdown(heartbeat_subscriber.shutdown)
    heartbeat_subscriber.run()
