import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Float64MultiArray, Int16MultiArray
import tf
import rospkg

import sys

from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.core import Listener

import time
import os
import json
import numpy as np

from dds_utils import (
    DEFAULT_AGENT_TYPE,
    DdsLogger,
    ENTRY_EXIT_TOPIC,
    HEARTBEAT_PERIOD,
    INIT_DISCOVERY_GRACE_S,
    INIT_MAX_RETRIES,
    INIT_RECENT_THRESHOLD_S,
    INIT_RETRY_SLEEP_S,
    INITIALIZATION_TOPIC,
    INTER_DDS_WRITE_SLEEP_S,
    EntryExit,
    Initialization,
    ROS_TOPIC_AGENTS_TO_SUBSCRIBE,
    ROS_TOPIC_ENTRY_AGENTS,
    ROS_TOPIC_EXITED_AGENTS,
    ROS_TOPIC_HEARTBEAT_AGENTS,
    ROS_TOPIC_MAP,
    ROS_TOPIC_MAP_METADATA,
    ROS_TOPIC_MAP_MOD,
    ROS_TOPIC_TRANSFORMATION_MATRIX,
    RobotIdError,
    TransformMixin,
    create_domain_participant,
    dispose_participant,
    entry_init_reliable_qos,
    get_local_ip,
    hash_robot_id,
    pack_transform_msg,
    require_robot_id_int,
)


_log = DdsLogger("entry_exit")
_log_info = _log.info
_log_warn = _log.warn
_log_debug = _log.debug


def _occupancy_grid_from_map_dict(map_data):
    """Build OccupancyGrid from mattbot_mcl current_map-style JSON map dict."""
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width = map_data.get("width")
    grid.info.height = map_data.get("height")
    grid.info.resolution = map_data.get("resolution")
    grid.info.origin.position.x = map_data.get("origin_x")
    grid.info.origin.position.y = map_data.get("origin_y")
    grid.info.origin.position.z = map_data.get("origin_z")
    grid.info.origin.orientation.x = map_data.get("origin_orientation_x")
    grid.info.origin.orientation.y = map_data.get("origin_orientation_y")
    grid.info.origin.orientation.z = map_data.get("origin_orientation_z")
    grid.info.origin.orientation.w = map_data.get("origin_orientation_w")
    grid.data = map_data.get("occupancy")
    return grid


def _read_known_points(path):
    known_points = []
    with open(path, "r") as f:
        for line in f:
            x, y = line.split(",")
            known_points.append((float(x), float(y)))
    return known_points


class EntryExitListener(Listener):
    """
    Listener class for handling entry and exit events of agents in the environment.

    Attributes:
    - participant (Participant): The DDS participant.
    - publisher (Publisher): The DDS publisher.
    - subscriber (Subscriber): The DDS subscriber.
    - my_id (int): The ID of the current agent.
    - my_ip (str): The IP address of the current agent.
    - my_hash (int): The hash value of the current agent.
    - init_writer (Writer): The writer for sending initialization messages.
    - agents (dict): Dictionary of active agents in the environment.
    - exited_agents (dict): Dictionary of agents that have exited the environment.
    - lost_agents (dict): Dictionary of agents that have been lost.
    - map_msg (OccupancyGrid): The occupancy grid map message.
    - map_md_msg (MapMetaData): The map metadata message.
    - update_to_agents (bool): Flag indicating if there are updates to be sent to agents.

    Methods:
    - on_data_available(reader): Callback method for handling incoming data.
    - agent_update_available(): Checks if there are updates to be sent to agents.
    - get_agents(): Retrieves the active agents, exited agents, and lost agents.
    - update_agents(agents): Updates the active agents.
    - update_map(map, map_md): Updates the occupancy grid map and map metadata.
    """

    def __init__(self, participant, publisher, subscriber, my_id, my_id_int, my_ip, my_hash, init_writer):
        super().__init__()
        self.participant = participant
        self.publisher = publisher
        self.subscriber = subscriber

        self.agents = dict()
        self.exited_agents = dict()
        self.agents[my_id_int] = {
            "agent_type": DEFAULT_AGENT_TYPE,
            "ip_address": my_ip,
            "hash": my_hash,
            "timestamp": int(time.time()),
        }

        self.my_id = my_id
        self.my_id_int = my_id_int
        self.my_ip = my_ip
        self.my_hash = my_hash
        self.map_msg = OccupancyGrid()
        self.map_mod_msg = OccupancyGrid()
        self.map_md_msg = MapMetaData()
        self.known_points = []
        self.init_writer = init_writer
        self.ready_to_welcome = False

        self.update_to_agents = False

    def set_ready_to_welcome(self, ready):
        """When False, ignore enter requests until local setup has finished."""
        self.ready_to_welcome = ready

    def on_data_available(self, reader):
        """
        Callback method for handling incoming data.

        Parameters:
        - reader (Reader): The DDS reader.

        Returns:
        - None
        """
        for sample in reader.read():

            if sample.agent_id == self.my_id_int:
                # Ignore messages from self
                continue

            # Determine if entry or exit message
            if sample.action == "enter":
                if not self.ready_to_welcome or not self.known_points:
                    continue

                _log_info(
                    "Agent %s of type '%s' is requesting entry",
                    sample.agent_id,
                    sample.agent_type,
                )

                agents_message = json.dumps(self.agents)
                known_points_json = json.dumps(self.known_points)

                init_message = Initialization(
                    target_agent=sample.agent_id,
                    sending_agent=self.my_id_int,
                    agents=agents_message,
                    known_points=known_points_json,
                )
                self.init_writer.write(init_message)
                time.sleep(INTER_DDS_WRITE_SLEEP_S)
            elif sample.action == "initialized":

                # Only if the sample.timestamp is recent
                if int(time.time()) - sample.timestamp < INIT_RECENT_THRESHOLD_S:
                    _log_info(
                        "Agent %s of type '%s' entered the environment",
                        sample.agent_id,
                        sample.agent_type,
                    )

                    # Agent initialized, add to agents dictionary
                    new_robot_hash = hash_robot_id(str(sample.agent_id))
                    self.agents[sample.agent_id] = {
                        "agent_type": sample.agent_type,
                        "ip_address": sample.ip_address,
                        "hash": new_robot_hash,
                        "timestamp": sample.timestamp,
                    }

                    # Remove from exited agents if it exists
                    if sample.agent_id in self.exited_agents:
                        self.exited_agents.pop(sample.agent_id)

                    self.update_to_agents = True
            elif sample.action == "exit":
                # Agent Exited, remove from agents dictionary
                if sample.agent_id in self.agents:
                    _log_info("Agent %s exited the environment", sample.agent_id)
                    self.agents.pop(sample.agent_id)  # Pop from agents dictionary
                    self.exited_agents[sample.agent_id] = int(time.time())  # Add to exited agents dictionary
                    self.update_to_agents = True

    def agent_update_available(self):
        """
        Checks if there are updates to be sent to agents.

        Returns:
        - bool: True if there are updates, False otherwise.
        """
        return self.update_to_agents

    def get_agents(self):
        """
        Retrieves the active agents, exited agents, and lost agents.

        Returns:
        - tuple: A tuple containing the active agents, exited agents, and lost agents.
        """
        self.update_to_agents = False
        exited_agents = self.exited_agents.copy()
        self.exited_agents.clear()
        return self.agents, exited_agents

    def update_agents(self, agents=None, exited_agents=None, lost_agents=None):
        """
        Updates the active agents.

        Parameters:
        - agents (dict): The updated dictionary of active agents.
        - exited_agents (dict): The updated dictionary of exited agents.
        - lost_agents (dict): The updated dictionary of lost agents.

        Returns:
        - None
        """
        if agents is not None:
            self.agents = agents

    def update_known_points(self, known_points):
        """
        Updates the known points in the environment.

        Parameters:
        - known_points (list): A list of known points in the environment.

        Returns:
        - None
        """
        self.known_points = known_points


class InitializationListener(Listener):
    """
    Listener class for handling initialization messages.

    Attributes:
        map_received (bool): Flag indicating if the map has been received.
        map_msg (OccupancyGrid): OccupancyGrid message containing the map data.
        map_md_msg (MapMetaData): MapMetaData message containing the map metadata.
        agents (dict): Dictionary containing information about the agents.
        my_id (int): ID of the current agent.
        map_publisher: Publisher for the map message.
        map_md_publisher: Publisher for the map metadata message.
    """

    def __init__(self, my_id, my_id_int):
        super().__init__()
        self.map_received = False
        self.map_msg = OccupancyGrid()
        self.map_mod_msg = OccupancyGrid()
        self.map_md_msg = MapMetaData()
        self.agents = dict()
        self.my_id = my_id
        self.my_id_int = my_id_int
        self.known_points_received = False
        self.reference_known_points = []

    def on_data_available(self, init_reader):
        """
        Callback function called when initialization data is available.

        Args:
            init_reader: Reader object for reading initialization data.
        """
        for sample in init_reader.read():

            sending_agent = sample.sending_agent
            # Ignore messages from self
            if sending_agent == self.my_id_int:
                continue

            _log_info("Initialization message received from agent %s", sending_agent)

            if sample.target_agent != self.my_id_int:
                continue

            try:
                known_points = json.loads(sample.known_points)
            except (json.JSONDecodeError, TypeError) as exc:
                _log_warn("Initialization known_points parse failed: %s", exc)
                continue

            if not self.known_points_received:
                self.reference_known_points = known_points
                self.known_points_received = True
                _log_info("Reference points received through initialization message")

            try:
                agent_dict = json.loads(sample.agents)
            except (json.JSONDecodeError, TypeError) as exc:
                _log_warn("Initialization agents parse failed: %s", exc)
                continue

            for agent_id, agent_info in agent_dict.items():
                try:
                    aid = int(agent_id)
                except (TypeError, ValueError):
                    _log_warn("Skipping initialization agent id %r", agent_id)
                    continue
                if aid == self.my_id_int:
                    continue
                if aid in self.agents:
                    continue
                if not isinstance(agent_info, dict):
                    continue
                try:
                    self.agents[aid] = {
                        "agent_type": agent_info.get("agent_type", DEFAULT_AGENT_TYPE),
                        "ip_address": agent_info.get("ip_address", ""),
                        "hash": agent_info.get("hash", hash_robot_id(str(aid))),
                        "timestamp": agent_info.get("timestamp", int(time.time())),
                    }
                except (TypeError, ValueError) as exc:
                    _log_warn("Skipping bad agent entry %r: %s", agent_id, exc)

    def map_available(self):
        """
        Check if the map has been received.

        Returns:
            bool: True if the map has been received, False otherwise.
        """
        return self.map_received

    def known_points_available(self):
        """
        Check if the known points have been received.

        Returns:
            bool: True if the known points have been received, False otherwise.
        """
        return self.known_points_received

    def get_map(self):
        """
        Get the map and map metadata.

        Returns:
            tuple: A tuple containing the map message and map metadata message.
        """
        return self.map_msg, self.map_mod_msg, self.map_md_msg

    def get_known_points(self):
        """
        Get the known points in the environment.

        Returns:
            list: A list of known points in the environment.
        """
        return self.reference_known_points

    def get_agents(self):
        """
        Get the agents dictionary.

        Returns:
            dict: Dictionary containing information about the agents.
        """
        return self.agents


class EntryExitCommunication(TransformMixin):
    def __init__(self):

        rospy.init_node("agent_entry_exit", anonymous=True)

        self.init_transform_state()

        try:
            self.my_id_int = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)
        self.my_id = str(self.my_id_int)
        self.my_hash = hash_robot_id(self.my_id_int)
        self.my_ip = get_local_ip()
        _log_info("My Agent ID is %s, IP address is %s", self.my_id, self.my_ip)

        # Dictionary to store agents in the environment
        self.agents = dict()

        # Map and Map Metadata messages, and publishers
        self.map_msg = OccupancyGrid()
        self.map_md_msg = MapMetaData()

        self.map_publisher = rospy.Publisher(ROS_TOPIC_MAP, OccupancyGrid, queue_size=10)
        self.map_mod_publisher = rospy.Publisher(ROS_TOPIC_MAP_MOD, OccupancyGrid, queue_size=10)
        self.map_md_publisher = rospy.Publisher(ROS_TOPIC_MAP_METADATA, MapMetaData, queue_size=10)

        # Create a DomainParticipant, Subscriber, and Publisher
        self.participant = create_domain_participant(domain_qos=True)
        self.subscriber = Subscriber(self.participant)
        self.publisher = Publisher(self.participant)

        # Create the topics needed
        self.entry_exit_topic = Topic(self.participant, ENTRY_EXIT_TOPIC, EntryExit)
        self.init_topic = Topic(self.participant, INITIALIZATION_TOPIC, Initialization)

        # Create the DataWriters and DataReaders
        self.enter_exit_writer = DataWriter(self.publisher, self.entry_exit_topic, qos=entry_init_reliable_qos)
        self.init_writer = DataWriter(self.publisher, self.init_topic, qos=entry_init_reliable_qos)

        # ROS Publisher for publishing transformation matrix
        self.transform_pub = rospy.Publisher(ROS_TOPIC_TRANSFORMATION_MATRIX, Float64MultiArray, queue_size=10)

        # FIXME
        self.agent_sub_pub = rospy.Publisher(ROS_TOPIC_AGENTS_TO_SUBSCRIBE, Int16MultiArray, queue_size=10)

        self.heartbeat_agents = list()
        self.agent_pub = rospy.Publisher(ROS_TOPIC_ENTRY_AGENTS, Int16MultiArray, queue_size=10)
        self.exited_agent_pub = rospy.Publisher(ROS_TOPIC_EXITED_AGENTS, Int16MultiArray, queue_size=10)
        self.agent_sub = rospy.Subscriber(ROS_TOPIC_HEARTBEAT_AGENTS, Int16MultiArray, self.heartbeat_agents_callback)

        self.entry_exit_listener = EntryExitListener(
            self.participant,
            self.publisher,
            self.subscriber,
            self.my_id,
            self.my_id_int,
            self.my_ip,
            self.my_hash,
            self.init_writer,
        )
        self.init_listener = InitializationListener(self.my_id, self.my_id_int)

        # We will start the readers later when it is necessary
        self.enter_exit_reader = None
        self.init_reader = None

        # TF Listener to get current robot position
        self.trans_listener = tf.TransformListener()
        self.my_location = None

        self.last_time = int(time.time())

    def heartbeat_agents_callback(self, data):
        self.heartbeat_agents = data.data

    def setup_and_run(self):
        """
        Sets up the agent and runs it.
        """
        self.setup()
        self.run()

    def setup(self):
        """
        Sets up the agent by retrieving the map and initializing the environment.

        If the agent is the first to enter the environment, it retrieves the map from a GraphQL server,
        converts the map data into ROS Occupancy grid format, and publishes the map to the appropriate topics.

        If the agent is not the first to enter the environment, it sends an entry message to the enter/exit writer,
        waits for the map to become available, retrieves the map and agent information from the init listener,
        updates the agent information, and publishes the map to the appropriate topics.

        Returns:
            None
        """
        _log_info("===== SETUP START (robot_id=%s) =====", self.my_id)

        # Load the map from the current_map.json file and publish it
        self.load_map()

        # Now get reference points
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("mattbot_dds")
        self.known_points = _read_known_points(os.path.join(package_path, "scripts", "known_points.txt"))

        self.entry_exit_listener.update_known_points(self.known_points)

        self.enter_exit_reader = DataReader(
            self.subscriber,
            self.entry_exit_topic,
            listener=self.entry_exit_listener,
            qos=entry_init_reliable_qos,
        )
        self.init_reader = DataReader(
            self.subscriber,
            self.init_topic,
            listener=self.init_listener,
            qos=entry_init_reliable_qos,
        )

        _log_debug("Waiting %.1fs for DDS discovery before enter", INIT_DISCOVERY_GRACE_S)
        time.sleep(INIT_DISCOVERY_GRACE_S)

        entry_message = EntryExit(self.my_id_int, DEFAULT_AGENT_TYPE, "enter", self.my_ip, int(time.time()))
        self.enter_exit_writer.write(entry_message)
        time.sleep(INTER_DDS_WRITE_SLEEP_S)

        num_tries = 0
        while not self.init_listener.known_points_available() and num_tries < INIT_MAX_RETRIES:
            _log_info(
                "Reference points not yet received (attempt %s/%s)",
                num_tries + 1,
                INIT_MAX_RETRIES,
            )
            time.sleep(INIT_RETRY_SLEEP_S)
            if not self.init_listener.known_points_available():
                entry_message.timestamp = int(time.time())
                self.enter_exit_writer.write(entry_message)
                time.sleep(INTER_DDS_WRITE_SLEEP_S)
                num_tries += 1

        if self.init_listener.known_points_available():
            _log_info("I am not the first agent, received reference points")

            # Store the map, map metadata, and agents
            self.reference_known_points = self.init_listener.get_known_points()
            self.agents = self.init_listener.get_agents()

            # Add myself to the agents dictionary
            self.agents[self.my_id_int] = {
                "agent_type": DEFAULT_AGENT_TYPE,
                "ip_address": self.my_ip,
                "hash": self.my_hash,
                "timestamp": int(time.time()),
            }

            # Update the agents in the entry/exit listener
            self.entry_exit_listener.update_agents(agents=self.agents)
        else:
            _log_info("I am the first agent, my map will be the reference map")
            self.reference_known_points = self.known_points

            self.agents[self.my_id_int] = {
                "agent_type": DEFAULT_AGENT_TYPE,
                "ip_address": self.my_ip,
                "hash": self.my_hash,
                "timestamp": int(time.time()),
            }

        self.create_transform()  # Create the transform from the known points

        # Update the entry/exit listener with the known points
        self.entry_exit_listener.update_known_points(self.reference_known_points)

        self.entry_exit_listener.update_agents(agents=self.agents)
        self.entry_exit_listener.set_ready_to_welcome(True)

        self.init_reader = None
        self.init_listener = None

        entry_message = EntryExit(self.my_id_int, DEFAULT_AGENT_TYPE, "initialized", self.my_ip, int(time.time()))
        self.enter_exit_writer.write(entry_message)
        time.sleep(INTER_DDS_WRITE_SLEEP_S)

        _log_info("===== SETUP COMPLETE (robot_id=%s) =====", self.my_id)

    def load_map(self):

        # find mattbot_mcl package path
        rospack = rospkg.RosPack()
        package_path = rospack.get_path("mattbot_mcl")
        map_json_dir = os.path.join(package_path, "map_json")

        with open(os.path.join(map_json_dir, "current_map.json"), "r") as f:
            map_data = json.load(f).get("data", {}).get("map", {})

        with open(os.path.join(map_json_dir, "current_map_mod.json"), "r") as f:
            map_mod_data = json.load(f).get("data", {}).get("map", {})

        self.map_msg = _occupancy_grid_from_map_dict(map_data)
        self.map_mod_msg = _occupancy_grid_from_map_dict(map_mod_data)

        self.map_md_msg.map_load_time = rospy.Time.now()
        self.map_md_msg.resolution = map_data.get("resolution")
        self.map_md_msg.width = map_data.get("width")
        self.map_md_msg.height = map_data.get("height")
        self.map_md_msg.origin.position.x = map_data.get("origin_x")
        self.map_md_msg.origin.position.y = map_data.get("origin_y")
        self.map_md_msg.origin.position.z = map_data.get("origin_z")
        self.map_md_msg.origin.orientation.x = map_data.get("origin_orientation_x")
        self.map_md_msg.origin.orientation.y = map_data.get("origin_orientation_y")
        self.map_md_msg.origin.orientation.z = map_data.get("origin_orientation_z")
        self.map_md_msg.origin.orientation.w = map_data.get("origin_orientation_w")

        # Publish the map and map metadata for ROS nodes
        self.map_publisher.publish(self.map_msg)
        self.map_mod_publisher.publish(self.map_mod_msg)
        self.map_md_publisher.publish(self.map_md_msg)

        _log_info("Map loaded from current_map.json")

    def create_transform(self):
        """
        Determines the transform from my map to the reference map
        """
        self.R = None
        self.t = None
        if self.known_points == self.reference_known_points:
            self.R = np.identity(2)
            self.t = np.zeros((2, 1))
        else:
            # Find the transform from the known points
            known_points = np.array(self.known_points)
            reference_known_points = np.array(self.reference_known_points)

            centroid1 = np.mean(known_points, axis=0)
            centroid2 = np.mean(reference_known_points, axis=0)
            centered_points1 = known_points - centroid1
            centered_points2 = reference_known_points - centroid2

            H = np.dot(centered_points1.T, centered_points2)
            U, S, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T

            if np.linalg.det(R) < 0:
                Vt[1, :] *= -1
                R = Vt.T @ U.T

            t = centroid2 - R @ centroid1

            self.R = R
            self.t = t

        # Now publish the transformation matrix
        self.transform_pub.publish(pack_transform_msg(self.R, self.t))

    def run(self):
        """
        Executes the main loop of the agent_entry_exit node.

        Returns:
            None
        """

        prev_agent_set = set()
        exited_agents = dict()

        # Loop through at the rate we wish to publish location
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            current_time = int(time.time())  # Get the current time

            # Periodically perform some updates
            if current_time - self.last_time >= HEARTBEAT_PERIOD:  # FIXME different period...
                self.last_time = current_time
                update_to_active_agents = False

                # Publish map/map metadata periodically
                self.map_publisher.publish(self.map_msg)
                self.map_mod_publisher.publish(self.map_mod_msg)
                self.map_md_publisher.publish(self.map_md_msg)

                # Check for new agents
                if self.entry_exit_listener.agent_update_available():
                    self.agents, newly_exited_agents = self.entry_exit_listener.get_agents()

                    if len(newly_exited_agents):
                        for agent_id in newly_exited_agents:
                            exited_agents[agent_id] = newly_exited_agents[agent_id]

                current_agents_list = list(self.agents.keys())
                for agent_id in current_agents_list:
                    if agent_id in exited_agents:
                        exited_agents.pop(agent_id)  # Remove from exited agents dictionary if reentered

                # get heartbeat agents at this snapshot in time
                heartbeat_agents = self.heartbeat_agents

                new_agents = set(heartbeat_agents) - set(current_agents_list)
                for agent_id in new_agents:
                    if agent_id not in exited_agents:
                        self.agents[agent_id] = {
                            "agent_type": "unknown",
                            "ip_address": "unknown",
                            "hash": hash_robot_id(str(agent_id)),
                            "timestamp": int(time.time()),
                        }
                        update_to_active_agents = True

                # Check for dead agents that haven't exited gracefully
                dead_agents = prev_agent_set - set(heartbeat_agents)
                prev_agent_set = set(heartbeat_agents)
                for agent_id in dead_agents:
                    if agent_id in self.agents:
                        self.agents.pop(agent_id)
                        update_to_active_agents = True

                # Update the entry/exit listener with the new agents
                if update_to_active_agents:
                    self.entry_exit_listener.update_agents(agents=self.agents)

                self.update_agents(exited_agents=exited_agents)

            agent_list_minus_self = list(self.agents.keys())
            if self.my_id_int in agent_list_minus_self:
                agent_list_minus_self.remove(self.my_id_int)
            self.agent_sub_pub.publish(Int16MultiArray(data=agent_list_minus_self))

            rate.sleep()

    def update_agents(self, exited_agents=None):

        # Publish the agents
        agent_list = list(self.agents.keys())
        entry_agents = Int16MultiArray(data=agent_list)
        self.agent_pub.publish(entry_agents)

        # Publish the exited agents
        if exited_agents is not None:
            exited_agent_list = list(exited_agents.keys())
            exited_agents = Int16MultiArray(data=exited_agent_list)
            self.exited_agent_pub.publish(exited_agents)

    def shutdown(self):
        _log_info("Sending DDS exit message")
        # Write exit message
        exit_message = EntryExit(self.my_id_int, DEFAULT_AGENT_TYPE, "exit", self.my_ip, int(time.time()))
        if self.enter_exit_writer is not None:
            self.enter_exit_writer.write(exit_message)
        self.enter_exit_reader = None
        self.init_reader = None
        self.enter_exit_writer = None
        self.init_writer = None
        self.subscriber = None
        self.publisher = None
        dispose_participant(self.participant)
        self.participant = None


if __name__ == "__main__":

    entry_exit_obj = EntryExitCommunication()
    rospy.on_shutdown(entry_exit_obj.shutdown)
    entry_exit_obj.setup_and_run()