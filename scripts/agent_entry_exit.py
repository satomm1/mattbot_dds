import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from mattbot_dds.msg import AgentLocationsArray
from geometry_msgs.msg import Pose
from std_msgs.msg import Header, Int32
from rospy_message_converter import message_converter
import tf

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

# Constants (Set depending on the agent)
HEARTBEAT_PERIOD = 2    # seconds
HEARTBEAT_TIMEOUT = 15  # seconds
LOCATION_FREQUENCY = 2  # Hz
AGENT_CAPABILITIES = ['camera', 'lidar']
AGENT_MESSAGE_TYPES = ['object_detection', 'object_tracking']
AGENT_TYPE = 'robot'
DISTANCE_THRESHOLD = 5.0

@dataclass
class EntryExit(IdlStruct):
    """
    Represents an entry or exit event of an agent.

    Attributes:
        agent_id (int): The ID of the agent.
        agent_type (str): The type of the agent (e.g., robot, sensor, human).
        action (str): The action performed by the agent (e.g., enter, exit).
        capabilities (sequence[str]): The sensing capabilities of the agent (e.g., camera, lidar).
        message_types (sequence[str]): The message topics this agent will publish.
        ip_address (str): The IP address of the agent.
        timestamp (int): The timestamp of the message.
    """
    agent_id: int
    agent_type: str
    action: str
    capabilities: sequence[str]
    message_types: sequence[str]
    ip_address: str
    timestamp: int

@dataclass
class Heartbeat(IdlStruct):
    """
    Represents a heartbeat message from an agent.
    
    Attributes:
        agent_id (int): The ID of the agent sending the heartbeat.
        timestamp (int): The timestamp of the heartbeat message.
    """
    agent_id: int
    timestamp: int
    location_valid: bool
    x: float
    y: float
    theta: float

@dataclass 
class Initialization(IdlStruct):
    """
    Represents the initialization parameters for the agent entry/exit system.

    Attributes:
        sending_agent (str): A json dict of the sending agent.
        agents (str): A json dict of all the agents that the sending_agent is aware of.
        map (str): A json of the ROS map message (Occupancy Grid) that the sending agent has.
        map_md (str): A json of the ROS map metadata message that the sending agent has.
    """
    target_agent: int
    sending_agent: str
    agents: str
    map: str
    map_md: str

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
    - find_if_closest_robot(robot_hash): Determines if the given robot is the closest robot to the current agent.
    - agent_update_available(): Checks if there are updates to be sent to agents.
    - get_agents(): Retrieves the active agents, exited agents, and lost agents.
    - update_agents(agents): Updates the active agents.
    - update_map(map, map_md): Updates the occupancy grid map and map metadata.
    """

    def __init__(self, participant, publisher, subscriber, my_id, my_ip, my_hash, init_writer):
        super().__init__()
        self.participant = participant
        self.publisher = publisher
        self.subscriber = subscriber
        self.agents = dict()
        self.exited_agents = dict()
        self.lost_agents = dict()
        self.my_id = my_id
        self.my_ip = my_ip
        self.my_hash = my_hash
        self.map_msg = OccupancyGrid()
        self.map_md_msg = MapMetaData()
        self.init_writer = init_writer

        self.update_to_agents = False

    def on_data_available(self, reader):
        """
        Callback method for handling incoming data.

        Parameters:
        - reader (Reader): The DDS reader.

        Returns:
        - None
        """
        for sample in reader.read():
            # print(sample)

            if sample.agent_id == int(self.my_id):
                continue

            # Determine what type of message was received
            if sample.action == 'enter':
                print(f'Agent {sample.agent_id} entered the environment')
                agent_type = sample.agent_type
                capabilities = sample.capabilities
                message_types = sample.message_types
                ip_address = sample.ip_address
                new_robot_hash = hash_id(str(sample.agent_id))
                am_closest_robot = self.find_if_closest_robot(new_robot_hash)
                self.agents[sample.agent_id] = {
                    'agent_type': agent_type,
                    'capabilities': capabilities,
                    'message_types': message_types,
                    'ip_address': ip_address,
                    'hash': new_robot_hash,
                    'timestamp': sample.timestamp
                }  
                self.update_to_agents = True
                if am_closest_robot:
                    my_dict = {
                        'id': int(self.my_id),
                        'agent_type': AGENT_TYPE,
                        'capabilities': AGENT_CAPABILITIES,
                        'message_types': AGENT_MESSAGE_TYPES,
                        'ip_address': self.my_ip,
                        'hash': self.my_hash,
                        'timestamp': int(time.time())
                    }
                    sending_agent = json.dumps(my_dict)

                    if len(self.agents) > 0:
                        agents_message = json.dumps(self.agents)
                    else:
                        agents_message = json.dumps("")

                    map_dict = message_converter.convert_ros_message_to_dictionary(self.map_msg)
                    map_json = json.dumps(map_dict)
                    map_md_dict = message_converter.convert_ros_message_to_dictionary(self.map_md_msg)
                    map_md_json = json.dumps(map_md_dict)

                    init_message = Initialization(target_agent=sample.agent_id, sending_agent=sending_agent, agents=agents_message, map=map_json, map_md=map_md_json)
                    self.init_writer.write(init_message)

                    print("Sent initialization message to new agent")
            elif sample.action == 'exit':
                # Agent Exited, remove from agents dictionary
                if sample.agent_id in self.agents:
                    print(f'Agent {sample.agent_id} exited the environment')
                    self.exited_agents[sample.agent_id] = self.agents.pop(sample.agent_id)
                    self.update_to_agents = True

    def find_if_closest_robot(self, robot_hash):
        """
        Finds if the given robot is the closest robot to the current agent. 
        The closest robot is the robot that has the smallest difference in hash value
        compared to the current agent after dividing by the number of agents.

        Parameters:
        - robot_hash (int): The hash value of the robot.

        Returns:
        - bool: True if the given robot is the closest robot, False otherwise.
        """
        num_agents = len(self.agents) + 1
        my_distance = abs(self.my_hash / num_agents - robot_hash / num_agents)

        for agent_id, agent_info in self.agents.items():
            agent_hash = agent_info['hash']

            distance = abs(agent_hash / num_agents - robot_hash / num_agents)
            if distance < my_distance:
                print("I am not the closest robot")
                return False

        print('I will provide initial details to the new agent')
        return True

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
        return self.agents, self.exited_agents, self.lost_agents

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
        if exited_agents is not None:
           self.exited_agents = exited_agents
        if lost_agents is not None:
            self.lost_agents = lost_agents
    
    def update_map(self, map, map_md):
        """
        Updates the occupancy grid map and map metadata.

        Parameters:
        - map (OccupancyGrid): The updated occupancy grid map.
        - map_md (MapMetaData): The updated map metadata.

        Returns:
        - None
        """
        self.map_msg = map
        self.map_md_msg = map_md

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
        self.locations = dict()
        self.my_id = my_id
        self.agents = dict()

    def on_data_available(self, reader):
        """
        Callback method called when data is available in the reader.

        Args:
            reader (DataReader): The DataReader object.

        Returns:
            None
        """
        for sample in reader.read():

            if sample.agent_id == int(self.my_id):
                continue
            
            print(f'Heartbeat from agent {sample.agent_id} at time {sample.timestamp}')
            
            if sample.agent_id in self.agents:
                self.heartbeats[sample.agent_id] = sample.timestamp

                if sample.location_valid:
                    self.locations[sample.agent_id] = (sample.x, sample.y, sample.theta)
                else:
                    self.locations[sample.agent_id] = None
                    
            else:
                print(f'Heartbeat from Agent {sample.agent_id}, but is not in the environment')
                

    def get_heartbeats(self):
        """
        Get a copy of the heartbeats dictionary.

        Returns:
            dict: A copy of the heartbeats dictionary.
        """
        return self.heartbeats.copy()
    
    def get_heartbeats_and_locations(self):
        """
        Get a copy of the heartbeats and locations dictionaries.

        Returns:
            tuple: A tuple containing copies of the heartbeats and locations dictionaries.
        """
        return self.heartbeats.copy(), self.locations

    def update_agents(self, agents):
        """
        Update the agents dictionary and heartbeats dictionary.

        Args:
            agents (dict): A dictionary containing information about all agents in the environment.

        Returns:
            None
        """
        self.agents = agents
        # Check for robot id in self.agents that isn't in self.heartbeats
        for agent_id in self.agents.keys():
            if agent_id not in self.heartbeats:
                self.heartbeats[agent_id] = self.agents[agent_id]['timestamp']

        # Remove any heartbeats of agents that no longer exist
        for agent_id in list(self.heartbeats.keys()):
            if agent_id not in self.agents:
                self.heartbeats.pop(agent_id)

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

    def __init__(self, my_id, map_publisher, map_md_publisher):
        super().__init__()
        self.map_received = False
        self.map_msg = OccupancyGrid()
        self.map_md_msg = MapMetaData()
        self.agents = dict()
        self.my_id = my_id
        self.map_publisher = map_publisher
        self.map_md_publisher = map_md_publisher

    def on_data_available(self, init_reader):
        """
        Callback function called when initialization data is available.

        Args:
            init_reader: Reader object for reading initialization data.
        """
        for sample in init_reader.read():

            sending_agent_dict = json.loads(sample.sending_agent)
            print(f'Initialization message received from agent {sending_agent_dict["id"]}')
            if sending_agent_dict['id'] == int(self.my_id):
                continue

            if sample.target_agent != int(self.my_id):
                continue

            print(f'Initialization message received from agent {sending_agent_dict["id"]}')

            self.agents[sending_agent_dict['id']] = {
                'agent_type': sending_agent_dict['agent_type'],
                'capabilities': sending_agent_dict['capabilities'],
                'message_types': sending_agent_dict['message_types'],
                'ip_address': sending_agent_dict['ip_address'],
                'hash': sending_agent_dict['hash'],
                'timestamp': sending_agent_dict['timestamp']
            }

            agent_dict = json.loads(sample.agents)
            if len(agent_dict) > 0:
                # Cycle through agents in the initialization message and insert into our agents dictionary
                for agent_id, agent_info in agent_dict.items():
                    if agent_id != self.my_id:
                        agent_type = agent_info['agent_type']
                        capabilities = agent_info['capabilities']
                        message_types = agent_info['message_types']
                        ip_address = agent_info['ip_address']
                        agent_hash = agent_info['hash']
                        timestamp = agent_info['timestamp']
                        self.agents[agent_id] = {
                            'agent_type': agent_type,
                            'capabilities': capabilities,
                            'message_types': message_types,
                            'ip_address': ip_address,
                            'hash': agent_hash,
                            'timestamp': timestamp
                        }  

            # Load the map from the initialization message
            map_dict = json.loads(sample.map)
            map_md_dict = json.loads(sample.map_md)

            load_time = rospy.Time.now()

            # Create the OccupancyGrid message
            self.map_msg.header.stamp = load_time
            self.map_msg.header.frame_id = 'map'
            self.map_msg.info.map_load_time = rospy.Time.now()
            self.map_msg.info.resolution = map_md_dict['resolution']
            self.map_msg.info.width = map_md_dict['width']
            self.map_msg.info.height = map_md_dict['height']
            self.map_msg.info.origin.position.x = map_md_dict['origin']['position']['x']
            self.map_msg.info.origin.position.y = map_md_dict['origin']['position']['y']
            self.map_msg.info.origin.position.z = map_md_dict['origin']['position']['z']
            self.map_msg.info.origin.orientation.x = map_md_dict['origin']['orientation']['x']
            self.map_msg.info.origin.orientation.y = map_md_dict['origin']['orientation']['y']
            self.map_msg.info.origin.orientation.z = map_md_dict['origin']['orientation']['z']
            self.map_msg.info.origin.orientation.w = map_md_dict['origin']['orientation']['w']
            self.map_msg.data = map_dict['data']

            # Create map metadata message
            self.map_md_msg = MapMetaData()
            self.map_md_msg.map_load_time = load_time
            self.map_md_msg.resolution = map_md_dict['resolution']
            self.map_md_msg.width = map_md_dict['width']
            self.map_md_msg.height = map_md_dict['height']
            self.map_md_msg.origin.position.x = map_md_dict['origin']['position']['x']
            self.map_md_msg.origin.position.y = map_md_dict['origin']['position']['y']
            self.map_md_msg.origin.position.z = map_md_dict['origin']['position']['z']
            self.map_md_msg.origin.orientation.x = map_md_dict['origin']['orientation']['x']
            self.map_md_msg.origin.orientation.y = map_md_dict['origin']['orientation']['y']
            self.map_md_msg.origin.orientation.z = map_md_dict['origin']['orientation']['z']
            self.map_md_msg.origin.orientation.w = map_md_dict['origin']['orientation']['w']

            # Publish the map and map metadata
            self.map_publisher.publish(self.map_msg)
            self.map_md_publisher.publish(self.map_md_msg)

            self.map_received = True

            print("Map received through initialization message")

    def map_available(self):
        """
        Check if the map has been received.

        Returns:
            bool: True if the map has been received, False otherwise.
        """
        return self.map_received

    def get_map(self):
        """
        Get the map and map metadata.

        Returns:
            tuple: A tuple containing the map message and map metadata message.
        """
        return self.map_msg, self.map_md_msg

    def get_agents(self):
        """
        Get the agents dictionary.

        Returns:
            dict: Dictionary containing information about the agents.
        """
        return self.agents

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

    def __init__(self, my_id):
        super().__init__()
        self.my_id = my_id
        self.agent_ids = []
        self.locations = dict()

    def on_data_available(self, reader):
        """
        Callback method called when data is available.

        Args:
            reader: The data reader object.

        Returns:
            None
        """
        for sample in reader.read():
            if sample.agent_id == int(self.my_id):
                continue
            
            if sample.agent_id in self.agent_ids:
                print(f'Location message from agent {sample.agent_id} at time {sample.timestamp}')
                self.locations[sample.agent_id] = (sample.x, sample.y, sample.theta)

    def get_locations(self):
        """
        Returns the locations dictionary.

        Returns:
            dict: Dictionary containing agent locations.
        """
        return self.locations

    def set_agent_ids(self, agent_ids):
        """
        Sets the agent IDs and updates the locations dictionary.

        Args:
            agent_ids (list): List of agent IDs.

        Returns:
            None
        """
        self.agent_ids = agent_ids
        # pop all location values that are not in agent_ids
        for agent_id in list(self.locations.keys()):
            if agent_id not in agent_ids:
                self.locations.pop(agent_id)

def hash_id(robot_id):
    """
    Hashes the given robot ID using SHA-256 algorithm.

    Parameters:
    robot_id (str): The robot ID to be hashed.

    Returns:
    int: The hashed robot ID as an integer.

    """
    return int(hashlib.sha256(robot_id.encode()).hexdigest(), 16)

class EntryExitCommunication:

    def __init__(self, server_url='http://192.168.50.2:8000/graphql'):

        # Get robot ID, Hash, and IP Address
        self.my_id = os.environ.get('ROBOT_ID')
        self.my_hash = self.hash_id(self.my_id)

        # Get IP Address
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # This doesn't have to be reachable; it just has to be a valid address
        s.connect(("8.8.8.8", 80))
        self.my_ip = s.getsockname()[0]
        s.close()
        print(f"My IP address is {self.my_ip}")

        # Dictionary to store agents in the environment
        self.agents = dict()
        self.exited_agents = dict()
        self.lost_agents = dict()

        # Map and Map Metadata messages, and publishers
        self.map_msg = OccupancyGrid()
        self.map_md_msg = MapMetaData()

        self.map_publisher = rospy.Publisher('map', OccupancyGrid, queue_size=10)
        self.map_md_publisher = rospy.Publisher('map_metadata', MapMetaData, queue_size=10)

        # Create a DomainParticipant, Subscriber, and Publisher
        self.participant = DomainParticipant()
        self.subscriber = Subscriber(self.participant)
        self.publisher = Publisher(self.participant)

        # Create the topics needed
        self.entry_exit_topic = Topic(self.participant, 'EntryExitTopic', EntryExit)
        self.heartbeat_topic = Topic(self.participant, 'HeartbeatTopic', Heartbeat)
        self.init_topic = Topic(self.participant, 'InitializationTopic', Initialization)
        self.location_topic = Topic(self.participant, 'LocationTopic'+ str(self.my_id), Location)

        # Create the DataWriters and DataReaders
        self.enter_exit_writer = DataWriter(self.publisher, self.entry_exit_topic)
        self.heartbeat_writer = DataWriter(self.publisher, self.heartbeat_topic)
        self.init_writer = DataWriter(self.publisher, self.init_topic)
        self.location_writer = DataWriter(self.publisher, self.location_topic)

        self.entry_exit_listener = EntryExitListener(self.participant, self.publisher, self.subscriber, self.my_id, self.my_ip, self.my_hash, self.init_writer)
        self.heartbeat_listener = HeartbeatListener(self.my_id)
        self.init_listener = InitializationListener(self.my_id, self.map_publisher, self.map_md_publisher)
        self.location_listener = LocationListener(self.my_id)

        # We will start the readers later when it is necessary
        self.enter_exit_reader = None
        self.init_reader = None
        self.heartbeat_reader = None
        self.location_readers = dict()

        # Built-in reader to detect number of participants
        self.built_in_reader = BuiltinDataReader(self.participant, BuiltinTopicDcpsParticipant)
        self.num_participants = 0

        # GraphQL server URL
        self.graphql_server = server_url

        # TF Listener to get current robot position
        self.trans_listener = tf.TransformListener()
        self.my_location = None

        # ROS publisher for publishing nearby agents locations
        self.agent_locations_publisher = rospy.Publisher('agent_locations', AgentLocationsArray, queue_size=10)

    def hash_id(self, robot_id):
        """
        Hashes the given robot ID using SHA-256 algorithm.

        Parameters:
        robot_id (str): The robot ID to be hashed.

        Returns:
        int: The hashed robot ID as an integer.
        """
        return int(hashlib.sha256(robot_id.encode()).hexdigest(), 16)

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

        # Determine the number of participants on the DDS network
        for _ in self.built_in_reader.take_iter(timeout=duration(milliseconds=100)):
            self.num_participants += 1

        if self.num_participants == 1:
            # We are the first participant, we are responsible for getting the map
            print('I am the first agent to enter the environment')

            map_query = """ 
                            {
                                map {
                                    width
                                    height
                                    origin_x
                                    origin_y
                                    origin_z
                                    origin_orientation_x
                                    origin_orientation_y
                                    origin_orientation_z
                                    origin_orientation_w
                                    resolution
                                    occupancy
                                }
                            }
                        """
            have_map = False
            while not have_map:  # Retry until we are able to get the map
                try:
                    # Get the map from the GraphQL server
                    response = requests.post(self.graphql_server, json={'query': map_query}, timeout=1)
                    if response.status_code == 200:
                        data = response.json()
                        map_data = data.get('data', {}).get('map', {})
                    
                        have_map = True

                        # Convert the strings into the ROS Occupancy grid
                        self.map_msg.header.frame_id = 'map'
                        self.map_msg.info.width = map_data.get('width')
                        self.map_msg.info.height = map_data.get('height')
                        self.map_msg.info.resolution = map_data.get('resolution')
                        self.map_msg.info.origin.position.x = map_data.get('origin_x')
                        self.map_msg.info.origin.position.y = map_data.get('origin_y')
                        self.map_msg.info.origin.position.z = map_data.get('origin_z')
                        self.map_msg.info.origin.orientation.x = map_data.get('origin_orientation_x')
                        self.map_msg.info.origin.orientation.y = map_data.get('origin_orientation_y')
                        self.map_msg.info.origin.orientation.z = map_data.get('origin_orientation_z')
                        self.map_msg.info.origin.orientation.w = map_data.get('origin_orientation_w')
                        self.map_msg.data = map_data.get('occupancy')

                        self.map_md_msg.map_load_time = rospy.Time.now()
                        self.map_md_msg.resolution = map_data.get('resolution')
                        self.map_md_msg.width = map_data.get('width')
                        self.map_md_msg.height = map_data.get('height')
                        self.map_md_msg.origin.position.x = map_data.get('origin_x')
                        self.map_md_msg.origin.position.y = map_data.get('origin_y')
                        self.map_md_msg.origin.position.z = map_data.get('origin_z')
                        self.map_md_msg.origin.orientation.x = map_data.get('origin_orientation_x')
                        self.map_md_msg.origin.orientation.y = map_data.get('origin_orientation_y')
                        self.map_md_msg.origin.orientation.z = map_data.get('origin_orientation_z')
                        self.map_md_msg.origin.orientation.w = map_data.get('origin_orientation_w')

                        self.entry_exit_listener.update_map(self.map_msg, self.map_md_msg)

                        # Publish the map and map metadata for ROS nodes
                        self.map_publisher.publish(self.map_msg)
                        self.map_md_publisher.publish(self.map_md_msg)

                        print("Map retrieved from GraphQL Server")

                        # Start the readers now that we have the map
                        self.enter_exit_reader = DataReader(self.subscriber, self.entry_exit_topic, listener=self.entry_exit_listener)
                        self.init_reader = DataReader(self.subscriber, self.init_topic, listener=self.init_listener)
                        self.heartbeat_reader = DataReader(self.subscriber, self.heartbeat_topic, listener=self.heartbeat_listener)
                    else:
                        print(f"Error retrieving map: {response.status_code}")
                except Exception as e:
                    print(f"Error retrieving map: {e}")
                time.sleep(1)
        else:
            # We are not the first participant, we will get the map from one of the other agents
            print('I am not the first agent to enter the environment')

            # We start the readers now since we will need them to access map information
            self.enter_exit_reader = DataReader(self.subscriber, self.entry_exit_topic, listener=self.entry_exit_listener)
            self.init_reader = DataReader(self.subscriber, self.init_topic, listener=self.init_listener)

            # Broadcast an entry message
            entry_message = EntryExit(int(self.my_id), AGENT_TYPE, 'enter', AGENT_CAPABILITIES, AGENT_MESSAGE_TYPES, self.my_ip, int(time.time()))
            self.enter_exit_writer.write(entry_message)

            # Wait for the map to become available
            while not self.init_listener.map_available():
                print("No Map yet...")
                time.sleep(1)
                if not self.init_listener.map_available():
                    entry_message.timestamp = int(time.time())
                    self.enter_exit_writer.write(entry_message)

            # Store the map, map metadata, and agents
            self.map_msg, self.map_md_msg = self.init_listener.get_map()
            self.agents = self.init_listener.get_agents()

            # Update the agents in the entry/exit listener
            self.entry_exit_listener.update_agents(agents=self.agents)

            # Publish the map and map metadata for ROS nodes
            self.map_publisher.publish(self.map_msg)
            self.map_md_publisher.publish(self.map_md_msg)

            # Start the heartbeat reader now that we have the map, stop listening for initialization messages
            self.init_reader = None
            self.init_listener = None
            self.heartbeat_reader = DataReader(self.subscriber, self.heartbeat_topic, listener=self.heartbeat_listener)

            print("Initialization complete")

    def run(self):
        """
        Executes the main loop of the agent_entry_exit node.
        This method continuously checks for new agents, updates the heartbeat of existing agents,
        sends out heartbeat messages, checks for dead agents, and removes them from the agent list.

        Returns:
            None
        """

        prev_nearby_agents = []

        # Loop through at the rate we publish location
        rate = rospy.Rate(LOCATION_FREQUENCY)
        last_time = int(time.time())
        while not rospy.is_shutdown():
            current_time = int(time.time())

            # Get current position of the agent and publish to location topic
            location_valid = False
            try:
                (translation, rotation) = self.trans_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
                x = translation[0]
                y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                theta = euler[2]

                self.my_location = (x, y, theta)
                print(f'Current position: ({self.x}, {self.y}, {self.theta})')
                location_valid = True

                # Publish to this agent's location topic
                location_message = Location(int(self.my_id), current_time, x, y, theta)
                self.location_writer.write(location_message)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("Location not available yet")
                # x = 1
                # y = 2
                # theta = 3
                # self.my_location = (x, y, theta)
                # location_valid = True
                # location_message = Location(int(self.my_id), current_time, x, y, theta)
                # self.location_writer.write(location_message)

            nearby_agents_locations = self.location_listener.get_locations()
            agent_locations_array = AgentLocationsArray()
            agent_locations_array.header.stamp = rospy.Time.now()
            agent_locations_array.header.frame_id = 'map'
            agent_list = []
            agent_location_list = []
            for agent_id, location in nearby_agents_locations.items():
                id_msg = Int32()
                id_msg.data = int(agent_id)
                agent_list.append(id_msg)   
                agent_pose = Pose()
                agent_pose.position.x = location[0]
                agent_pose.position.y = location[1]
                agent_pose.position.z = 0
                quat = tf.transformations.quaternion_from_euler(0, 0, location[2])
                agent_pose.orientation.x = quat[0]
                agent_pose.orientation.y = quat[1]
                agent_pose.orientation.z = quat[2]
                agent_pose.orientation.w = quat[3]
                agent_location_list.append(agent_pose)
            agent_locations_array.agentIDs = agent_list
            agent_locations_array.locations = agent_location_list

            if len(agent_list):
                self.agent_locations_publisher.publish(agent_locations_array)
                
            # Now publish heartbeat periodically
            if current_time - last_time >= HEARTBEAT_PERIOD:
                last_time = current_time
                
                # Check for new agents
                if self.entry_exit_listener.agent_update_available():
                    self.agents, self.exited_agents, self.lost_agents = self.entry_exit_listener.get_agents()
                current_agents_list = list(self.agents.keys())

                self.heartbeat_listener.update_agents(self.agents)

                # Send out heartbeat
                if location_valid:
                    heartbeat_message = Heartbeat(int(self.my_id), current_time, location_valid, x, y, theta)
                else:
                    heartbeat_message = Heartbeat(int(self.my_id), current_time, location_valid, 0.0, 0.0, 0.0)
                self.heartbeat_writer.write(heartbeat_message)

                # Update agents with new heartbeats
                heartbeats, locations = self.heartbeat_listener.get_heartbeats_and_locations()
                for agent_id, timestamp in heartbeats.items():
                    if agent_id in current_agents_list:
                        self.agents[agent_id]['timestamp'] = timestamp

                nearby_agents = []
                for agent_id, location in locations.items():
                    if location is not None and agent_id in current_agents_list:
                        x, y, theta = location
                        self.agents[agent_id]['location'] = (x, y, theta)

                        # Determine if the agent is close to the robot
                        if self.my_location is not None:
                            distance = ((x - self.my_location[0])**2 + (y - self.my_location[1])**2)**0.5
                            if distance < DISTANCE_THRESHOLD:
                                print(f'Agent {agent_id} is close to the robot')
                                nearby_agents.append(agent_id)
                                if 'agent_id' not in list(self.location_readers.keys()):
                                    new_location_topic = Topic(self.participant, 'LocationTopic' + str(agent_id), Location)
                                    self.location_readers[agent_id] = DataReader(self.subscriber, new_location_topic, listener=self.location_listener)
                
                # Only need to perform this housekeeping if the list of nearby agents has changed
                if nearby_agents != prev_nearby_agents:
                    prev_nearby_agents = nearby_agents
                   
                    # Remove readers that are no longer needed
                    for agent_id in list(self.location_readers.keys()):
                        if agent_id not in nearby_agents:
                            self.location_readers.pop(agent_id)
                    self.location_listener.set_agent_ids(nearby_agents)  # update the list of agents we should be listening for

                # Check Periodically for Dead Agents
                dead_agents = []
                for agent_id, agent_info in self.agents.items():
                    time_difference = current_time - agent_info['timestamp']

                    if time_difference > HEARTBEAT_TIMEOUT:
                        # Agent has timed out
                        print(f'Agent {agent_id} has timed out')
                        dead_agents.append(agent_id)
                
                # Remove Dead Agents
                for agent_id in dead_agents:
                    self.lost_agents[agent_id] = self.agents.pop(agent_id)
                if dead_agents:
                    self.entry_exit_listener.update_agents(agents=self.agents, lost_agents=self.lost_agents)

            rate.sleep()

    def shutdown(self):
        print('Shutting down...')
        # Write exit message
        exit_message = EntryExit(int(self.my_id), AGENT_TYPE, 'exit', [], [], self.my_ip, int(time.time()))
        self.enter_exit_writer.write(exit_message)


if __name__ == '__main__':

    rospy.init_node('agent_entry_exit', anonymous=True)
    entry_exit_obj = EntryExitCommunication()
    rospy.on_shutdown(entry_exit_obj.shutdown)
    entry_exit_obj.setup_and_run()
    