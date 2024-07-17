import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rospy_message_converter import message_converter

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
HEARTBEAT_FREQUENCY = 2
HEARTBEAT_TIMEOUT = 15
AGENT_CAPABILITIES = ['camera', 'lidar']
AGENT_MESSAGE_TYPES = ['object_detection', 'object_tracking']
AGENT_TYPE = 'robot'


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
            else:
                print(f'Heartbeat from Agent {sample.agent_id}, but is not in the environment')

    def get_heartbeats(self):
        """
        Get a copy of the heartbeats dictionary.

        Returns:
            dict: A copy of the heartbeats dictionary.
        """
        return self.heartbeats.copy()

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
        self.my_hash = hash_id(self.my_id)

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

        # Create the DataWriters and DataReaders
        self.enter_exit_writer = DataWriter(self.publisher, self.entry_exit_topic)
        self.heartbeat_writer = DataWriter(self.publisher, self.heartbeat_topic)
        self.init_writer = DataWriter(self.publisher, self.init_topic)

        self.entry_exit_listener = EntryExitListener(self.participant, self.publisher, self.subscriber, self.my_id, self.my_ip, self.my_hash, self.init_writer)
        self.heartbeat_listener = HeartbeatListener(self.my_id)
        self.init_listener = InitializationListener(self.my_id, self.map_publisher, self.map_md_publisher)
        self.enter_exit_reader = DataReader(self.subscriber, self.entry_exit_topic, listener=self.entry_exit_listener)
        self.init_reader = DataReader(self.subscriber, self.init_topic, listener=self.init_listener)
        self.heartbeat_reader = DataReader(self.subscriber, self.heartbeat_topic, listener=self.heartbeat_listener)

        self.built_in_reader = BuiltinDataReader(self.participant, BuiltinTopicDcpsParticipant)
        self.num_participants = 0
        self.graphql_server = server_url

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
        for _ in self.built_in_reader.take_iter(timeout=duration(milliseconds=100)):
            self.num_participants += 1

        if self.num_participants == 1:
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
            while not have_map:
                try:
                    # Get the map
                    response = requests.post(self.graphql_server, json={'query': map_query})
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

                        self.map_publisher.publish(self.map_msg)
                        self.map_md_publisher.publish(self.map_md_msg)

                        print("Map retrieved from GraphQL Server")

                    else:
                        print(f"Error retrieving map: {response.status_code}")
                except Exception as e:
                    print(f"Error retrieving map: {e}")
                time.sleep(1)
        else:
            print('I am not the first agent to enter the environment')

            entry_message = EntryExit(int(self.my_id), AGENT_TYPE, 'enter', AGENT_CAPABILITIES, AGENT_MESSAGE_TYPES, self.my_ip, int(time.time()))
            self.enter_exit_writer.write(entry_message)

            while not self.init_listener.map_available():
                print("No Map yet...")
                time.sleep(1)
                if not self.init_listener.map_available():
                    entry_message.timestamp = int(time.time())
                    self.enter_exit_writer.write(entry_message)

            self.map_msg, self.map_md_msg = self.init_listener.get_map()
            self.agents = self.init_listener.get_agents()

            self.entry_exit_listener.update_agents(agents=self.agents)

            self.map_publisher.publish(self.map_msg)
            self.map_md_publisher.publish(self.map_md_msg)

            self.init_reader = None
            self.init_listener = None

            print("Initialization complete")

    def run(self):
        """
        Executes the main loop of the agent_entry_exit node.
        This method continuously checks for new agents, updates the heartbeat of existing agents,
        sends out heartbeat messages, checks for dead agents, and removes them from the agent list.

        Returns:
            None
        """
        while not rospy.is_shutdown():
            current_time = int(time.time())
                
            # Check for new agents
            if self.entry_exit_listener.agent_update_available():
                self.agents, self.exited_agents, self.lost_agents = self.entry_exit_listener.get_agents()

            self.heartbeat_listener.update_agents(self.agents)

            # Send out heartbeat
            heartbeat_message = Heartbeat(int(self.my_id), current_time)
            self.heartbeat_writer.write(heartbeat_message)

            heartbeats = self.heartbeat_listener.get_heartbeats()
            for agent_id, timestamp in heartbeats.items():
                self.agents[agent_id]['timestamp'] = timestamp

            # Check Periodically for Dead Agents
            dead_agents = []
            for agent_id, agent_info in self.agents.items():
                time_difference = current_time - agent_info['timestamp']

                if time_difference > HEARTBEAT_TIMEOUT:
                    print(f'Agent {agent_id} has not sent a heartbeat in too long')
                    dead_agents.append(agent_id)
            
            # Remove Dead Agents
            for agent_id in dead_agents:
                self.lost_agents[agent_id] = self.agents.pop(agent_id)
            if dead_agents:
                self.entry_exit_listener.update_agents(agents=self.agents, lost_agents=self.lost_agents)
            time.sleep(HEARTBEAT_FREQUENCY)

    def shutdown(self):
        print('Shutting down...')
        
        exit_message = EntryExit(int(self.my_id), AGENT_TYPE, 'exit', [], [], self.my_ip, int(time.time()))
        self.enter_exit_writer.write(exit_message)


if __name__ == '__main__':

    rospy.init_node('agent_entry_exit', anonymous=True)
    entry_exit_obj = EntryExitCommunication()
    rospy.on_shutdown(entry_exit_obj.shutdown)
    entry_exit_obj.setup_and_run()
    