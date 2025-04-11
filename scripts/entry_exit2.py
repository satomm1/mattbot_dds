import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from mattbot_dds.msg import AgentLocationsArray
from mattbot_dds.msg import AgentSubscription
from geometry_msgs.msg import Pose, Pose2D
from std_msgs.msg import Header, Int32, Float64MultiArray
from rospy_message_converter import message_converter
import tf
import rospkg

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
import hashlib
import socket
import json
import requests
import numpy as np

from dds_utils import EntryExit, Heartbeat, Initialization, Location, DataMessage, reliable_qos, best_effort_qos

# Constants (Set depending on the agent)
HEARTBEAT_PERIOD = 10    # seconds
HEARTBEAT_TIMEOUT = 31  # seconds
LOCATION_FREQUENCY = 1  # Hz
AGENT_CAPABILITIES = ['camera', 'lidar']
AGENT_MESSAGE_TYPES = ['object_detection', 'object_tracking']
AGENT_TYPE = 'robot'
DISTANCE_THRESHOLD = 5.0
SENSOR_AGENT_START = 200  # The id of agents which are sensor's only


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
        self.agents[my_hash] = {
            'agent_type': AGENT_TYPE,
            'ip_address': my_ip,
            'hash': my_hash
        }  

        self.my_id = my_id
        self.my_ip = my_ip
        self.my_hash = my_hash
        self.map_msg = OccupancyGrid()
        self.map_mod_msg = OccupancyGrid()
        self.map_md_msg = MapMetaData()
        self.known_points = []
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

            if sample.agent_id == int(self.my_id):
                # Ignore messages from self
                continue

            # Determine if entry or exit message
            if sample.action == 'enter':
                new_robot_hash = hash_func(str(sample.agent_id))
                # If the new agent is the closest robot, send an initialization message
                # The initalization message contains the map, map metadata, and all agents in the environment
                if self.find_if_closest_robot(new_robot_hash):
                    print(f'Agent {sample.agent_id} of type \'{sample.agent_type}\' is requesting entry')

                    # Message containing details of all active agents
                    agents_message = json.dumps(self.agents)

                    known_points_json = json.dumps(self.known_points)

                    init_message = Initialization(target_agent=sample.agent_id, sending_agent=int(self.my_id), agents=agents_message, known_points=known_points_json)
                    self.init_writer.write(init_message)

                    print(f'Sent initialization message to agent {sample.agent_id}')
            elif sample.action == 'initialized':
                
                # Only if the sample.timestamp is recent
                if int(time.time()) - sample.timestamp < 10: 
                    print(f'Agent {sample.agent_id} of type \'{sample.agent_type}\' entered the environment')

                    # Agent initialized, add to agents dictionary
                    new_robot_hash = hash_func(str(sample.agent_id))
                    self.agents[sample.agent_id] = {
                        'agent_type': sample.agent_type,
                        'ip_address': sample.ip_address,
                        'hash': new_robot_hash,
                        'timestamp': sample.timestamp
                    }

                    self.update_to_agents = True
            elif sample.action == 'exit':
                # Agent Exited, remove from agents dictionary
                if sample.agent_id in self.agents:
                    print(f'Agent {sample.agent_id} exited the environment')
                    self.agents.pop(sample.agent_id)
                    self.update_to_agents = True

    def find_if_closest_robot(self, robot_hash):
        """
        Finds if the given robot is the closest robot to the current agent. 
        The closest robot is the robot that has the smallest difference in hash value

        Parameters:
        - robot_hash (int): The hash value of the robot.

        Returns:
        - bool: True if the given robot is the closest robot, False otherwise.
        """
        my_distance = abs(self.my_hash - robot_hash)

        # Loop through all agents to see if there is a closer robot (by hash)
        for agent_id, agent_info in self.agents.items():
            agent_hash = agent_info['hash']

            distance = abs(agent_hash - robot_hash)
            if distance < my_distance and distance != 0:
                print("I will not provide initialization.")  # I am not the closest robot
                return False

        print('I will provide initialization.')  # I am the closest robot
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
        return self.agents

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
    
    # def update_map(self, my_map, my_map_mod, map_md):
    #     """
    #     Updates the occupancy grid map and map metadata.

    #     Parameters:
    #     - map (OccupancyGrid): The updated occupancy grid map.
    #     - map_md (MapMetaData): The updated map metadata.

    #     Returns:
    #     - None
    #     """
    #     self.map_msg = my_map
    #     self.map_mod_msg = my_map_mod
    #     self.map_md_msg = map_md

    def update_known_points(self, known_points):
        """
        Updates the known points in the environment.

        Parameters:
        - known_points (list): A list of known points in the environment.

        Returns:
        - None
        """
        self.known_points = known_points

class HeartbeatListener(Listener):
    """
    Listener class that handles heartbeat data from agents.

    Attributes:
        heartbeats (dict): A dictionary to store the heartbeats of agents.
        locations (dict): A dictionary to store the locations of agents.
        my_id (int): The ID of the current agent.
        agents (dict): A dictionary to store information about all agents in the environment.
    """

    def __init__(self, my_id):
        super().__init__()
        self.heartbeats = dict()
        self.new_heartbeats = dict()
        self.locations = dict()
        self.new_locations = dict()
        self.my_id = my_id

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
        Callback method called when data is available in the reader.

        Args:
            reader (DataReader): The DataReader object.

        Returns:
            None
        """
        for sample in reader.read():

            if sample.agent_id == int(self.my_id):
                # Ignore messages from self
                continue
            
            self.new_heartbeats[sample.agent_id] = sample.timestamp
            self.heartbeats[sample.agent_id] = sample.timestamp

            if sample.location_valid:

                new_point = self.transform_point([sample.x, sample.y, sample.theta], forward=False)
                x = new_point[0]
                y = new_point[1]
                theta = new_point[2]

                self.locations[sample.agent_id] = (x, y, theta)
                self.new_locations[sample.agent_id] = (x, y, theta)
            else:
                self.locations[sample.agent_id] = None
                self.new_locations[sample.agent_id] = None
                
    def get_heartbeats(self):
        """
        Get a copy of the heartbeats dictionary.

        Returns:
            dict: A copy of the heartbeats dictionary.
        """
        returned_heartbeats = self.new_heartbeats.copy()
        return returned_heartbeats
    
    def get_heartbeats_and_locations(self):
        """
        Get a copy of the heartbeats and locations dictionaries.

        Returns:
            tuple: A tuple containing copies of the heartbeats and locations dictionaries.
        """
        returned_heartbeats = self.new_heartbeats.copy()
        self.new_heartbeats = dict()

        returned_locations = self.new_locations.copy() 
        self.new_locations = dict()

        return returned_heartbeats, returned_locations

    # TODO Should provide function to alert of new agents detected through heartbeats

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

    def __init__(self, my_id):
        super().__init__()
        self.map_received = False
        self.map_msg = OccupancyGrid()
        self.map_mod_msg = OccupancyGrid()
        self.map_md_msg = MapMetaData()
        self.agents = dict()
        self.my_id = my_id
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
            if sending_agent == int(self.my_id):
                continue

            print(f'Initialization message received from agent {sending_agent}')

            # Ignore messages not intended for this agent
            if sample.target_agent != int(self.my_id):
                continue

            # Load the agents from the initialization message
            agent_dict = json.loads(sample.agents)
            if len(agent_dict) > 0:
                # Cycle through agents in the initialization message and insert into our agents dictionary
                for agent_id, agent_info in agent_dict.items():
                    if agent_id != self.my_id:
                        self.agents[int(agent_id)] = {
                            'agent_type': agent_info['agent_type'],
                            'ip_address': agent_info['ip_address'],
                            'hash': agent_info['hash'],
                            'timestamp': agent_info['timestamp']
                        }  

            # Load the known points from the initialization message
            known_points = json.loads(sample.known_points)
            self.reference_known_points = known_points
            self.known_points_received = True

            print("Reference points received through initialization message")

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

# FIXME Might not need this...
class LocationListener(Listener):
    """
    Listener class that handles location data for agents.

    Attributes:
        my_id (int): The ID of the listener.
        agent_ids (list): List of agent IDs.
        location (tuple): Tuple to store agent location.

    Methods:
        on_data_available(reader): Callback method called when data is available.
        get_location(): Returns the location tuple.
    """

    def __init__(self, my_id):
        super().__init__()
        self.my_id = my_id
        self.location = None

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

            # Ignore messages from self
            if sample.agent_id == int(self.my_id):
                continue
            
            new_point = self.transform_point([sample.x, sample.y, sample.theta], forward=False)
            self.location = (new_point[0], new_point[1], new_point[2])

    def get_location(self):
        """
        Returns the locations dictionary.

        Returns:
            dict: Tuple containing agent location.
        """
        return self.location


def hash_func(robot_id):
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

        rospy.init_node('agent_entry_exit', anonymous=True)

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

        # Map and Map Metadata messages, and publishers
        self.map_msg = OccupancyGrid()
        self.map_md_msg = MapMetaData()

        self.map_publisher = rospy.Publisher('map', OccupancyGrid, queue_size=10)
        self.map_mod_publisher = rospy.Publisher('map_mod', OccupancyGrid, queue_size=10)
        self.map_md_publisher = rospy.Publisher('map_metadata', MapMetaData, queue_size=10)

        self.lease_duration_ms = 30000
        qos_profile = DomainParticipantQos()
        qos_profile.lease_duration = duration(milliseconds=self.lease_duration_ms)

        # Create a DomainParticipant, Subscriber, and Publisher
        self.participant = DomainParticipant(qos=qos_profile)
        self.subscriber = Subscriber(self.participant)
        self.publisher = Publisher(self.participant)

        # Create the topics needed
        self.entry_exit_topic = Topic(self.participant, 'EntryExitTopic', EntryExit)
        self.heartbeat_topic = Topic(self.participant, 'HeartbeatTopic', Heartbeat)
        self.init_topic = Topic(self.participant, 'InitializationTopic', Initialization)
        self.location_topic = Topic(self.participant, 'LocationTopic'+ str(self.my_id), Location)
        self.data_topic = Topic(self.participant, 'DataTopic' + str(self.my_id), DataMessage)

        # Create the DataWriters and DataReaders
        self.enter_exit_writer = DataWriter(self.publisher, self.entry_exit_topic, qos=reliable_qos)
        # self.heartbeat_writer = DataWriter(self.publisher, self.heartbeat_topic, qos=best_effort_qos)
        self.init_writer = DataWriter(self.publisher, self.init_topic, qos=reliable_qos)
        self.location_writer = DataWriter(self.publisher, self.location_topic, qos=best_effort_qos)
        self.data_writer = DataWriter(self.publisher, self.data_topic, qos=reliable_qos)

        # ROS Publisher for publishing transformation matrix
        self.transform_pub = rospy.Publisher('transformation_matrix', Float64MultiArray, queue_size=10)

        # ROS publisher for publishing external goals
        self.goal_pub = rospy.Publisher('/external_goal', Pose2D, queue_size=10)
        self.agent_sub_pub = rospy.Publisher('/agent_to_subscribe', AgentSubscription, queue_size=10)

        self.entry_exit_listener = EntryExitListener(self.participant, self.publisher, self.subscriber, self.my_id, self.my_ip, self.my_hash, self.init_writer)
        self.heartbeat_listener = HeartbeatListener(self.my_id)
        self.init_listener = InitializationListener(self.my_id)
        # self.my_data_listener = DataListener(self.my_id, self.my_id, self.goal_pub)
        self.agent_data_listeners = dict()

        # We will start the readers later when it is necessary
        self.enter_exit_reader = None
        self.init_reader = None
        self.heartbeat_reader = None
        self.location_readers = dict()
        self.location_listeners = dict()
        # self.my_data_reader = DataReader(self.subscriber, self.data_topic, listener=self.my_data_listener, qos=reliable_qos)
        self.agent_data_listeners = dict()
        self.agent_data_readers = dict()

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
        # Load the map from the current_map.json file and publish it
        self.load_map()
        
        # Now get reference points
        self.known_points = []
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('mattbot_dds')
        with open(os.path.join(package_path, 'scripts', 'known_points.txt'), 'r') as f:
            for line in f:
                x, y = line.split(',')
                self.known_points.append((float(x), float(y)))

        self.entry_exit_listener.update_known_points(self.known_points)

        self.enter_exit_reader = DataReader(self.subscriber, self.entry_exit_topic, listener=self.entry_exit_listener, qos=reliable_qos)
        self.init_reader = DataReader(self.subscriber, self.init_topic, listener=self.init_listener, qos=reliable_qos)

        # Broadcast an entry message
        entry_message = EntryExit(int(self.my_id), AGENT_TYPE, 'enter', self.my_ip, int(time.time()))
        self.enter_exit_writer.write(entry_message)

        # Wait for the reference points to become available
        num_tries = 0
        while not self.init_listener.known_points_available() and num_tries < 10:
            print("Reference Points not yet received (attempt {0}/10)".format(num_tries+1))
            time.sleep(1)
            if not self.init_listener.known_points_available():
                entry_message.timestamp = int(time.time())
                self.enter_exit_writer.write(entry_message)
                num_tries += 1

        if self.init_listener.known_points_available():
            print("I am not the first agent, received reference points")

            # Store the map, map metadata, and agents
            self.reference_known_points = self.init_listener.get_known_points()
            self.agents = self.init_listener.get_agents()

            # Add myself to the agents dictionary
            self.agents[int(self.my_id)] = {
                'agent_type': AGENT_TYPE,
                'ip_address': self.my_ip,
                'hash': self.my_hash,
                'timestamp': int(time.time())
            }

            # Update the agents in the entry/exit listener
            self.entry_exit_listener.update_agents(agents=self.agents)
        else: 
            print("I am the first agent, my map will be the reference map")
            self.reference_known_points = self.known_points

            self.agents[int(self.my_id)] = {
                'agent_type': AGENT_TYPE,
                'ip_address': self.my_ip,
                'hash': self.my_hash,
                'timestamp': int(time.time())
            }

        self.create_transform()  # Create the transform from the known points

        # Update the entry/exit listener with the known points
        self.entry_exit_listener.update_known_points(self.reference_known_points)

        # Update the agents in the entry/exit listener
        self.entry_exit_listener.update_agents(agents=self.agents)  

        # Start the heartbeat reader now that we have the reference points, stop listening for initialization messages
        self.init_reader = None
        self.init_listener = None
        self.heartbeat_reader = DataReader(self.subscriber, self.heartbeat_topic, listener=self.heartbeat_listener, qos=best_effort_qos)

        # Send confirmation message to entry_exit topic
        entry_message = EntryExit(int(self.my_id), AGENT_TYPE, 'initialized', self.my_ip, int(time.time()))
        self.enter_exit_writer.write(entry_message)

        print("Initialization complete")

    def load_map(self):

        # find mattbot_mcl package path
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('mattbot_mcl')

        # load the map from the current_map.json file
        with open(os.path.join(package_path, 'map_json', 'current_map.json'), 'r') as f:
            data = json.load(f)
        map_data = data.get('data', {}).get('map', {})

        with open(os.path.join(package_path, 'map_json', 'current_map_mod.json'), 'r') as f:
            mod_data = json.load(f)
        map_mod_data = mod_data.get('data', {}).get('map', {})

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

        self.map_mod_msg = OccupancyGrid()
        self.map_mod_msg.header.frame_id = 'map'
        self.map_mod_msg.info.width = map_mod_data.get('width')
        self.map_mod_msg.info.height = map_mod_data.get('height')
        self.map_mod_msg.info.resolution = map_mod_data.get('resolution')
        self.map_mod_msg.info.origin.position.x = map_mod_data.get('origin_x')
        self.map_mod_msg.info.origin.position.y = map_mod_data.get('origin_y')
        self.map_mod_msg.info.origin.position.z = map_mod_data.get('origin_z')
        self.map_mod_msg.info.origin.orientation.x = map_mod_data.get('origin_orientation_x')
        self.map_mod_msg.info.origin.orientation.y = map_mod_data.get('origin_orientation_y')
        self.map_mod_msg.info.origin.orientation.z = map_mod_data.get('origin_orientation_z')
        self.map_mod_msg.info.origin.orientation.w = map_mod_data.get('origin_orientation_w')
        self.map_mod_msg.data = map_mod_data.get('occupancy')

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

        # Publish the map and map metadata for ROS nodes
        self.map_publisher.publish(self.map_msg)
        self.map_mod_publisher.publish(self.map_mod_msg)
        self.map_md_publisher.publish(self.map_md_msg)

        print("Map loaded")

    def create_transform(self):
        """
        Determines the transform from my map to the reference map
        """
        self.R = None
        self.t = None
        if self.known_points == self.reference_known_points:
            return

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

        self.heartbeat_listener.update_transformation(R, t)

        # Now publish the transformation matrix
        transform_msg = Float64MultiArray()
        transform_msg.data = np.concatenate((R.flatten(), t))
        self.transform_pub.publish(transform_msg)

    def transform_point(self, point, forward=True):
        """
        Transforms a point from the current map to the reference map or vice versa

        Parameters:
        - point (tuple): The point to be transformed.
        - forward (bool): True if transforming from current map to reference map, False otherwise.

        Returns:
        - tuple: The transformed point.
        """
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
        """
        Executes the main loop of the agent_entry_exit node.
        This method continuously checks for new agents, updates the heartbeat of existing agents,
        sends out heartbeat messages, checks for dead agents, and removes them from the agent list.

        Returns:
            None
        """

        prev_nearby_agents = []

        # Loop through at the rate we wish to publish location
        rate = rospy.Rate(LOCATION_FREQUENCY)
        last_time = int(time.time())
        while not rospy.is_shutdown():
            current_time = int(time.time())  # Get the current time

            # Collect received locations of nearby agents and publish them to a ROS topic
            agent_locations_array = AgentLocationsArray()
            agent_locations_array.header.stamp = rospy.Time.now()
            agent_locations_array.header.frame_id = 'map'
            agent_list = []
            agent_location_list = []
            for agent_id in self.location_listeners.keys():
                location = self.location_listeners[agent_id].get_location()

                if location is not None:

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

            # Publish map/map metadata periodically
            self.map_publisher.publish(self.map_msg)
            self.map_mod_publisher.publish(self.map_mod_msg)
            self.map_md_publisher.publish(self.map_md_msg)
                
            # Now publish heartbeat periodically
            if current_time - last_time >= HEARTBEAT_PERIOD:
                last_time = current_time
                
                # Check for new agents
                if self.entry_exit_listener.agent_update_available():
                    self.agents = self.entry_exit_listener.get_agents()
                current_agents_list = list(self.agents.keys())

                # Update agents with new heartbeats
                heartbeats = self.heartbeat_listener.get_heartbeats()

                update_to_active_agents = False
                for agent_id in heartbeats.keys():
                    if agent_id in current_agents_list:
                        self.agents[agent_id]['timestamp'] = heartbeats[agent_id]
                    else:
                        print(f'Detected heartbeat from unknown agent {agent_id}')
                        agent_hash = hash_func(str(agent_id))

                        # FIXME: Add correct agent_type and ip_address
                        self.agents[agent_id] = {
                            'agent_type': 'unknown',
                            'ip_address': 'unknown',
                            'hash': agent_hash,
                            'timestamp': heartbeats[agent_id]
                        }
                        update_to_active_agents = True

                # # Check for nearby agents
                # nearby_agents = set()
                # for agent_id, location in locations.items():
                #     if location is not None and agent_id in current_agents_list:
                #         x, y, theta = location
                #         self.agents[agent_id]['location'] = (x, y, theta)

                #         # Determine if the agent is close to the robot
                #         if self.my_location is not None:
                #             distance = ((x - self.my_location[0])**2 + (y - self.my_location[1])**2)**0.5
                #             if distance < DISTANCE_THRESHOLD:
                #                 nearby_agents.add(agent_id)
                #                 if 'agent_id' not in list(self.location_readers.keys()):
                #                     new_location_topic = Topic(self.participant, 'LocationTopic' + str(agent_id), Location)
                #                     self.location_listeners[agent_id] = LocationListener(self.my_id)
                #                     self.location_listeners[agent_id].update_transformation(self.R, self.t)
                #                     self.location_readers[agent_id] = DataReader(self.subscriber, new_location_topic, listener=self.location_listeners[agent_id], qos=best_effort_qos)
                
                #                     # new_data_topic = Topic(self.participant, 'DataTopic' + str(agent_id), DataMessage)
                #                     # self.agent_data_listeners[agent_id] = DataListener(self.my_id, agent_id)
                #                     # self.agent_data_readers[agent_id] = DataReader(self.subscriber, new_data_topic, listener=self.agent_data_listeners[agent_id], qos=reliable_qos)

                # # Only need to perform this housekeeping if the list of nearby agents has changed
                # if nearby_agents != prev_nearby_agents:
                #     prev_nearby_agents = nearby_agents
                   
                #     # Remove readers that are no longer needed
                #     agent_list = list(self.location_readers.keys())
                #     for agent_id in agent_list:
                #         if agent_id not in nearby_agents:
                #             self.location_listeners[agent_id] = None
                #             self.location_readers[agent_id] = None
                #             self.location_listeners.pop(agent_id)
                #             self.location_readers.pop(agent_id)

                #             # self.agent_data_listeners[agent_id] = None
                #             # self.agent_data_readers[agent_id] = None
                #             # self.agent_data_listeners.pop(agent_id)
                #             # self.agent_data_readers.pop(agent_id)

                # Check Periodically for Dead Agents
                dead_agents = []
                for agent_id, agent_info in self.agents.items():

                    # Skip self
                    if agent_id == int(self.my_id):
                        continue

                    time_difference = current_time - agent_info['timestamp']

                    if time_difference > HEARTBEAT_TIMEOUT:
                        # Agent has timed out
                        print(f'Agent {agent_id} has timed out')
                        dead_agents.append(agent_id)              
                
                # Remove Dead Agents
                for agent_id in dead_agents:
                    self.agents.pop(agent_id)

                # Update the entry/exit listener with the new agents
                if update_to_active_agents or dead_agents:
                    self.entry_exit_listener.update_agents(agents=self.agents)

                if len(self.agents) > 0:
                    agent_sub_list = AgentSubscription()
                    agent_sub_list.agentIDs.data = list(self.agents.keys())
                    agent_sub_list.header.stamp = rospy.Time.now()
                    agent_sub_list.header.frame_id = 'map'
                    self.agent_sub_pub.publish(agent_sub_list)

            rate.sleep()

    def shutdown(self):
        print('\nSending exit message...')
        # Write exit message
        exit_message = EntryExit(int(self.my_id), AGENT_TYPE, 'exit', self.my_ip, int(time.time()))
        self.enter_exit_writer.write(exit_message)


if __name__ == '__main__':

    entry_exit_obj = EntryExitCommunication()
    rospy.on_shutdown(entry_exit_obj.shutdown)
    entry_exit_obj.setup_and_run()
    