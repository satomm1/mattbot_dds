import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rospy_message_converter import message_converter

import cyclonedds
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

import asyncio
import time
import os
import hashlib
import socket
import json
import requests


HEARTBEAT_FREQUENCY = 10
HEARTBEAT_TIMEOUT = 15
AGENT_CAPABILITIES = ['camera', 'lidar']
AGENT_MESSAGE_TYPES = ['object_detection', 'object_tracking']
AGENT_TYPE = 'robot'

# Define a data class for the Entry/Exit messages
@dataclass
class EntryExit(IdlStruct):
    agent_id: int
    agent_type: str  # i.e. robot, sensor, human
    action: str  # i.e. enter, initialized, exit
    capabilities: sequence[str]  # indicates sensing capabilities of agent, i.e. camera, lidar, etc.
    message_types: sequence[str]  # Indicates message topics this agent will publish
    ip_address: str  # IP address of the agent
    timestamp: int  # Timestamp of the message

# Define a data class for the Heartbeat messages
@dataclass
class Heartbeat(IdlStruct):
    agent_id: int
    timestamp: int

@dataclass 
class Initialization(IdlStruct):   
    sending_agent: str
    agents: str
    map: str
    map_md: str

class EntryExitListener(Listener):
    def __init__(self, participant, publisher, subscriber, my_id, my_ip, my_hash):
        super().__init__()
        self.participant = participant
        self.publisher = publisher
        self.subscriber = subscriber
        self.agents = dict()
        self.temp_agents = dict()
        self.exited_agents = dict()
        self.lost_agents = dict()
        self.my_id = my_id
        self.my_ip = my_ip
        self.my_hash = my_hash

        self.update_to_agents = False

    def on_data_available(self, reader):
        for sample in reader.read():
            print(sample)

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
                self.agents[sample.agent_id] = {
                    'agent_type': agent_type,
                    'capabilities': capabilities,
                    'message_types': message_types,
                    'ip_address': ip_address,
                    'hash': new_robot_hash,
                    'timestamp': sample.timestamp
                }  
                self.update_to_agents = True
                if find_if_closest_robot(new_robot_hash):
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

                    if len(agents) > 0:
                        agents_message = json.dumps(self.agents)
                    else:
                        agents_message = json.dumps("")

                    init_receiving_topic = Topic(self.participant, 'InitializationTopic' + str(sample.agent_id), Initialization)
                    init_receiving_write = DataWriter(self.publisher, init_receiving_topic)

                    map_dict = message_converter.convert_ros_message_to_dictionary(map_msg)
                    map_json = json.dumps(map_dict)
                    map_md_dict = message_converter.convert_ros_message_to_dictionary(map_md_msg)
                    map_md_json = json.dumps(map_md_dict)

                    init_message = Initialization(sending_agent, agents_message, map_json, map_md_json)
                    init_receiving_write.write(init_message)
                    print("Sent initialization message to new agent")

                    return

            # elif sample.action == 'initialized':

            #     # Agent Initialized, move to agents dictionary
            #     print(f'Agent {sample.agent_id} initialized in the environment')
            #     if sample.agent_id in self.temp_agents:
            #         self.agents[sample.agent_id] = self.temp_agents.pop(sample.agent_id)
            #         self.update_to_agents = True
            #     else:
            #         agent_type = sample.agent_type
            #         capabilities = sample.capabilities
            #         message_types = sample.message_types
            #         ip_address = sample.ip_address
            #         new_robot_hash = hash_id(str(sample.agent_id))
            #         self.agents[sample.agent_id] = {
            #             'agent_type': agent_type,
            #             'capabilities': capabilities,
            #             'message_types': message_types,
            #             'ip_address': ip_address,
            #             'hash': new_robot_hash,
            #             'timestamp': sample.timestamp
            #         }   
            #         self.update_to_agents = True

            elif sample.action == 'exit':

                # Agent Exited, remove from agents dictionary
                if sample.agent_id in self.agents:
                    print(f'Agent {sample.agent_id} exited the environment')
                    self.exited_agents[sample.agent_id] = self.agents.pop(sample.agent_id)
                    self.update_to_agents = True

    def agent_update_available(self):
        return self.update_to_agents
    
    def get_agents(self):
        self.update_to_agents = False
        return self.agents, self.temp_agents, self.exited_agents, self.lost_agents

class HeartbeatListener(Listener):

    def __init__(self, my_id):
        super().__init__()
        self.heartbeats = dict()
        self.my_id = my_id
        self.agents = dict()

    def on_data_available(self, heartbeat_reader):
        for sample in heartbeat_reader.read():

            if sample.agent_id == int(self.my_id):
                continue

            if sample.agent_id in self.agents:
                self.heartbeats[sample.agent_id] = sample.timestamp
            else:
                print(f'Agent {sample.agent_id} is not in the environment')

    def get_heartbeats(self):
        return self.heartbeats

    def update_agents(self, agents):
        self.agents = agents
        # Check for robot id in self.agents that isn't in self.heartbeats
        for agent_id in self.agents.keys():
            if agent_id not in self.heartbeats:
                self.heartbeats[agent_id] = self.agents[agent_id]['timestamp']

class InitializationListener(Listener):

    def __init__(self):
        super().__init__()
        self.map_received = False
        self.map_msg = OccupancyGrid()
        self.map_md_msg = MapMetaData()

    def on_data_available(self, init_reader):
        for sample in init_reader.read():

            sending_agent_dict = json.loads(sample.sending_agent)
            if sending_agent_dict['id'] == int(my_id):
                continue

            print(f'Initialization message received from agent {sending_agent_dict["id"]}')

            agents[sending_agent_dict['id']] = {
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
                    agent_type = agent_info['agent_type']
                    capabilities = agent_info['capabilities']
                    message_types = agent_info['message_types']
                    ip_address = agent_info['ip_address']
                    agent_hash = agent_info['hash']
                    agents[agent_id] = {
                        'agent_type': agent_type,
                        'capabilities': capabilities,
                        'message_types': message_types,
                        'ip_address': ip_address,
                        'hash': agent_hash,
                        'timestamp': sample.timestamp
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
            map_publisher = rospy.Publisher('map', OccupancyGrid, queue_size=10)
            map_md_publisher = rospy.Publisher('map_metadata', MapMetaData, queue_size=10)

            map_publisher.publish(map_msg)
            map_md_publisher.publish(map_md_msg)

            self.map_received = True

            print("Map received through initialization message")

    def map_available(self):
        return self.map_received

    def get_map(self):
        return self.map_msg, self.map_md_msg

def hash_id(robot_id):
    return int(hashlib.sha256(robot_id.encode()).hexdigest(), 16) 

def find_if_closest_robot(robot_hash):
    num_agents = len(agents)+1
    my_distance = abs(my_hash/num_agents - robot_hash/num_agents)

    for agent_id, agent_info in agents.items():
        agent_hash = agent_info['hash']
        
        distance = abs(agent_hash/num_agents - robot_hash/num_agents)
        if distance < my_distance:
            return False
    
    print('I will provide initial details to the new agent')
    return True

def shutdown():
    print('Shutting down...')
    exit_message = EntryExit(int(my_id), AGENT_TYPE, 'exit', [], [], my_ip, int(time.time()))
    enter_exit_writer.write(exit_message)
    rospy.signal_shutdown('Shutting down...')

map_msg = OccupancyGrid()
map_md_msg = MapMetaData()

if __name__ == '__main__':

    rospy.init_node('agent_entry_exit', anonymous=True)
    rospy.on_shutdown(shutdown)

    # Get my ID and Hash and IP Address
    my_id = os.environ.get('ROBOT_ID')
    my_hash = hash_id(my_id)
    
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # This doesn't have to be reachable; it just has to be a valid address
    s.connect(("8.8.8.8", 80))
    my_ip = s.getsockname()[0]
    s.close()
    print(f"My IP address is {my_ip}")

    # Dictionary to store agents in the environment
    agents = dict()
    temp_agents = dict()
    exited_agents = dict()
    lost_agents = dict()

    # Create a DomainParticipant
    participant = DomainParticipant()

    # Create a Subscriber
    subscriber = Subscriber(participant)
    
    qos = Qos(
        Policy.Reliability.BestEffort,
        # Policy.Deadline(duration(microseconds=10)),
        # Policy.Durability.TransientLocal,
        # Policy.History.KeepLast(10)
    )

    # Create The Topic
    entry_exit_topic = Topic(participant, 'EntryExitTopic', EntryExit)
    heartbeat_topic = Topic(participant, 'HeartbeatTopic', Heartbeat)

    publisher = Publisher(participant)
    enter_exit_writer = DataWriter(publisher, entry_exit_topic)
    heartbeat_writer = DataWriter(publisher, heartbeat_topic)

    # Builtin Topic to check if I am the first agent to enter the environment
    built_in_reader = BuiltinDataReader(participant, BuiltinTopicDcpsParticipant)

    # Create a DataReader
    entry_exit_listener = EntryExitListener(participant, publisher, subscriber, my_id, my_ip, my_hash)
    reader = DataReader(subscriber, entry_exit_topic, listener=entry_exit_listener)

    heartbeat_listener = HeartbeatListener(my_id)
    heartbeat_reader = DataReader(subscriber, heartbeat_topic, listener=heartbeat_listener)

    num_participants = 0
    for sample in built_in_reader.take_iter(timeout=duration(milliseconds=100)):
        num_participants += 1
    
    # map_msg = OccupancyGrid()
    # map_md_msg = MapMetaData()
    if num_participants == 1:
        print('I am the first agent to enter the environment')

        server_url='http://192.168.50.2:8000/graphql'
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
                response = requests.post(server_url, json={'query': map_query})
                if response.status_code == 200:
                    data = response.json()
                    map_data = data.get('data', {}).get('map', {})
                
                    have_map = True

                    # Convert the strings into the ROS Occupancy grid
                    map_msg.header.frame_id = 'map'
                    map_msg.info.width = map_data.get('width')
                    map_msg.info.height = map_data.get('height')
                    map_msg.info.resolution = map_data.get('resolution')
                    map_msg.info.origin.position.x = map_data.get('origin_x')
                    map_msg.info.origin.position.y = map_data.get('origin_y')
                    map_msg.info.origin.position.z = map_data.get('origin_z')
                    map_msg.info.origin.orientation.x = map_data.get('origin_orientation_x')
                    map_msg.info.origin.orientation.y = map_data.get('origin_orientation_y')
                    map_msg.info.origin.orientation.z = map_data.get('origin_orientation_z')
                    map_msg.info.origin.orientation.w = map_data.get('origin_orientation_w')
                    map_msg.data = map_data.get('occupancy')

                    map_md_msg.map_load_time = rospy.Time.now()
                    map_md_msg.resolution = map_data.get('resolution')
                    map_md_msg.width = map_data.get('width')
                    map_md_msg.height = map_data.get('height')
                    map_md_msg.origin.position.x = map_data.get('origin_x')
                    map_md_msg.origin.position.y = map_data.get('origin_y')
                    map_md_msg.origin.position.z = map_data.get('origin_z')
                    map_md_msg.origin.orientation.x = map_data.get('origin_orientation_x')
                    map_md_msg.origin.orientation.y = map_data.get('origin_orientation_y')
                    map_md_msg.origin.orientation.z = map_data.get('origin_orientation_z')
                    map_md_msg.origin.orientation.w = map_data.get('origin_orientation_w')
                    
                    # Publish the map and map metadata
                    map_publisher = rospy.Publisher('map', OccupancyGrid, queue_size=10)
                    map_md_publisher = rospy.Publisher('map_metadata', MapMetaData, queue_size=10)

                    map_publisher.publish(map_msg)
                    map_md_publisher.publish(map_md_msg)

                    print("Map retrieved")

                else:
                    print(f"Error retrieving map: {response.status_code}")
            except Exception as e:
                print(f"Error retrieving map: {e}")
            time.sleep(1)

    else:
        print('I am not the first agent to enter the environment')

        init_topic = Topic(participant, 'InitializationTopic' + my_id, Initialization)
        init_listener = InitializationListener()
        init_reader = DataReader(subscriber, init_topic, listener=init_listener)

        amInitialized = False
        entry_message = EntryExit(int(my_id), AGENT_TYPE, 'enter', AGENT_CAPABILITIES, AGENT_MESSAGE_TYPES, my_ip, int(time.time()))
        enter_exit_writer.write(entry_message)

        while not init_listener.map_available():
            print("No Map yet...")
            time.sleep(1)
            if not init_listener.map_available():
                entry_message.timestamp = int(time.time())
                enter_exit_writer.write(entry_message)

        map_msg, map_md_msg = init_listener.get_map()

        # Send entry exit topic message that i am initialized
        # init_finished_message = EntryExit(int(my_id), AGENT_TYPE, 'initialized', AGENT_CAPABILITIES, AGENT_MESSAGE_TYPES, my_ip, int(time.time()))
        # enter_exit_writer.write(init_finished_message)
        print("Made it here")
    

    while True:
        current_time = int(time.time())

        # Check for new agents
        if entry_exit_listener.agent_update_available():
            agents, temp_agents, exited_agents, lost_agents = entry_exit_listener.get_agents()

        print(agents)
        heartbeat_listener.update_agents(agents)

        # Send out heartbeat
        heartbeat_message = Heartbeat(int(my_id), current_time)
        heartbeat_writer.write(heartbeat_message)

        heartbeats = heartbeat_listener.get_heartbeats()

        # Check Periodically for Dead Agents
        dead_agents = []
        for agent_id, agent_info in agents.items():
            time_difference = current_time - heartbeats[agent_id]
            agent_timestamp = heartbeats[agent_id]

            if time_difference > HEARTBEAT_TIMEOUT:
                print(f'Agent {agent_id} has not sent a heartbeat in too long')
                dead_agents.append(agent_id)  # Add to list of dead agents

        # Remove Dead Agents
        for agent_id in dead_agents:
            lost_agents[agent_id] = agents.pop(agent_id)

        time.sleep(HEARTBEAT_FREQUENCY)
