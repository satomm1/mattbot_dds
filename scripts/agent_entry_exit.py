import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData

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
    agents: sequence[str]
    map: str
    map_md: str

class EntryExitListener(Listener):
    def on_data_available(self, reader):
        for sample in reader.read():

            # Determine what type of message was received
            if sample.action == 'enter':
                print(f'Agent {sample.agent_id} entered the environment')
                agent_type = sample.agent_type
                capabilities = sample.capabilities
                message_types = sample.message_types
                ip_address = sample.ip_address
                new_robot_hash = hash_id(str(sample.agent_id))
                temp_agents[sample.agent_id] = {
                    'agent_type': agent_type,
                    'capabilities': capabilities,
                    'message_types': message_types,
                    'ip_address': ip_address,
                    'hash': new_robot_hash,
                    'timestamp': sample.timestamp
                }  
                if find_if_closest_robot(new_robot_hash):
                    my_dict = {
                        'id': int(my_id),
                        'agent_type': AGENT_TYPE,
                        'capabilities': AGENT_CAPABILITIES,
                        'message_types': AGENT_MESSAGE_TYPES,
                        'ip_address': my_ip,
                        'hash': my_hash,
                        'timestamp': int(time.time())
                    }
                    sending_agent = json.dumps(my_dict)
                    agents_message = json.dumps(agents)

                    # TODO Get map too

                    init_receiving_topic = Topic(participant, 'InitializationTopic' + str(sample.agent_id), Initialization)
                    init_receiving_write = DataWriter(publisher, init_receiving_topic)

                    init_message = Initialization(sending_agent, agents_message, '', '')
                    init_receiving_write.write(init_message)

            elif sample.action == 'initialized':

                # Agent Initialized, move to agents dictionary
                print(f'Agent {sample.agent_id} initialized in the environment')
                if sample.agent_id in temp_agents:
                    agents[sample.agent_id] = temp_agents.pop(sample.agent_id)
                else:
                    agent_type = sample.agent_type
                    capabilities = sample.capabilities
                    message_types = sample.message_types
                    ip_address = sample.ip_address
                    new_robot_hash = hash_id(str(sample.agent_id))
                    agents[sample.agent_id] = {
                        'agent_type': agent_type,
                        'capabilities': capabilities,
                        'message_types': message_types,
                        'ip_address': ip_address,
                        'hash': new_robot_hash,
                        'timestamp': sample.timestamp
                    }   

            elif sample.action == 'exit':

                # Agent Exited, remove from agents dictionary
                if sample.agent_id in agents:
                    print(f'Agent {sample.agent_id} exited the environment')
                    exited_agents[sample.agent_id] = agents.pop(sample.agent_id)

class HeartbeatListener(Listener):
    def on_data_available(self, reader):
        for sample in reader.read():
            if sample.agent_id in agents:
                agents[sample.agent_id]['timestamp'] = sample.timestamp
            else:
                print(f'Agent {sample.agent_id} is not in the environment')

class InitializationListener(Listener):
    def on_data_available(self, reader):
        for sample in reader.read():
            print(f'Initialization message received from agent {sample.agents[0]}')
            agent_dict = json.loads(sample.agents)

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
            map_msg = OccupancyGrid()
            map_msg.header.stamp = load_time
            map_msg.header.frame_id = 'map'
            map_msg.info.map_load_time = rospy.Time.now()
            map_msg.info.resolution = map_md_dict['resolution']
            map_msg.info.width = map_md_dict['width']
            map_msg.info.height = map_md_dict['height']
            map_msg.info.origin.position.x = map_md_dict['origin_x']
            map_msg.info.origin.position.y = map_md_dict['origin_y']
            map_msg.info.origin.position.z = map_md_dict['origin_z']
            map_msg.info.origin.orientation.x = map_md_dict['origin_orientation_x']
            map_msg.info.origin.orientation.y = map_md_dict['origin_orientation_y']
            map_msg.info.origin.orientation.z = map_md_dict['origin_orientation_z']
            map_msg.info.origin.orientation.w = map_md_dict['origin_orientation_w']
            map_msg.data = map_dict['map_data']

            # Creat map metadata message
            map_md_msg = MapMetaData()
            map_md_msg.map_load_time = load_time
            map_md_msg.resolution = map_md_dict['resolution']
            map_md_msg.width = map_md_dict['width']
            map_md_msg.height = map_md_dict['height']
            map_md_msg.origin.position.x = map_md_dict['origin_x']
            map_md_msg.origin.position.y = map_md_dict['origin_y']
            map_md_msg.origin.position.z = map_md_dict['origin_z']
            map_md_msg.origin.orientation.x = map_md_dict['origin_orientation_x']
            map_md_msg.origin.orientation.y = map_md_dict['origin_orientation_y']
            map_md_msg.origin.orientation.z = map_md_dict['origin_orientation_z']
            map_md_msg.origin.orientation.w = map_md_dict['origin_orientation_w']

            # Publish the map and map metadata
            map_publisher = rospy.Publisher('map', OccupancyGrid, queue_size=10)
            map_md_publisher = rospy.Publisher('map_metadata', MapMetaData, queue_size=10)

            map_publisher.publish(map_msg)
            map_md_publisher.publish(map_md_msg)

        amInitialized = True

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

    num_participants = 0
    for sample in built_in_reader.take_iter(timeout=duration(milliseconds=100)):
        num_participants += 1
    
    if num_participants == 1:
        print('I am the first agent to enter the environment')
        # TODO: Now do something...
    else:
        print('I am not the first agent to enter the environment')

        init_topic = Topic(participant, 'InitializationTopic' + my_id, Initialization)

        amInitialized = False
        entry_message = EntryExit(int(my_id), AGENT_TYPE, 'enter', AGENT_CAPABILITIES, AGENT_MESSAGE_TYPES, my_ip, int(time.time()))
        enter_exit_writer.write(entry_message)

        while not amInitialized:
            time.sleep(5)
            if not amInitialized:
                entry_message.timestamp = int(time.time())
                enter_exit_writer.write(entry_message)
       

    # Now that we are initialized, we create a Subscriber for Entry/Exit Messages to listen for other agents
    subscriber = Subscriber(participant)

    # Create a DataReader
    listener = EntryExitListener()
    reader = DataReader(subscriber, entry_exit_topic, listener=listener)

    while True:
        current_time = int(time.time())

        # Send out heartbeat
        heartbeat_message = Heartbeat(int(my_id), current_time)
        heartbeat_writer.write(heartbeat_message)

        # Check Periodically for Dead Agents
        dead_agents = []
        for agent_id, agent_info in agents.items():
            agent_timestamp = agent_info['timestamp']
            time_difference = current_time - agent_timestamp

            if time_difference > HEARTBEAT_TIMEOUT:
                print(f'Agent {agent_id} has not sent a heartbeat in too long')
                dead_agents.append(agent_id)  # Add to list of dead agents

        # Remove Dead Agents
        for agent_id in dead_agents:
            lost_agents[agent_id] = agents.pop(agent_id)

        time.sleep(HEARTBEAT_FREQUENCY)
