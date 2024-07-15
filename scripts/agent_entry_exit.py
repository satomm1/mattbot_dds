import rospy

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

HEARTBEAT_FREQUENCY = 10
HEARTBEAT_TIMEOUT = 15

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
                    # TODO Perform Initialization
                    pass

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
    exit_message = EntryExit(int(my_id), 'robot', 'exit', [], [], my_ip, int(time.time()))
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
        entry_message = EntryExit(int(my_id), 'robot', 'enter', ['camera', 'lidar'], ['object_detection', 'object_tracking'], my_ip, int(time.time()))
        enter_exit_writer.write(entry_message)
        # TODO: continue with initialization

    # Create a Subscriber for Entry/Exit Messages
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
