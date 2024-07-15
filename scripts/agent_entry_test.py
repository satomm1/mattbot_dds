import cyclonedds
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.util import duration
from cyclonedds.idl import IdlStruct
from dataclasses import dataclass
from cyclonedds.idl.types import sequence

import time
import socket

# Define a data class for the Entry/Exit messages
@dataclass
class EntryExit(IdlStruct):
    agent_id: int
    agent_type: str  # i.e. robot, sensor, human
    action: str  # i.e. enter, exit
    capabilities: sequence[str]  # indicates sensing capabilities of agent, i.e. camera, lidar, etc.
    message_types: sequence[str]  # Indicates message topics this agent will publish
    ip_address: str  # IP address of the agent
    timestamp: int  # Timestamp of the message

my_ip = socket.gethostbyname(socket.gethostname())

# Create a DomainParticipant
participant = DomainParticipant()

# Create a Topic
topic = Topic(participant, 'EntryExitTopic', EntryExit)

# Create a Publisher
publisher = Publisher(participant)

# Create a DataWriter
writer = DataWriter(publisher, topic)

# Publish a sample message
for i in range(20):
    timestamp = int(time.time())
    message = EntryExit(i, 'robot', 'enter', ['camera', 'lidar'], ['object_detection', 'object_tracking'], my_ip, timestamp)
    writer.write(message)

    time.sleep(0.5)