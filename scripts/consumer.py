import cyclonedds
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.util import duration
from cyclonedds.idl import IdlStruct
from dataclasses import dataclass

import time

# Define a data class for the message
@dataclass
class HelloWorld(IdlStruct):
    message: str

# Create a DomainParticipant
participant = DomainParticipant()

# Create a Topic
topic = Topic(participant, 'HelloWorldTopic', HelloWorld)

# Create a Subscriber
subscriber = Subscriber(participant)

# Create a DataReader
reader = DataReader(subscriber, topic)

# Receive messages
while True:
    sample = reader.read()
    if sample:
        message = sample[0]
        print(f'Received: {message.message}')
    time.sleep(1)
