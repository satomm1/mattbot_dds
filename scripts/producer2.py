import cyclonedds
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.pub import Publisher, DataWriter
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

# Create a Publisher
publisher = Publisher(participant)

# Create a DataWriter
writer = DataWriter(publisher, topic)

# Publish messages
for i in range(10):
    message = HelloWorld(f'Goodbye, Galaxy {i}')
    writer.write(message)
    print(f'Sent: {message.message}')
    time.sleep(1)
