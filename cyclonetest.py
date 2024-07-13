from dataclasses import dataclass
from cyclonedds.idl import IdlStruct
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.pub import DataWriter

@dataclass
class Message(IdlStruct):
    text: str

name = input("What is your name? ")
message = Message(text=f"{name} has started his first DDS Python application!")

participant = DomainParticipant()
topic = Topic(participant, "Announcements", Message)
writer = DataWriter(participant, topic)

writer.write(message)
print("Success")