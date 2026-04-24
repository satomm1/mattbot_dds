from dataclasses import dataclass
from cyclonedds.domain import DomainParticipant
from cyclonedds.topic import Topic
from cyclonedds.sub import DataReader
from cyclonedds.util import duration
from cyclonedds.idl import IdlStruct

@dataclass
class Message(IdlStruct):
    text: str

participant = DomainParticipant()
topic = Topic(participant, "Announcements", Message)
reader = DataReader(participant, topic)

# If we don't receive a single announcement for five minutes we want the script to exit.
for msg in reader.take_iter(timeout=duration(minutes=5)):
    print(msg.text)
print("Success")