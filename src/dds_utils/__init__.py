from cyclonedds.util import duration
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import sequence
from cyclonedds.core import Qos, Policy

from dataclasses import dataclass

@dataclass
class Heartbeat(IdlStruct):
    """
    Represents a heartbeat message from an agent.

    Attributes:
        agent_id (int): The ID of the agent sending the heartbeat.
        timestamp (int): The timestamp of the heartbeat message.
        agent_type (str): The type of the agent sending the heartbeat.
        location_valid (bool): Indicates if the agent's location is valid.
        x (float): The x-coordinate of the agent's location.
        y (float): The y-coordinate of the agent's location.
        theta (float): The orientation of the agent.
        topics (sequence[str]): A sequence of topics the agent is publishing to
    """
    agent_id: int
    timestamp: int
    agent_type: str
    ip_address: str
    location_valid: bool
    x: float
    y: float
    theta: float
    topics: sequence[str]

@dataclass
class EntryExit(IdlStruct):
    agent_id: int
    agent_type: str
    action: str
    ip_address: str
    timestamp: int

@dataclass
class Initialization(IdlStruct):
    """
    Represents the initialization parameters for the agent entry/exit system.

    Attributes:
        target_agent (int): The ID of the target agent.
        agents (str): A json dict of all the agents that the sending_agent is aware of.
        known_points (str): 
    """
    target_agent: int
    sending_agent: int
    agents: str
    known_points: str

@dataclass
class DataMessage(IdlStruct):
    message_type: str
    sending_agent: int
    timestamp: int
    data: str


# std_msgs/Time (wall ROS time) bridged fleet-wide; JSON {"sec": int, "nsec": int}.
# ROS trigger topic defaults to /global_observe_start_dds (not /global_observe_start) to avoid relay echo.
MSG_GLOBAL_OBSERVE_START = "global_observe_start"

@dataclass
class Location(IdlStruct):
    """
    Represents the location of an agent.

    Attributes:
        agent_id (int): The ID of the agent.
        timestamp (int): The timestamp of the location message.
        x (float): The x-coordinate of the agent.
        y (float): The y-coordinate of the agent.
        theta (float): The orientation of the agent.
        static (bool): Indicates if the agent is currently moving towards a goal or is static
    """
    agent_id: int
    timestamp: int
    x: float
    y: float
    theta: float
    static: bool

@dataclass
class ImageMessage(IdlStruct):
    """
    Represents an image message.

    Attributes:
        agent_id (int): The ID of the agent sending the image.
        timestamp (int): The timestamp of the image message.
        data (bytes): The image data in bytes.
        width (int): The width of the image.
        height (int): The height of the image.
        encoding (str): The encoding format of the image.
    """
    agent_id: int
    timestamp: int
    data: sequence[int]
    width: int
    height: int
    encoding: str


# Create different policies for the DDS entities
reliable_qos = Qos(
    Policy.Reliability.Reliable(max_blocking_time=duration(milliseconds=10)),
    Policy.Durability.TransientLocal,
    Policy.History.KeepLast(depth=1)
)

best_effort_qos = Qos(
    Policy.Reliability.BestEffort,
    Policy.Durability.Volatile,
    Policy.Liveliness.ManualByParticipant(lease_duration=duration(milliseconds=30000))
)