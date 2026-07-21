import json
import time
from dataclasses import dataclass

from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import sequence


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


@dataclass
class Location(IdlStruct):
    """
    Represents the location of an agent.

    Attributes:
        agent_id (int): The ID of the agent.
        timestamp (float): Unix epoch seconds (fractional) when the pose was captured.
        x (float): The x-coordinate of the agent.
        y (float): The y-coordinate of the agent.
        theta (float): The orientation of the agent.
        static (bool): Indicates if the agent is currently moving towards a goal or is static
    """

    agent_id: int
    timestamp: float
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
        timestamp (float): Unix epoch seconds (fractional) when the frame was captured.
        data (bytes): The image data in bytes.
        width (int): The width of the image.
        height (int): The height of the image.
        encoding (str): The encoding format of the image.
    """

    agent_id: int
    timestamp: float
    data: sequence[int]
    width: int
    height: int
    encoding: str


# std_msgs/Time (wall ROS time) bridged fleet-wide; JSON {"sec": int, "nsec": int}.
MSG_GLOBAL_OBSERVE_START = "global_observe_start"

# Directed goals for fleet coordination
MSG_MULTI_ROBOT_GOAL = "multi_robot_goal"
# JSON: {"plan_id": str, "path": dict} — path is nav_msgs/Path via message_converter
MSG_MULTI_AGENT_PLANNED_PATH = "multi_agent_planned_path"
# JSON: {"plan_id": str, "sec": int, "nsec": int, "fleet_robot_ids": [int, ...]}
MSG_MULTI_AGENT_EXECUTE_AT = "multi_agent_execute_at"
# JSON: plan_id, source_agent, fleet_robot_ids, waypoint_counts, waypoint_times_flat
MSG_MULTI_AGENT_TIMING_SOLVE = "multi_agent_timing_solve"
# JSON: robot_id, plan_id, active, sec/nsec execute_at, path dict, waypoint_times, dds_forward_robot_ids
MSG_MULTI_AGENT_ACTIVE_TRAJECTORY = "multi_agent_active_trajectory"
# JSON: plan_id, source_agent, robot_i, robot_j, segment_i, segment_j, complete
MSG_MULTI_AGENT_COLLISION_REPORT = "multi_agent_collision_report"

MSG_DETECTED_OBJECT = "detected_object"
MSG_LLM_DETECTED_OBJECT = "llm_detected_object"
MSG_PERSON_DETECTED = "person_detected"
MSG_SENSOR_DETECTED_OBJECTS = "sensor_detected_objects"
MSG_PATH = "path"
MSG_GOAL = "goal"
MSG_STOP = "stop"
# JSON optional: {"reason": str, ...} — full roslaunch teardown when handled by own_data_subscriber (required node).
MSG_ROBOT_SHUTDOWN = "robot_shutdown"
MSG_INVALID_GOAL = "invalid_goal"
MSG_FACE_ENCODING = "face_encoding"
MSG_MAP_UPDATE = "map_update"
MSG_STAR_ENCODER_STATE = "star_encoder_state"
MSG_STAR_GRU_OUT_EGO = "star_gru_out_ego"
MSG_POSITION_INIT = "position_init"
MSG_SEND_UNKNOWN_IMAGES = "send_unknown_images"
MSG_AIR_QUALITY = "air_quality"


def make_data_message(message_type: str, sending_agent: int, payload: dict) -> DataMessage:
    return DataMessage(
        message_type=message_type,
        sending_agent=sending_agent,
        timestamp=int(time.time()),
        data=json.dumps(payload),
    )
