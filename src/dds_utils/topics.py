"""DDS topic name constants/builders and ROS topic name constants."""

# DDS topic names
ENTRY_EXIT_TOPIC = "EntryExitTopic"
INITIALIZATION_TOPIC = "InitializationTopic"
HEARTBEAT_TOPIC = "HeartbeatTopic"


def data_topic_name(agent_id) -> str:
    return "DataTopic" + str(agent_id)


def location_topic_name(agent_id) -> str:
    return "LocationTopic" + str(agent_id)


def image_topic_name(agent_id) -> str:
    return "ImageTopic" + str(agent_id)


# ROS topics — preserve prior per-script resolution (relative vs absolute) for behavior parity
ROS_TOPIC_TRANSFORMATION_MATRIX = "transformation_matrix"
ROS_TOPIC_TRANSFORMATION_MATRIX_ABS = "/transformation_matrix"

ROS_TOPIC_AGENTS_TO_SUBSCRIBE = "/agents_to_subscribe"
ROS_TOPIC_HEARTBEAT_AGENTS = "/heartbeat_agents"
ROS_TOPIC_ENTRY_AGENTS = "/entry_agents"
ROS_TOPIC_EXITED_AGENTS = "/exited_agents"

ROS_TOPIC_MAP = "map"
ROS_TOPIC_MAP_MOD = "map_mod"
ROS_TOPIC_MAP_METADATA = "map_metadata"
