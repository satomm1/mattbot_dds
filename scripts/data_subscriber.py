import rospy
from rospy_message_converter import message_converter
from mattbot_image_detection.msg import DetectedObject, DetectedObjectArray
from geometry_msgs.msg import Pose
from mattbot_dds.msg import AgentPath, MapUpdate, MultiAgentPlannedPath, MultiAgentCollisionReport
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Int16MultiArray, Time as RosTimeMsg
from mattbot_image_detection.msg import FaceEncoding

import sys

from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.core import Listener

from database_utils import RobotDatabase

import time
import json
import numpy as np

from dds_utils import (
    MSG_DETECTED_OBJECT,
    MSG_FACE_ENCODING,
    MSG_GLOBAL_OBSERVE_START,
    MSG_MAP_UPDATE,
    MSG_PATH,
    MSG_MULTI_AGENT_PLANNED_PATH,
    MSG_MULTI_AGENT_COLLISION_REPORT,
    MSG_SENSOR_DETECTED_OBJECTS,
    MSG_STAR_ENCODER_STATE,
    MSG_STAR_GRU_OUT_EGO,
    DataMessage,
    ROS_TOPIC_AGENTS_TO_SUBSCRIBE,
    ROS_TOPIC_TRANSFORMATION_MATRIX,
    RobotIdError,
    TransformMixin,
    create_domain_participant,
    data_topic_name,
    dispose_participant,
    parse_transform_msg,
    reliable_qos,
    require_robot_id_int,
)

##################################################
# This script is used to subscribe to various DataTopics
# and process them.
##################################################


class DataListener(Listener, TransformMixin):
    def __init__(
        self,
        topic_id,
        object_publisher,
        object_sensor_publisher,
        path_publisher,
        multi_agent_planned_path_publisher,
        multi_agent_collision_report_publisher,
        map_update_publisher,
        face_encoding_publisher,
        global_observe_publisher=None,
        sqlite_db=None,
    ):
        super().__init__()
        self.init_transform_state()
        self.topic_id = topic_id
        self.object_publisher = object_publisher
        self.object_sensor_publisher = object_sensor_publisher
        self.path_publisher = path_publisher
        self.multi_agent_planned_path_publisher = multi_agent_planned_path_publisher
        self.multi_agent_collision_report_publisher = multi_agent_collision_report_publisher
        self.map_update_publisher = map_update_publisher
        self.face_encoding_publisher = face_encoding_publisher
        self.global_observe_publisher = global_observe_publisher

        self.db = sqlite_db

        self._pub_star_encoder = rospy.Publisher(
            "/team/dds/star_encoder_state/" + str(topic_id),
            Float32MultiArray,
            queue_size=2,
        )
        self._pub_star_gru = rospy.Publisher(
            "/team/dds/star_gru_out_ego/" + str(topic_id),
            Float32MultiArray,
            queue_size=2,
        )

    def on_data_available(self, reader):
        for sample in reader.read():

            sending_agent = sample.sending_agent

            # Check if the message is from the agent
            if sending_agent == self.topic_id:
                message_type = sample.message_type
                timestamp = sample.timestamp
                data = json.loads(sample.data)
                if message_type == MSG_DETECTED_OBJECT:
                    new_object = message_converter.convert_dictionary_to_ros_message(
                        "mattbot_image_detection/DetectedObject", data
                    )

                    # Now convert pose of detected object to my frame
                    x = new_object.pose.position.x
                    y = new_object.pose.position.y
                    new_point = self.transform_point([x, y, 0], forward=False)
                    new_object.pose.position.x = new_point[0]
                    new_object.pose.position.y = new_point[1]

                    self.object_publisher.publish(new_object)
                    print("Received object from agent " + str(self.topic_id))

                    if self.db is not None:
                        self.db.add_object(
                            new_object.class_name,
                            new_object.pose.position.x,
                            new_object.pose.position.y,
                            self.topic_id,
                            timestamp,
                        )
                elif message_type == MSG_SENSOR_DETECTED_OBJECTS:
                    x = data["x"]
                    y = data["y"]
                    w = data["w"]
                    class_name = data["class"]

                    object_array = DetectedObjectArray()
                    object_array.header.stamp = rospy.Time.now()
                    object_array.header.frame_id = "map"
                    object_array.sending_agent = sending_agent
                    object_array.objects = []
                    for i in range(len(x)):
                        new_point = self.transform_point([x[i], y[i], 0], forward=False)

                        detected_object = DetectedObject()
                        detected_object.class_name = class_name[i]
                        detected_object.probability = 1.0
                        detected_object_pose = Pose()
                        detected_object_pose.position.x = new_point[0]
                        detected_object_pose.position.y = new_point[1]
                        detected_object_pose.position.z = 0
                        detected_object_pose.orientation.w = 1
                        detected_object.pose = detected_object_pose
                        detected_object.width = w[i]
                        object_array.objects.append(detected_object)

                    # print(object_array)
                    # if len(object_array.objects) > 0:
                    self.object_sensor_publisher.publish(object_array)

                elif message_type == MSG_PATH:
                    new_path = message_converter.convert_dictionary_to_ros_message("nav_msgs/Path", data)

                    # Now convert poses of path to my frame
                    for i in range(len(new_path.poses)):
                        x = new_path.poses[i].pose.position.x
                        y = new_path.poses[i].pose.position.y
                        new_point = self.transform_point([x, y, 0], forward=False)
                        new_path.poses[i].pose.position.x = new_point[0]
                        new_path.poses[i].pose.position.y = new_point[1]

                    new_agent_path = AgentPath()
                    new_agent_path.agentID.data = self.topic_id
                    new_agent_path.path = new_path
                    self.path_publisher.publish(new_agent_path)
                    print("Received path from agent " + str(self.topic_id))

                elif message_type == MSG_MULTI_AGENT_PLANNED_PATH:
                    plan_id = data.get("plan_id", "")
                    path_dict = data.get("path")
                    if not isinstance(path_dict, dict):
                        rospy.logwarn(
                            "multi_agent_planned_path from agent %s: missing or invalid path dict",
                            self.topic_id,
                        )
                        continue
                    new_path = message_converter.convert_dictionary_to_ros_message("nav_msgs/Path", path_dict)
                    for i in range(len(new_path.poses)):
                        x = new_path.poses[i].pose.position.x
                        y = new_path.poses[i].pose.position.y
                        new_point = self.transform_point([x, y, 0], forward=False)
                        new_path.poses[i].pose.position.x = new_point[0]
                        new_path.poses[i].pose.position.y = new_point[1]
                    out = MultiAgentPlannedPath()
                    out.plan_id = plan_id
                    out.source_agent = int(sending_agent)
                    out.path = new_path
                    self.multi_agent_planned_path_publisher.publish(out)
                    rospy.logdebug(
                        "dds_data_subscriber: multi_agent_planned_path from agent %s plan_id=%s poses=%d",
                        self.topic_id,
                        plan_id,
                        len(new_path.poses),
                    )

                elif message_type == MSG_MULTI_AGENT_COLLISION_REPORT:
                    out = MultiAgentCollisionReport()
                    out.plan_id = str(data.get("plan_id", ""))
                    out.source_agent = int(sending_agent)
                    out.robot_i = int(data.get("robot_i", 0))
                    out.robot_j = int(data.get("robot_j", 0))
                    out.segment_i = [int(x) for x in (data.get("segment_i") or [])]
                    out.segment_j = [int(x) for x in (data.get("segment_j") or [])]
                    out.complete = bool(data.get("complete", False))
                    out.sent_stamp = float(data.get("sent_stamp", 0.0))
                    self.multi_agent_collision_report_publisher.publish(out)
                    rospy.logdebug(
                        "dds_data_subscriber: multi_agent_collision_report from agent %s plan_id=%s pair=(%s,%s)",
                        self.topic_id,
                        out.plan_id,
                        out.robot_i,
                        out.robot_j,
                    )

                elif message_type == MSG_MAP_UPDATE:
                    map_update = message_converter.convert_dictionary_to_ros_message("mattbot_dds/MapUpdate", data)

                    # Need to transform the map update to my frame
                    x = map_update.x
                    y = map_update.y
                    new_point = self.transform_point([x, y, 0], forward=False)

                    map_update.x = new_point[0]
                    map_update.y = new_point[1]

                    # Publish the map update
                    self.map_update_publisher.publish(map_update)

                elif message_type in (MSG_STAR_ENCODER_STATE, MSG_STAR_GRU_OUT_EGO):
                    vec = data.get("v")
                    if not isinstance(vec, list):
                        continue
                    out = Float32MultiArray()
                    out.data = [float(x) for x in vec]
                    if message_type == MSG_STAR_ENCODER_STATE:
                        self._pub_star_encoder.publish(out)
                    else:
                        self._pub_star_gru.publish(out)

                elif message_type == MSG_GLOBAL_OBSERVE_START:
                    if self.global_observe_publisher is None:
                        continue
                    sec, nsec = data.get("sec"), data.get("nsec")
                    if sec is None or nsec is None:
                        continue
                    tmsg = RosTimeMsg()
                    tmsg.data.secs = int(sec)
                    tmsg.data.nsecs = int(nsec)
                    self.global_observe_publisher.publish(tmsg)
                    rospy.loginfo(
                        "dds_data_subscriber: relayed global_observe_start from agent %s -> %s",
                        self.topic_id,
                        tmsg.data,
                    )

                elif message_type == MSG_FACE_ENCODING:
                    data = json.loads(sample.data)
                    encoding = data["encoding"]
                    name = data["name"]

                    face_encoding = FaceEncoding()
                    face_encoding.encoding = list(encoding)
                    face_encoding.name = name
                    face_encoding.external = True  # This face encoding came from an external agent

                    print("Received face encoding via DDS")

                    self.face_encoding_publisher.publish(face_encoding)
            else:
                # This was a message to the agent, we can safely ignore
                continue


class DataSubscriber(TransformMixin):
    def __init__(self):

        rospy.init_node("dds_data_subscriber", anonymous=True)

        self.init_transform_state()

        try:
            self.my_id_int = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)
        # Get sqlite parameter
        self.sqlite = rospy.get_param("~sqlite", False)
        self.db = None
        if self.sqlite:
            self.db = RobotDatabase()
            self.db.create_objects_table()

        self.participant = create_domain_participant(domain_qos=False)
        self.subscriber = Subscriber(self.participant)

        self.data_listeners = dict()
        self.data_readers = dict()

        rospy.Subscriber(ROS_TOPIC_TRANSFORMATION_MATRIX, Float64MultiArray, self.transformation_callback)

        self.object_publisher = rospy.Publisher("/object_from_agent", DetectedObject, queue_size=10)
        self.object_sensor_publisher = rospy.Publisher("/object_from_sensor", DetectedObjectArray, queue_size=10)
        self.path_publisher = rospy.Publisher("/path_from_agent", AgentPath, queue_size=10)
        self._multi_agent_planned_path_from_agent_topic = rospy.get_param(
            "~multi_agent_planned_path_from_agent_topic", "/multi_agent_planned_path_from_agent"
        ).strip() or "/multi_agent_planned_path_from_agent"
        self.multi_agent_planned_path_publisher = rospy.Publisher(
            self._multi_agent_planned_path_from_agent_topic, MultiAgentPlannedPath, queue_size=10
        )
        rospy.loginfo(
            "dds_data_subscriber: peer multi_agent_planned_path -> %s",
            self._multi_agent_planned_path_from_agent_topic,
        )
        self._multi_agent_collision_report_from_agent_topic = rospy.get_param(
            "~multi_agent_collision_report_from_agent_topic", "/multi_agent_collision_report_from_agent"
        ).strip() or "/multi_agent_collision_report_from_agent"
        self.multi_agent_collision_report_publisher = rospy.Publisher(
            self._multi_agent_collision_report_from_agent_topic, MultiAgentCollisionReport, queue_size=10
        )
        rospy.loginfo(
            "dds_data_subscriber: peer multi_agent_collision_report -> %s",
            self._multi_agent_collision_report_from_agent_topic,
        )
        self.map_update_publisher = rospy.Publisher("/map_update", MapUpdate, queue_size=10)
        self.face_encoding_publisher = rospy.Publisher("/new_face_encoding", FaceEncoding, queue_size=10)

        self._global_observe_ros_topic = rospy.get_param("~global_observe_start_ros_topic", "/global_observe_start").strip() or "/global_observe_start"
        self._relay_global_observe = bool(rospy.get_param("~relay_global_observe_start_from_dds", True))
        self.global_observe_publisher = None
        if self._relay_global_observe:
            self.global_observe_publisher = rospy.Publisher(
                self._global_observe_ros_topic,
                RosTimeMsg,
                queue_size=1,
                latch=True,
            )
            rospy.loginfo(
                "dds_data_subscriber: relaying %s from DDS to %s",
                MSG_GLOBAL_OBSERVE_START,
                self._global_observe_ros_topic,
            )

        self.subscribed_agents = set()
        self.agents_to_subscribe = set()
        self.agents_to_subscribe_subscriber = rospy.Subscriber(
            ROS_TOPIC_AGENTS_TO_SUBSCRIBE, Int16MultiArray, self.agents_to_subscribe_callback
        )

    def transformation_callback(self, data):
        self.R, self.t = parse_transform_msg(data)

        for agent_id in self.data_listeners:
            self.data_listeners[agent_id].update_transformation(self.R, self.t)

    def agents_to_subscribe_callback(self, data):
        # Get the list of agents to subscribe to
        agents_to_subscribe = data.data

        self.agents_to_subscribe = set(agents_to_subscribe)

    def run(self):
        while not rospy.is_shutdown():
            try:
                new_agents = self.agents_to_subscribe - self.subscribed_agents
                old_agents = self.subscribed_agents - self.agents_to_subscribe

                for agent_id in new_agents:
                    print(f"    Subscribed to agent {agent_id} data")
                    new_data_topic = Topic(self.participant, data_topic_name(agent_id), DataMessage)
                    self.data_listeners[agent_id] = DataListener(
                        agent_id,
                        self.object_publisher,
                        self.object_sensor_publisher,
                        self.path_publisher,
                        self.multi_agent_planned_path_publisher,
                        self.multi_agent_collision_report_publisher,
                        self.map_update_publisher,
                        self.face_encoding_publisher,
                        global_observe_publisher=self.global_observe_publisher,
                        sqlite_db=self.db,
                    )
                    self.data_listeners[agent_id].update_transformation(self.R, self.t)
                    self.data_readers[agent_id] = DataReader(
                        self.subscriber, new_data_topic, listener=self.data_listeners[agent_id], qos=reliable_qos
                    )

                for agent_id in old_agents:
                    print(f"    Unsubscribed from agent {agent_id} data")
                    self.data_readers[agent_id] = None
                    self.data_listeners[agent_id] = None
                    self.data_readers.pop(agent_id)
                    self.data_listeners.pop(agent_id)

                self.subscribed_agents = self.agents_to_subscribe

            except Exception as e:
                pass

            rospy.sleep(1)

    def shutdown(self):
        print("Shutting down DDS Data Subscriber")
        self.data_readers.clear()
        self.data_listeners.clear()
        self.subscriber = None
        dispose_participant(self.participant)
        self.participant = None


if __name__ == "__main__":

    data_subscriber = DataSubscriber()
    time.sleep(11)  # Wait
    rospy.on_shutdown(data_subscriber.shutdown)
    data_subscriber.run()
