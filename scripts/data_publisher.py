import rospy
from rospy_message_converter import message_converter
from mattbot_image_detection.msg import DetectedObject, DetectedObjectArray
from mattbot_image_detection.msg import LabeledObject, LabeledObjectArray
from mattbot_dds.msg import (
    MultiRobotGoalPlan,
    MultiRobotExternalGoal,
    MultiAgentPlannedPath,
    MultiAgentExecuteAt,
    MultiAgentTimingSolve,
)
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Float32MultiArray, Float64MultiArray, Time as RosTimeMsg
from mattbot_image_detection.msg import FaceEncoding

from cyclonedds.topic import Topic
from cyclonedds.pub import Publisher, DataWriter

from database_utils import RobotDatabase

import sys
import time
import json
import copy
import numpy as np

from dds_utils import (
    INTER_DDS_WRITE_SLEEP_S,
    MSG_DETECTED_OBJECT,
    MSG_FACE_ENCODING,
    MSG_GLOBAL_OBSERVE_START,
    MSG_GOAL,
    MSG_INVALID_GOAL,
    MSG_LLM_DETECTED_OBJECT,
    MSG_MULTI_ROBOT_GOAL,
    MSG_MULTI_AGENT_PLANNED_PATH,
    MSG_MULTI_AGENT_EXECUTE_AT,
    MSG_MULTI_AGENT_TIMING_SOLVE,
    MSG_PATH,
    MSG_PERSON_DETECTED,
    MSG_STAR_ENCODER_STATE,
    MSG_STAR_GRU_OUT_EGO,
    DataMessage,
    ROS_TOPIC_TRANSFORMATION_MATRIX,
    RobotIdError,
    TransformMixin,
    create_domain_participant,
    data_topic_name,
    dispose_participant,
    make_data_message,
    reliable_qos,
    require_robot_id_int,
)


class DataPublisher(TransformMixin):
    def __init__(self):
        rospy.init_node("dds_data_publisher", anonymous=True)

        self.init_transform_state()

        try:
            self.my_id_int = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)
        self.my_id = str(self.my_id_int)

        # Get sqlite parameter
        self.sqlite = rospy.get_param("~sqlite", False)
        self.db = None
        if self.sqlite:
            self.db = RobotDatabase()

        self.participant = create_domain_participant(domain_qos=True)
        self.publisher = Publisher(self.participant)

        # Create my data topic
        self.data_topic = Topic(self.participant, data_topic_name(self.my_id_int), DataMessage)
        self.data_writer = DataWriter(self.publisher, self.data_topic, qos=reliable_qos)

        rospy.Subscriber(ROS_TOPIC_TRANSFORMATION_MATRIX, Float64MultiArray, self.transformation_callback)

        self.object_subscriber = rospy.Subscriber("/confirmed_objects", DetectedObject, self.confirmed_object_callback, queue_size=10)
        self.path_subscriber = rospy.Subscriber("/cmd_smoothed_path", Path, self.path_callback, queue_size=10)
        self.voice_goal_subscriber = rospy.Subscriber("/voice_goal", Pose2D, self.voice_goal_callback, queue_size=10)
        self.new_face_encoding_subscriber = rospy.Subscriber("/new_face_encoding", FaceEncoding, self.face_encoding_callback, queue_size=10)
        self.llm_image_subscriber = rospy.Subscriber("/labeled_unknown_objects", LabeledObjectArray, self.labeled_callback, queue_size=3)
        self.invalid_goal_subscriber = rospy.Subscriber("/invalid_goal", Pose2D, self.invalid_goal_callback, queue_size=10)
        self.detected_object_subscriber = rospy.Subscriber("/detected_objects", DetectedObjectArray, self.detected_object_callback, queue_size=10)

        self._star_enc_topic = rospy.get_param("~star_encoder_ros_topic", "").strip()
        self._star_gru_topic = rospy.get_param("~star_gru_out_ego_ros_topic", "").strip()
        self._sub_star_enc = None
        self._sub_star_gru = None
        if self._star_enc_topic:
            self._sub_star_enc = rospy.Subscriber(
                self._star_enc_topic,
                Float32MultiArray,
                self._star_encoder_callback,
                queue_size=2,
            )
        if self._star_gru_topic:
            self._sub_star_gru = rospy.Subscriber(
                self._star_gru_topic,
                Float32MultiArray,
                self._star_gru_out_ego_callback,
                queue_size=2,
            )

        # Subscribe only to the DDS trigger topic (not /global_observe_start) so peers can
        # relay to /global_observe_start without echoing back onto DDS.
        self._forward_global_observe = bool(rospy.get_param("~forward_global_observe_start_via_dds", True))
        self._global_observe_dds_trigger_topic = rospy.get_param(
            "~global_observe_start_dds_trigger_topic", "/global_observe_start_dds"
        ).strip() or "/global_observe_start_dds"
        self._sub_global_observe = None
        if self._forward_global_observe:
            self._sub_global_observe = rospy.Subscriber(
                self._global_observe_dds_trigger_topic,
                RosTimeMsg,
                self._global_observe_start_callback,
                queue_size=2,
            )
            rospy.loginfo(
                "dds_data_publisher: forwarding %s to DDS as %s (ROBOT_ID=%s)",
                self._global_observe_dds_trigger_topic,
                MSG_GLOBAL_OBSERVE_START,
                self.my_id,
            )

        self._target_data_writers = {}
        self._external_goal_multi_topic = rospy.get_param(
            "~external_goal_multi_ros_topic", "/external_goal_multi"
        ).strip() or "/external_goal_multi"
        self._pub_external_goal_multi_local = rospy.Publisher(
            self._external_goal_multi_topic, MultiRobotExternalGoal, queue_size=10, latch=False
        )
        self._multi_robot_goal_plan_topic = rospy.get_param(
            "~multi_robot_goal_plan_topic", "/multi_robot_goal_plan"
        ).strip() or "/multi_robot_goal_plan"
        self._sub_multi_robot_plan = rospy.Subscriber(
            self._multi_robot_goal_plan_topic,
            MultiRobotGoalPlan,
            self.multi_robot_goal_plan_callback,
            queue_size=2,
        )
        rospy.loginfo(
            "dds_data_publisher: multi-robot plans on %s -> DDS %s (local self -> %s)",
            self._multi_robot_goal_plan_topic,
            MSG_MULTI_ROBOT_GOAL,
            self._external_goal_multi_topic,
        )

        self._multi_agent_planned_path_topic = rospy.get_param(
            "~multi_agent_planned_path_for_dds_topic", "/multi_agent_planned_path_for_dds"
        ).strip() or "/multi_agent_planned_path_for_dds"
        self._sub_multi_agent_planned_path = rospy.Subscriber(
            self._multi_agent_planned_path_topic,
            MultiAgentPlannedPath,
            self.multi_agent_planned_path_callback,
            queue_size=2,
        )
        rospy.loginfo(
            "dds_data_publisher: multi-agent planned paths on %s -> DDS %s",
            self._multi_agent_planned_path_topic,
            MSG_MULTI_AGENT_PLANNED_PATH,
        )

        self._forward_multi_agent_execute_at = bool(
            rospy.get_param("~forward_multi_agent_execute_at_via_dds", True)
        )
        self._multi_agent_execute_at_dds_trigger_topic = rospy.get_param(
            "~multi_agent_execute_at_dds_trigger_topic", "/multi_agent_execute_at_dds"
        ).strip() or "/multi_agent_execute_at_dds"
        self._multi_agent_execute_at_ros_topic = rospy.get_param(
            "~multi_agent_execute_at_ros_topic", "/multi_agent_execute_at"
        ).strip() or "/multi_agent_execute_at"
        self._pub_execute_at_local = rospy.Publisher(
            self._multi_agent_execute_at_ros_topic, MultiAgentExecuteAt, queue_size=2, latch=True
        )
        self._sub_multi_agent_execute_at = None
        if self._forward_multi_agent_execute_at:
            self._sub_multi_agent_execute_at = rospy.Subscriber(
                self._multi_agent_execute_at_dds_trigger_topic,
                MultiAgentExecuteAt,
                self.multi_agent_execute_at_callback,
                queue_size=2,
            )
            rospy.loginfo(
                "dds_data_publisher: forwarding %s to DDS %s (local echo -> %s)",
                self._multi_agent_execute_at_dds_trigger_topic,
                MSG_MULTI_AGENT_EXECUTE_AT,
                self._multi_agent_execute_at_ros_topic,
            )

        self._forward_multi_agent_timing_solve = bool(
            rospy.get_param("~forward_multi_agent_timing_solve_via_dds", True)
        )
        self._multi_agent_timing_solve_for_dds_topic = rospy.get_param(
            "~multi_agent_timing_solve_for_dds_topic", "/multi_agent_timing_solve_for_dds"
        ).strip() or "/multi_agent_timing_solve_for_dds"
        self._multi_agent_timing_solve_ros_topic = rospy.get_param(
            "~multi_agent_timing_solve_ros_topic", "/multi_agent_timing_solve"
        ).strip() or "/multi_agent_timing_solve"
        self._pub_timing_solve_local = rospy.Publisher(
            self._multi_agent_timing_solve_ros_topic, MultiAgentTimingSolve, queue_size=2, latch=False
        )
        self._sub_multi_agent_timing_solve = None
        if self._forward_multi_agent_timing_solve:
            self._sub_multi_agent_timing_solve = rospy.Subscriber(
                self._multi_agent_timing_solve_for_dds_topic,
                MultiAgentTimingSolve,
                self.multi_agent_timing_solve_callback,
                queue_size=2,
            )
            rospy.loginfo(
                "dds_data_publisher: forwarding %s to DDS %s (local echo -> %s)",
                self._multi_agent_timing_solve_for_dds_topic,
                MSG_MULTI_AGENT_TIMING_SOLVE,
                self._multi_agent_timing_solve_ros_topic,
            )

    def _sender_id_optional(self):
        """Sending agent id for DDS messages that previously used 0 when ROBOT_ID was unset."""
        return self.my_id_int

    def _sender_id_strict(self):
        """Sending agent id for object/goal paths (always valid after ``require_robot_id_int``)."""
        return self.my_id_int

    def _publish_data(self, message_type, payload, *, sleep=True, sending_agent=None):
        aid = self._sender_id_strict() if sending_agent is None else sending_agent
        self.data_writer.write(make_data_message(message_type, aid, payload))
        if sleep:
            time.sleep(INTER_DDS_WRITE_SLEEP_S)

    def _get_writer_for_target(self, agent_id):
        """Reliable DataWriter on DataTopic{agent_id}, cached per target."""
        if agent_id in self._target_data_writers:
            return self._target_data_writers[agent_id]
        topic = Topic(self.participant, data_topic_name(agent_id), DataMessage)
        writer = DataWriter(self.publisher, topic, qos=reliable_qos)
        self._target_data_writers[agent_id] = writer
        return writer

    def multi_robot_goal_plan_callback(self, msg):
        """Fan out fleet goals to each peer's DataTopic; orchestrator's own id uses local ROS only."""
        my_id_int = self.my_id_int
        aid_sender = self._sender_id_optional()
        fleet_ids = [int(e.robot_id) for e in msg.goals]

        for entry in msg.goals:
            rid = int(entry.robot_id)
            x = entry.goal.x
            y = entry.goal.y
            th = entry.goal.theta
            new_point = self.transform_point([x, y, th])
            plan_id = msg.plan_id
            coordinated = msg.coordinated

            if rid == my_id_int:
                out = MultiRobotExternalGoal()
                out.goal.x = float(new_point[0])
                out.goal.y = float(new_point[1])
                out.goal.theta = float(new_point[2])
                out.plan_id = plan_id
                out.coordinated = coordinated
                out.source_agent = aid_sender
                out.target_agent = rid
                out.fleet_robot_ids = fleet_ids
                self._pub_external_goal_multi_local.publish(out)
                rospy.loginfo(
                    "dds_data_publisher: local multi-robot goal (self): plan_id=%s robot=%s",
                    plan_id,
                    rid,
                )
                continue

            payload = {
                "x": float(new_point[0]),
                "y": float(new_point[1]),
                "theta": float(new_point[2]),
                "plan_id": plan_id,
                "coordinated": bool(coordinated),
                "target_agent": rid,
                "fleet_robot_ids": fleet_ids,
            }
            dm = make_data_message(MSG_MULTI_ROBOT_GOAL, aid_sender, payload)
            writer = self._get_writer_for_target(rid)
            writer.write(dm)
            rospy.loginfo(
                "dds_data_publisher: sent %s to DataTopic%s plan_id=%s",
                MSG_MULTI_ROBOT_GOAL,
                rid,
                plan_id,
            )
            time.sleep(INTER_DDS_WRITE_SLEEP_S)

    def multi_agent_timing_solve_callback(self, msg):
        """Fan out MILP timing result to each fleet robot's DataTopic; echo locally for coordinator."""
        aid = self._sender_id_optional()
        fleet_ids = [int(x) for x in (msg.fleet_robot_ids or [])]
        if not fleet_ids:
            rospy.logwarn("dds_data_publisher: multi_agent_timing_solve missing fleet_robot_ids; skipping DDS")
            return
        payload = {
            "plan_id": str(msg.plan_id),
            "source_agent": int(msg.source_agent),
            "fleet_robot_ids": fleet_ids,
            "waypoint_counts": [int(x) for x in (msg.waypoint_counts or [])],
            "waypoint_times_flat": [float(x) for x in (msg.waypoint_times_flat or [])],
        }
        my_id_int = self.my_id_int
        for rid in fleet_ids:
            if rid == my_id_int:
                self._pub_timing_solve_local.publish(msg)
                rospy.loginfo(
                    "dds_data_publisher: local multi_agent_timing_solve plan_id=%s source=%s",
                    msg.plan_id,
                    msg.source_agent,
                )
                continue
            dm = make_data_message(MSG_MULTI_AGENT_TIMING_SOLVE, aid, payload)
            self._get_writer_for_target(rid).write(dm)
            rospy.loginfo(
                "dds_data_publisher: sent %s to DataTopic%s plan_id=%s",
                MSG_MULTI_AGENT_TIMING_SOLVE,
                rid,
                msg.plan_id,
            )
            time.sleep(INTER_DDS_WRITE_SLEEP_S)

    def _global_observe_start_callback(self, msg: RosTimeMsg):
        aid = self._sender_id_optional()
        self.data_writer.write(
            make_data_message(
                MSG_GLOBAL_OBSERVE_START,
                aid,
                {"sec": int(msg.data.secs), "nsec": int(msg.data.nsecs)},
            )
        )

    def multi_agent_execute_at_callback(self, msg):
        """Fan out synchronized execute time to each fleet robot's DataTopic; echo locally for orchestrator."""
        aid = self._sender_id_optional()
        fleet_ids = [int(x) for x in (msg.fleet_robot_ids or [])]
        if not fleet_ids:
            rospy.logwarn("dds_data_publisher: multi_agent_execute_at missing fleet_robot_ids; skipping DDS")
            return
        payload = {
            "plan_id": str(msg.plan_id),
            "sec": int(msg.execute_at.secs),
            "nsec": int(msg.execute_at.nsecs),
            "fleet_robot_ids": fleet_ids,
        }
        my_id_int = self.my_id_int
        for rid in fleet_ids:
            if rid == my_id_int:
                self._pub_execute_at_local.publish(msg)
                rospy.loginfo(
                    "dds_data_publisher: local multi_agent_execute_at plan_id=%s execute_at=%s",
                    msg.plan_id,
                    msg.execute_at,
                )
                continue
            dm = make_data_message(MSG_MULTI_AGENT_EXECUTE_AT, aid, payload)
            self._get_writer_for_target(rid).write(dm)
            rospy.loginfo(
                "dds_data_publisher: sent %s to DataTopic%s plan_id=%s",
                MSG_MULTI_AGENT_EXECUTE_AT,
                rid,
                msg.plan_id,
            )
            time.sleep(INTER_DDS_WRITE_SLEEP_S)

    def _star_encoder_callback(self, msg: Float32MultiArray):
        aid = self._sender_id_optional()
        self._publish_data(MSG_STAR_ENCODER_STATE, {"v": [float(x) for x in msg.data]}, sleep=True, sending_agent=aid)

    def _star_gru_out_ego_callback(self, msg: Float32MultiArray):
        aid = self._sender_id_optional()
        self._publish_data(MSG_STAR_GRU_OUT_EGO, {"v": [float(x) for x in msg.data]}, sleep=True, sending_agent=aid)

    def transform_points(self, points, forward=True):
        if self.R is None:
            return points

        points_xy = np.array([points[0, :], points[1, :]])
        if forward:
            new_point_xy = self.R @ points_xy + self.t
            new_point_theta = points[2, :] + np.arctan2(self.R[1, 0], self.R[0, 0])
            return np.concatenate((new_point_xy, new_point_theta))
        else:
            new_point_xy = self.R.T @ (points_xy - self.t)
            new_point_theta = points[2, :] - np.arctan2(self.R[1, 0], self.R[0, 0])
            return np.concatenate((new_point_xy, [new_point_theta]))

    def confirmed_object_callback(self, msg):

        # First update the pose in msg to the new frame
        x = msg.pose.position.x
        y = msg.pose.position.y
        new_point = self.transform_point([x, y, 0])
        msg.pose.position.x = new_point[0]
        msg.pose.position.y = new_point[1]

        self._publish_data(
            MSG_DETECTED_OBJECT,
            message_converter.convert_ros_message_to_dictionary(msg),
        )

        if self.db is not None:
            # Save the object to the SQLite database
            self.db.add_object(msg.class_name, msg.pose.position.x, msg.pose.position.y, self.my_id, int(time.time()))

    def labeled_callback(self, msg):
        for obj in msg.objects:
            x = obj.pose.position.x
            y = obj.pose.position.y
            new_point = self.transform_point([x, y, 0])
            new_msg = DetectedObject()
            new_msg.class_name = obj.class_name
            new_msg.pose.position.x = new_point[0]
            new_msg.pose.position.y = new_point[1]

            self._publish_data(
                MSG_LLM_DETECTED_OBJECT,
                message_converter.convert_ros_message_to_dictionary(new_msg),
            )

    def detected_object_callback(self, msg):
        for obj in msg.objects:
            class_name = obj.class_name
            if class_name == "person":
                x = obj.pose.position.x
                y = obj.pose.position.y
                new_point = self.transform_point([x, y, 0])

                new_msg = DetectedObject()
                new_msg.class_name = class_name
                new_msg.pose.position.x = new_point[0]
                new_msg.pose.position.y = new_point[1]

                self._publish_data(
                    MSG_PERSON_DETECTED,
                    message_converter.convert_ros_message_to_dictionary(new_msg),
                )

    def path_callback(self, msg):

        # First update the poses in msg to the new frame
        for i in range(len(msg.poses)):
            x = msg.poses[i].pose.position.x
            y = msg.poses[i].pose.position.y
            new_point = self.transform_point([x, y, 0])
            msg.poses[i].pose.position.x = new_point[0]
            msg.poses[i].pose.position.y = new_point[1]

        self._publish_data(MSG_PATH, message_converter.convert_ros_message_to_dictionary(msg))

    def multi_agent_planned_path_callback(self, msg):
        """Forward planned path + plan_id to DDS (inter-robot frame, same as path_callback)."""
        path_msg = copy.deepcopy(msg.path)
        for i in range(len(path_msg.poses)):
            x = path_msg.poses[i].pose.position.x
            y = path_msg.poses[i].pose.position.y
            new_point = self.transform_point([x, y, 0])
            path_msg.poses[i].pose.position.x = new_point[0]
            path_msg.poses[i].pose.position.y = new_point[1]
        payload = {
            "plan_id": msg.plan_id,
            "path": message_converter.convert_ros_message_to_dictionary(path_msg),
        }
        self._publish_data(MSG_MULTI_AGENT_PLANNED_PATH, payload)
        rospy.loginfo(
            "dds_data_publisher: sent %s plan_id=%s poses=%d",
            MSG_MULTI_AGENT_PLANNED_PATH,
            msg.plan_id,
            len(path_msg.poses),
        )

    def voice_goal_callback(self, msg):
        x = msg.x
        y = msg.y
        th = msg.theta
        new_point = self.transform_point([x, y, th])
        msg.x = new_point[0]
        msg.y = new_point[1]
        msg.theta = new_point[2]

        self._publish_data(MSG_GOAL, message_converter.convert_ros_message_to_dictionary(msg))

    def face_encoding_callback(self, msg):
        # Convert the Float64MultiArray to a list
        external = msg.external
        if external:
            return  # Ignore external face encodings

        face_encoding = list(msg.encoding)
        name = msg.name

        self._publish_data(MSG_FACE_ENCODING, {"name": name, "encoding": face_encoding})

    def invalid_goal_callback(self, msg):
        x = msg.x
        y = msg.y
        th = msg.theta
        new_point = self.transform_point([x, y, th])
        msg.x = new_point[0]
        msg.y = new_point[1]
        msg.theta = new_point[2]

        self._publish_data(MSG_INVALID_GOAL, message_converter.convert_ros_message_to_dictionary(msg))

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def shutdown(self):
        print("Shutting down DDS data publisher")
        self._target_data_writers.clear()
        self.data_writer = None
        self.publisher = None
        dispose_participant(self.participant)
        self.participant = None


if __name__ == "__main__":
    data_publisher = DataPublisher()
    rospy.on_shutdown(data_publisher.shutdown)
    data_publisher.run()
