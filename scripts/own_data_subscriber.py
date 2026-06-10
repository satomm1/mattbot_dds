import rospy
import tf
import sys
from rospy_message_converter import message_converter
from geometry_msgs.msg import Pose2D, PoseWithCovarianceStamped
from std_msgs.msg import Bool, Float64MultiArray, UInt32

from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.pub import Publisher
from cyclonedds.core import Listener

from database_utils import RobotDatabase

import time
import json
import numpy as np

from dds_utils import (
    MSG_GOAL,
    MSG_MULTI_ROBOT_GOAL,
    MSG_MULTI_AGENT_EXECUTE_AT,
    MSG_MULTI_AGENT_TIMING_SOLVE,
    MSG_MULTI_AGENT_ACTIVE_TRAJECTORY,
    MSG_POSITION_INIT,
    MSG_ROBOT_SHUTDOWN,
    MSG_SEND_UNKNOWN_IMAGES,
    MSG_STOP,
    DataMessage,
    DdsLogger,
    POSITION_INIT_RECENT_THRESHOLD_S,
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
from mattbot_dds.msg import (
    MultiRobotExternalGoal,
    MultiAgentExecuteAt,
    MultiAgentTimingSolve,
    MultiAgentActiveTrajectory,
)

_log = DdsLogger("own_data_subscriber")

##################################################
# This script process data messages sent to this agent
##################################################


class SelfDataListener(Listener, TransformMixin):

    def __init__(self, my_id, my_id_int, topic_id, sqlite_db=None):
        super().__init__()
        self.init_transform_state()
        self.my_id = my_id
        self.my_id_int = my_id_int
        self.topic_id = topic_id
        self.goal_pub = rospy.Publisher("/external_goal", Pose2D, queue_size=10)
        self._external_goal_multi_topic = rospy.get_param(
            "~external_goal_multi_ros_topic", "/external_goal_multi"
        ).strip() or "/external_goal_multi"
        self._mirror_multi_to_external_goal = bool(
            rospy.get_param("~mirror_multi_robot_goal_to_external_goal", False)
        )
        self.goal_multi_pub = rospy.Publisher(
            self._external_goal_multi_topic, MultiRobotExternalGoal, queue_size=10, latch=False
        )
        self._multi_agent_execute_at_ros_topic = rospy.get_param(
            "~multi_agent_execute_at_ros_topic", "/multi_agent_execute_at"
        ).strip() or "/multi_agent_execute_at"
        self.multi_agent_execute_at_pub = rospy.Publisher(
            self._multi_agent_execute_at_ros_topic, MultiAgentExecuteAt, queue_size=2, latch=True
        )
        self._multi_agent_timing_solve_ros_topic = rospy.get_param(
            "~multi_agent_timing_solve_ros_topic", "/multi_agent_timing_solve"
        ).strip() or "/multi_agent_timing_solve"
        self.multi_agent_timing_solve_pub = rospy.Publisher(
            self._multi_agent_timing_solve_ros_topic, MultiAgentTimingSolve, queue_size=2, latch=False
        )
        self._multi_agent_active_traj_ros_topic = rospy.get_param(
            "~multi_agent_active_trajectory_ros_topic", "/multi_agent_active_trajectory"
        ).strip() or "/multi_agent_active_trajectory"
        self.multi_agent_active_traj_pub = rospy.Publisher(
            self._multi_agent_active_traj_ros_topic, MultiAgentActiveTrajectory, queue_size=2, latch=False
        )
        _log.debug(
            "multi-agent ROS pubs -> execute_at=%s timing_solve=%s active_traj=%s",
            self._multi_agent_execute_at_ros_topic,
            self._multi_agent_timing_solve_ros_topic,
            self._multi_agent_active_traj_ros_topic,
        )
        self.send_unknown_images_pub = rospy.Publisher("/send_unknown_images", UInt32, queue_size=10)
        self.init_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

        self._stop_topic = rospy.get_param("~stop_ros_topic", "/stop").strip() or "/stop"
        self._stop_pub = rospy.Publisher(self._stop_topic, Bool, queue_size=1, latch=False)
        self._allow_dds_roslaunch_shutdown = bool(rospy.get_param("~allow_dds_roslaunch_shutdown", True))

        # Database connection
        self.db = sqlite_db

        _log.debug("created listener for topic %s", topic_id)

    def on_data_available(self, reader):
        for sample in reader.read():

            sending_agent = sample.sending_agent
            if sending_agent == self.my_id_int:
                # Ignore messages from me
                continue

            message_type = sample.message_type
            timestamp = sample.timestamp
            data = json.loads(sample.data)

            _log.debug(
                "RX from agent %s type=%s",
                sending_agent,
                message_type,
            )

            # Process the message
            if message_type == MSG_GOAL:

                # Transform the goal point to this occupancy grid
                x, y, theta = self.transform_point([data["x"], data["y"], data["theta"]], forward=False)

                _log.debug(
                    "goal from agent %s x=%s y=%s theta=%s",
                    sending_agent,
                    x,
                    y,
                    theta,
                )
                goal_msg = Pose2D()
                goal_msg.x = x
                goal_msg.y = y
                goal_msg.theta = theta
                self.goal_pub.publish(goal_msg)

                if self.db is not None:
                    self.db.add_goal(self.my_id, x, y, theta, timestamp)

            elif message_type == MSG_MULTI_ROBOT_GOAL:

                x, y, theta = self.transform_point([data["x"], data["y"], data["theta"]], forward=False)
                plan_id = data.get("plan_id", "")
                coordinated = bool(data.get("coordinated", True))
                target_agent = int(data.get("target_agent", self.my_id_int))
                if target_agent != self.my_id_int:
                    _log.warn(
                        "multi_robot_goal target_agent %s != my_id %s; using transformed pose anyway",
                        target_agent,
                        self.my_id,
                    )
                ext = MultiRobotExternalGoal()
                ext.goal.x = x
                ext.goal.y = y
                ext.goal.theta = theta
                ext.plan_id = plan_id
                ext.coordinated = coordinated
                ext.source_agent = int(sending_agent)
                ext.target_agent = self.my_id_int
                fleet = data.get("fleet_robot_ids")
                if isinstance(fleet, list):
                    ext.fleet_robot_ids = [int(x) for x in fleet]
                else:
                    ext.fleet_robot_ids = []
                self.goal_multi_pub.publish(ext)
                _log.debug(
                    "Received multi_robot_goal from agent %s plan_id=%s",
                    sending_agent,
                    plan_id,
                )
                if self._mirror_multi_to_external_goal:
                    g = Pose2D()
                    g.x, g.y, g.theta = x, y, theta
                    self.goal_pub.publish(g)
                if self.db is not None:
                    self.db.add_goal(self.my_id, x, y, theta, timestamp)
            elif message_type == MSG_MULTI_AGENT_EXECUTE_AT:
                plan_id = data.get("plan_id", "")
                sec, nsec = data.get("sec"), data.get("nsec")
                if sec is None or nsec is None:
                    _log.warn("multi_agent_execute_at: missing sec/nsec from agent %s", sending_agent)
                    continue
                fleet = data.get("fleet_robot_ids")
                fleet_ids = [int(x) for x in fleet] if isinstance(fleet, list) else []
                ext = MultiAgentExecuteAt()
                ext.plan_id = str(plan_id)
                ext.execute_at = rospy.Time(int(sec), int(nsec))
                ext.fleet_robot_ids = fleet_ids
                self.multi_agent_execute_at_pub.publish(ext)
                _log.debug(
                    "multi_agent_execute_at from agent %s plan_id=%s execute_at=%s",
                    sending_agent,
                    plan_id,
                    ext.execute_at,
                )
            elif message_type == MSG_MULTI_AGENT_TIMING_SOLVE:
                plan_id = data.get("plan_id", "")
                src = data.get("source_agent")
                if src is None:
                    _log.warn("multi_agent_timing_solve: missing source_agent from agent %s", sending_agent)
                    continue
                fleet = data.get("fleet_robot_ids")
                fleet_ids = [int(x) for x in fleet] if isinstance(fleet, list) else []
                wc = data.get("waypoint_counts")
                counts = [int(x) for x in wc] if isinstance(wc, list) else []
                wtf = data.get("waypoint_times_flat")
                flat = [float(x) for x in wtf] if isinstance(wtf, list) else []
                out = MultiAgentTimingSolve()
                out.plan_id = str(plan_id)
                out.source_agent = int(src)
                out.fleet_robot_ids = fleet_ids
                out.waypoint_counts = counts
                out.waypoint_times_flat = flat
                self.multi_agent_timing_solve_pub.publish(out)
                _log.debug(
                    "multi_agent_timing_solve from agent %s plan_id=%s source=%s",
                    sending_agent,
                    plan_id,
                    out.source_agent,
                )
            elif message_type == MSG_MULTI_AGENT_ACTIVE_TRAJECTORY:
                rid = data.get("robot_id")
                if rid is None:
                    _log.warn("multi_agent_active_trajectory: missing robot_id from agent %s", sending_agent)
                    continue
                sec, nsec = data.get("sec"), data.get("nsec")
                if sec is None or nsec is None:
                    _log.warn("multi_agent_active_trajectory: missing execute_at from agent %s", sending_agent)
                    continue
                path_dict = data.get("path")
                if not isinstance(path_dict, dict):
                    _log.warn("multi_agent_active_trajectory: invalid path from agent %s", sending_agent)
                    continue
                new_path = message_converter.convert_dictionary_to_ros_message("nav_msgs/Path", path_dict)
                for i in range(len(new_path.poses)):
                    x = new_path.poses[i].pose.position.x
                    y = new_path.poses[i].pose.position.y
                    new_point = self.transform_point([x, y, 0], forward=False)
                    new_path.poses[i].pose.position.x = new_point[0]
                    new_path.poses[i].pose.position.y = new_point[1]
                wtf = data.get("waypoint_times")
                flat = [float(x) for x in wtf] if isinstance(wtf, list) else []
                fwd = data.get("dds_forward_robot_ids")
                forward_ids = [int(x) for x in fwd] if isinstance(fwd, list) else []
                out = MultiAgentActiveTrajectory()
                out.robot_id = int(rid)
                out.plan_id = str(data.get("plan_id", ""))
                out.active = bool(data.get("active", True))
                out.execute_at = rospy.Time(int(sec), int(nsec))
                out.path = new_path
                out.waypoint_times = flat
                out.dds_forward_robot_ids = forward_ids
                self.multi_agent_active_traj_pub.publish(out)
                _log.debug(
                    "multi_agent_active_trajectory from agent %s robot_id=%s active=%s",
                    sending_agent,
                    out.robot_id,
                    out.active,
                )
            elif message_type == MSG_POSITION_INIT:
                # Transform the position to this occupancy grid
                x, y, theta = self.transform_point([data["x"], data["y"], data["theta"]], forward=False)

                # Check that timestamp is recent
                current_time = rospy.Time.now()
                message_time = rospy.Time.from_sec(timestamp)
                if (current_time - message_time).to_sec() > POSITION_INIT_RECENT_THRESHOLD_S:
                    continue

                _log.debug(
                    "position_init from agent %s x=%s y=%s theta=%s",
                    sending_agent,
                    x,
                    y,
                    theta,
                )
                init_msg = PoseWithCovarianceStamped()
                init_msg.header.stamp = rospy.Time.now()
                init_msg.header.frame_id = "map"  # Assuming the frame is 'map'
                init_msg.pose.pose.position.x = x
                init_msg.pose.pose.position.y = y
                init_msg.pose.pose.position.z = 0.0  # Assuming a 2D position
                orientation = tf.transformations.quaternion_from_euler(0, 0, theta)
                init_msg.pose.pose.orientation.x = orientation[0]
                init_msg.pose.pose.orientation.y = orientation[1]
                init_msg.pose.pose.orientation.z = orientation[2]
                init_msg.pose.pose.orientation.w = orientation[3]

                covariance = np.zeros((6, 6))
                covariance[0, 0] = 0.25  # Variance in x
                covariance[1, 1] = 0.25  # Variance in y
                covariance[5, 5] = 6.28  # Variance in theta
                init_msg.pose.covariance = covariance.flatten().tolist()

                self.init_pub.publish(init_msg)
            elif message_type == MSG_SEND_UNKNOWN_IMAGES:
                # Publish to topic to let the image detection node know to send unknown images
                # We need to send the agent id to which the images should be sent
                msg = UInt32()
                msg.data = sending_agent
                self.send_unknown_images_pub.publish(msg)

            elif message_type == MSG_STOP:
                _log.info(
                    "stop from agent %s ts=%s payload=%s -> %s",
                    sending_agent,
                    timestamp,
                    data,
                    self._stop_topic,
                )
                self._stop_pub.publish(Bool(data=True))

            elif message_type == MSG_ROBOT_SHUTDOWN:
                if not self._allow_dds_roslaunch_shutdown:
                    _log.warn(
                        "ignoring robot_shutdown from agent %s (allow_dds_roslaunch_shutdown is false)",
                        sending_agent,
                    )
                    continue
                reason = data.get("reason") if isinstance(data, dict) else None
                extra = " reason=%r" % (reason,) if reason else ""
                _log.warn(
                    "robot_shutdown from agent %s ts=%s payload=%s%s; signaling rospy shutdown",
                    sending_agent,
                    timestamp,
                    data,
                    extra,
                )
                rospy.signal_shutdown(
                    "dds robot_shutdown from agent %s%s" % (sending_agent, extra)
                )


class OwnDataSubscriber(TransformMixin):

    def __init__(self):

        rospy.init_node("dds_own_data_subscriber", anonymous=True)

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
            self.db.create_goals_table()

        self.participant = create_domain_participant(domain_qos=False)
        self.subscriber = Subscriber(self.participant)
        self.publisher = Publisher(self.participant)

        # Create my data topic
        self.data_topic = Topic(self.participant, data_topic_name(self.my_id_int), DataMessage)
        self.data_listener = SelfDataListener(self.my_id, self.my_id_int, self.my_id_int, sqlite_db=self.db)
        self.data_reader = DataReader(self.subscriber, self.data_topic, listener=self.data_listener, qos=reliable_qos)

        rospy.Subscriber(ROS_TOPIC_TRANSFORMATION_MATRIX, Float64MultiArray, self.transformation_callback)

    def transformation_callback(self, data):

        # Get the transformation matrix
        self.R, self.t = parse_transform_msg(data)

        self.data_listener.update_transformation(self.R, self.t)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def shutdown(self):
        _log.debug("Shutting down")
        self.data_reader = None
        self.subscriber = None
        self.publisher = None
        dispose_participant(self.participant)
        self.participant = None


if __name__ == "__main__":
    own_data_subscriber = OwnDataSubscriber()
    rospy.on_shutdown(own_data_subscriber.shutdown)
    own_data_subscriber.run()
