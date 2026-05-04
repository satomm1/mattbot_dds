import rospy
import tf
import sys
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
    MSG_POSITION_INIT,
    MSG_SEND_UNKNOWN_IMAGES,
    MSG_STOP,
    DataMessage,
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
from mattbot_dds.msg import MultiRobotExternalGoal

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
        self.send_unknown_images_pub = rospy.Publisher("/send_unknown_images", UInt32, queue_size=10)
        self.init_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=10)

        self._stop_topic = rospy.get_param("~stop_ros_topic", "/stop").strip() or "/stop"
        self._stop_pub = rospy.Publisher(self._stop_topic, Bool, queue_size=1, latch=False)

        # Database connection
        self.db = sqlite_db

        print("Created listener for topic", topic_id)

    def on_data_available(self, reader):
        for sample in reader.read():

            sending_agent = sample.sending_agent
            if sending_agent == self.my_id_int:
                # Ignore messages from me
                continue

            message_type = sample.message_type
            timestamp = sample.timestamp
            data = json.loads(sample.data)

            print("Received message from agent", sending_agent, "of type", message_type)

            # Process the message
            if message_type == MSG_GOAL:

                # Transform the goal point to this occupancy grid
                x, y, theta = self.transform_point([data["x"], data["y"], data["theta"]], forward=False)

                print(f"Received goal message from agent {sending_agent}: x={x}, y={y}, theta={theta}")
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
                    rospy.logwarn(
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
                self.goal_multi_pub.publish(ext)
                rospy.loginfo(
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
            elif message_type == MSG_POSITION_INIT:
                # Transform the position to this occupancy grid
                x, y, theta = self.transform_point([data["x"], data["y"], data["theta"]], forward=False)

                # Check that timestamp is recent
                current_time = rospy.Time.now()
                message_time = rospy.Time.from_sec(timestamp)
                if (current_time - message_time).to_sec() > POSITION_INIT_RECENT_THRESHOLD_S:
                    continue

                print(f"Received position_init message from agent {sending_agent}: x={x}, y={y}, theta={theta}")
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
                rospy.loginfo(
                    "dds_own_data_subscriber: stop from agent %s ts=%s payload=%s -> %s",
                    sending_agent,
                    timestamp,
                    data,
                    self._stop_topic,
                )
                self._stop_pub.publish(Bool(data=True))


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
        print("Shutting down DDS Own Data Subscriber")
        self.data_reader = None
        self.subscriber = None
        self.publisher = None
        dispose_participant(self.participant)
        self.participant = None


if __name__ == "__main__":
    own_data_subscriber = OwnDataSubscriber()
    rospy.on_shutdown(own_data_subscriber.shutdown)
    own_data_subscriber.run()
