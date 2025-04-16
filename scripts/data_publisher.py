import rospy
from rospy_message_converter import message_converter
import tf
import rospkg
from mattbot_image_detection.msg import DetectedObject, DetectedObjectArray
# from image_detection_with_unknowns import LabeledObject, LabeledObjectArray
from sensor_msgs.msg import Image
from mattbot_dds.msg import AgentSubscription, AgentPath, AgentLocation
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Pose2D
from std_msgs.msg import Float64MultiArray, Int16MultiArray

from cyclonedds.domain import DomainParticipant, DomainParticipantQos
from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.util import duration
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import sequence
from cyclonedds.core import Qos, Policy, Listener
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsParticipant

import time
import os
import hashlib
import socket
import json
import requests
import numpy as np

from dds_utils import DataMessage, reliable_qos

class DataPublisher:

    def __init__(self):
        rospy.init_node('dds_data_publisher', anonymous=True)

        self.my_id = os.environ.get('ROBOT_ID')

        self.lease_duration_ms = 30000
        qos_profile = DomainParticipantQos()
        qos_profile.lease_duration = duration(milliseconds=self.lease_duration_ms)

        # Create a DomainParticipant, Subscriber, and Publisher
        self.participant = DomainParticipant(qos=qos_profile)
        self.publisher = Publisher(self.participant)

        # Create my data topic
        self.data_topic = Topic(self.participant, 'DataTopic' + str(self.my_id), DataMessage)
        self.data_writer = DataWriter(self.publisher, self.data_topic, qos=reliable_qos)

        self.R = None
        self.t = None
        transformation_subscriber = rospy.Subscriber('transformation_matrix', Float64MultiArray, self.transformation_callback)

        self.cone_subscriber = rospy.Subscriber('/new_cone_map', DetectedObject, self.cone_callback, queue_size=10)
        self.path_subscriber = rospy.Subscriber('/cmd_smoothed_path', Path, self.path_callback, queue_size=10)
        self.voice_goal_subscriber = rospy.Subscriber('/voice_goal', Pose2D, self.voice_goal_callback, queue_size=10)

    def transformation_callback(self, data):
        # Get the transformation matrix
        transformation_matrix = data.data

        # Reshape the transformation matrix
        self.R = np.array(transformation_matrix[:4]).reshape(2, 2)
        self.t = np.array(transformation_matrix[4:])

    def transform_point(self, point, forward=True):
        if self.R is None:
            return point

        point_xy = np.array([point[0], point[1]])
        if forward:
            new_point_xy = self.R @ point_xy + self.t
            new_point_theta = point[2] + np.arctan2(self.R[1, 0], self.R[0, 0])
            return np.concatenate((new_point_xy, [new_point_theta]))
        else:
            new_point_xy = self.R.T @ (point_xy - self.t)
            new_point_theta = point[2] - np.arctan2(self.R[1, 0], self.R[0, 0])
            return np.concatenate((new_point_xy, [new_point_theta]))

    def transform_points(self, points, forward=True):
        if self.R is None:
            return points

        points_xy = np.array([points[0,:], points[1,:]])
        if forward:
            new_point_xy = self.R @ points_xy + self.t
            new_point_theta = points[2,:] + np.arctan2(self.R[1, 0], self.R[0, 0])
            return np.concatenate((new_point_xy, new_point_theta))
        else:
            new_point_xy = self.R.T @ (points_xy - self.t)
            new_point_theta = points[2,:] - np.arctan2(self.R[1, 0], self.R[0, 0])
            return np.concatenate((new_point_xy, [new_point_theta]))

    def cone_callback(self, msg):

        # First update the pose in msg to the new frame
        x = msg.pose.position.x
        y = msg.pose.position.y
        new_point = self.transform_point([x, y, 0])
        msg.pose.position.x = new_point[0]
        msg.pose.position.y = new_point[1]

        cone_message = DataMessage(
            message_type="detected_object",
            sending_agent=int(self.my_id),
            timestamp=int(time.time()),
            data=json.dumps(message_converter.convert_ros_message_to_dictionary(msg))
        )
        self.data_writer.write(cone_message)
        time.sleep(0.01)

    def path_callback(self, msg):
        
        # First update the poses in msg to the new frame
        for i in range(len(msg.poses)):
            x = msg.poses[i].pose.position.x
            y = msg.poses[i].pose.position.y
            new_point = self.transform_point([x, y, 0])
            msg.poses[i].pose.position.x = new_point[0]
            msg.poses[i].pose.position.y = new_point[1]

        path_message = DataMessage(
            message_type="path",
            sending_agent=int(self.my_id),
            timestamp=int(time.time()),
            data=json.dumps(message_converter.convert_ros_message_to_dictionary(msg))
        )
        self.data_writer.write(path_message)
        time.sleep(0.01)

    def voice_goal_callback(self, msg):
        x = msg.x
        y = msg.y
        th = msg.theta
        new_point = self.transform_point([x, y, th])
        msg.x = new_point[0]
        msg.y = new_point[1]
        msg.theta = new_point[2]

        goal_message = DataMessage(
            message_type="goal",
            sending_agent=int(self.my_id),
            timestamp=int(time.time()),
            data=json.dumps(message_converter.convert_ros_message_to_dictionary(msg))
        )
        self.data_writer.write(goal_message)
        time.sleep(0.01)

    def run(self):
        while not rospy.is_shutdown():
            rospy.spin()

    def shutdown(self):
        print("Shutting down DDS data publisher")


if __name__ == '__main__':
    data_publisher = DataPublisher()
    rospy.on_shutdown(data_publisher.shutdown)
    data_publisher.run()
    