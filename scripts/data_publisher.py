import rospy
from rospy_message_converter import message_converter
import tf
import rospkg
from mattbot_image_detection.msg import DetectedObject, DetectedObjectArray
from mattbot_image_detection.msg import LabeledObject, LabeledObjectArray
from sensor_msgs.msg import Image
from mattbot_dds.msg import AgentSubscription, AgentPath, AgentLocation
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, Pose2D
from std_msgs.msg import Float64MultiArray, Int16MultiArray
from mattbot_image_detection.msg import FaceEncoding

from cyclonedds.domain import DomainParticipant, DomainParticipantQos
from cyclonedds.topic import Topic
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.util import duration
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import sequence
from cyclonedds.core import Qos, Policy, Listener
from cyclonedds.builtin import BuiltinDataReader, BuiltinTopicDcpsParticipant

from database_utils import RobotDatabase

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

        # Get sqlite parameter
        self.sqlite = rospy.get_param('~sqlite', False)
        self.db = None
        if self.sqlite:
            self.db = RobotDatabase()

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

        self.object_subscriber = rospy.Subscriber('/confirmed_objects', DetectedObject, self.confirmed_object_callback, queue_size=10)
        self.path_subscriber = rospy.Subscriber('/cmd_smoothed_path', Path, self.path_callback, queue_size=10)
        self.voice_goal_subscriber = rospy.Subscriber('/voice_goal', Pose2D, self.voice_goal_callback, queue_size=10)
        self.new_face_encoding_subscriber = rospy.Subscriber('/new_face_encoding', FaceEncoding, self.face_encoding_callback, queue_size=10)
        self.llm_image_subscriber = rospy.Subscriber('/labeled_unknown_objects', LabeledObjectArray, self.labeled_callback, queue_size=3)
        self.invalid_goal_subscriber = rospy.Subscriber('/invalid_goal', Pose2D, self.invalid_goal_callback, queue_size=10)
        self.detected_object_subscriber = rospy.Subscriber('/detected_objects', DetectedObjectArray, self.detected_object_callback, queue_size=10)

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

    def confirmed_object_callback(self, msg):

        # First update the pose in msg to the new frame
        x = msg.pose.position.x
        y = msg.pose.position.y
        new_point = self.transform_point([x, y, 0])
        msg.pose.position.x = new_point[0]
        msg.pose.position.y = new_point[1]

        object_message = DataMessage(
            message_type="detected_object",
            sending_agent=int(self.my_id),
            timestamp=int(time.time()),
            data=json.dumps(message_converter.convert_ros_message_to_dictionary(msg))
        )
        self.data_writer.write(object_message)
        time.sleep(0.01)

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

            object_message = DataMessage(
                message_type="llm_detected_object",
                sending_agent=int(self.my_id),
                timestamp=int(time.time()),
                data=json.dumps(message_converter.convert_ros_message_to_dictionary(new_msg))
            )
            self.data_writer.write(object_message)
            time.sleep(0.01)

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

                object_message = DataMessage(
                    message_type="person_detected",
                    sending_agent=int(self.my_id),
                    timestamp=int(time.time()),
                    data=json.dumps(message_converter.convert_ros_message_to_dictionary(new_msg))
                )
                self.data_writer.write(object_message)
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

    def face_encoding_callback(self, msg):
        # Convert the Float64MultiArray to a list
        external = msg.external
        if external:
            return  # Ignore external face encodings

        face_encoding = list(msg.encoding)
        name = msg.name

        face_encoding_message = DataMessage(
            message_type="face_encoding",
            sending_agent=int(self.my_id),
            timestamp=int(time.time()),
            data=json.dumps({
                "name": name,
                "encoding": face_encoding
            })
        )
        self.data_writer.write(face_encoding_message)
        time.sleep(0.01)

    def invalid_goal_callback(self, msg):
        x = msg.x
        y = msg.y
        th = msg.theta
        new_point = self.transform_point([x, y, th])
        msg.x = new_point[0]
        msg.y = new_point[1]
        msg.theta = new_point[2]

        goal_message = DataMessage(
            message_type="invalid_goal",
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
    