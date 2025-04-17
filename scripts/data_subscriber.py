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
import json
import numpy as np

from dds_utils import DataMessage, reliable_qos

##################################################
# This script is used to subscribe to various DataTopics
# and process them.
##################################################

class SelfDataListener(Listener):

    def __init__(self, my_id, topic_id):
        super().__init__()
        self.my_id = my_id
        self.topic_id = topic_id
        self.goal_pub = rospy.Publisher('/external_goal', Pose2D, queue_size=10)

        self.R = None
        self.t = None

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

    def update_transformation(self, R, t):
        self.R = R
        self.t = t

    def on_data_available(self, reader):
        for sample in reader.read():
            
            sending_agent = sample.sending_agent
            if sending_agent == int(self.my_id):
                # Ignore messages from me
                continue

            message_type = sample.message_type
            timestamp = sample.timestamp
            data = json.loads(sample.data)

            if self.topic_id == self.my_id:  # This is my topic, just a check
                # Process the message
                if message_type == "goal":
                    new_points = self.transform_point([data['x'], data['y'], data['theta']], forward=False)
                    x, y, theta = new_points
                    print(f"Received goal message from agent {sending_agent}: x={x}, y={y}, theta={theta}")
                    goal_msg = Pose2D()
                    goal_msg.x = x
                    goal_msg.y = y
                    goal_msg.theta = theta
                    self.goal_pub.publish(goal_msg)

class DataListener(Listener):

    def __init__(self, my_id, topic_id, object_publisher, object_sensor_publisher, path_publisher):
        super().__init__()
        self.my_id = my_id
        self.topic_id = topic_id
        self.object_publisher = object_publisher
        self.object_sensor_publisher = object_sensor_publisher
        self.path_publisher = path_publisher

        self.R = None
        self.t = None

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

    def update_transformation(self, R, t):
        self.R = R
        self.t = t

    def on_data_available(self, reader):
        for sample in reader.read():
            
            sending_agent = sample.sending_agent

            # Check if the message is from the agent
            if sending_agent == self.topic_id:
                message_type = sample.message_type
                timestamp = sample.timestamp
                data = json.loads(sample.data)
                if message_type == "detected_object":
                    new_object = message_converter.convert_dictionary_to_ros_message('mattbot_image_detection/DetectedObject', data)
                    
                    # Now convert pose of detected object to my frame
                    x = new_object.pose.position.x
                    y = new_object.pose.position.y
                    new_point = self.transform_point([x, y, 0], forward=False)
                    new_object.pose.position.x = new_point[0]
                    new_object.pose.position.y = new_point[1]

                    self.object_publisher.publish(new_object)
                    print("Received object from agent " + str(self.topic_id))
                elif message_type == "sensor_detected_objects":
                    x = data['x']
                    y = data['y']
                    w = data['w']
                    class_name = data['class']

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

                elif message_type == "path":
                    new_path = message_converter.convert_dictionary_to_ros_message('nav_msgs/Path', data)

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
            else:
                # This was a message to the agent, we can safely ignore
                continue


class DataSubscriber:

    def __init__(self):
        
        rospy.init_node('dds_data_subscriber', anonymous=True)

        # Get robot ID, Hash, and IP Address
        self.my_id = os.environ.get('ROBOT_ID')
        self.agents_subscribed = set()

        # Create a DomainParticipant, Subscriber, and Publisher
        self.participant = DomainParticipant()
        self.subscriber = Subscriber(self.participant)

        self.data_listeners = dict()
        self.data_readers = dict()

        self.R = None
        self.t = None
        transformation_subscriber = rospy.Subscriber('transformation_matrix', Float64MultiArray, self.transformation_callback)

        self.object_publisher = rospy.Publisher('/object_from_agent', DetectedObject, queue_size=10)
        self.object_sensor_publisher = rospy.Publisher('/object_from_sensor', DetectedObjectArray, queue_size=10)
        self.path_publisher = rospy.Publisher('/path_from_agent', AgentPath, queue_size=10)

        self.subscribed_agents = set()
        self.agents_to_subscribe = set()
        self.agents_to_subscribe_subscriber = rospy.Subscriber('/agents_to_subscribe', Int16MultiArray, self.agents_to_subscribe_callback)

    def transformation_callback(self, data):
        # Get the transformation matrix
        transformation_matrix = data.data

        # Reshape the transformation matrix
        self.R = np.array(transformation_matrix[:4]).reshape(2, 2)
        self.t = np.array(transformation_matrix[4:])

        for agent_id in self.data_listeners:
            self.data_listeners[agent_id].update_transformation(self.R, self.t)

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
                    new_data_topic = Topic(self.participant, 'DataTopic' + str(agent_id), DataMessage)
                    self.data_listeners[agent_id] = DataListener(self.my_id, agent_id, self.object_publisher, self.object_sensor_publisher, self.path_publisher)
                    self.data_listeners[agent_id].update_transformation(self.R, self.t)
                    self.data_readers[agent_id] = DataReader(self.subscriber, new_data_topic, listener=self.data_listeners[agent_id], qos=reliable_qos)

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


if __name__ == '__main__':
    
    data_subscriber = DataSubscriber()
    time.sleep(11)  # Wait
    rospy.on_shutdown(data_subscriber.shutdown)
    data_subscriber.run()