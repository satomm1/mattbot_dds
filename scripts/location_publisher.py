import rospy
from std_msgs.msg import Float64MultiArray, Int32, Bool, Int16MultiArray
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
import tf

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
import numpy as np

from dds_utils import Location, best_effort_qos

from navigation_utils import StochOccupancyGrid2D

LOCATION_PERIOD = 0.5    # seconds

class LocationPublisher:
    def __init__(self):
        rospy.init_node('dds_location_publisher', anonymous=True)

        # Get robot ID, Hash, and IP Address
        self.my_id = os.environ.get('ROBOT_ID')

        self.lease_duration_ms = 30000
        qos_profile = DomainParticipantQos()
        qos_profile.lease_duration = duration(milliseconds=self.lease_duration_ms)

        # Store current location in the local frame
        self.x = None
        self.y = None
        self.theta = None

        # Create a DomainParticipant, Subscriber, and Publisher
        self.participant = DomainParticipant(qos=qos_profile)
        self.publisher = Publisher(self.participant)

        self.location_topic = Topic(self.participant, 'LocationTopic' + str(self.my_id), Location)
        self.location_writer = DataWriter(self.publisher, self.location_topic, qos=best_effort_qos)

        self.trans_listener = tf.TransformListener()

        self.R = None
        self.t = None
        transformation_subscriber = rospy.Subscriber('transformation_matrix', Float64MultiArray, self.transformation_callback)

        self.is_static = False
        robot_mode_subscriber = rospy.Subscriber("/robot_mode", Int32, self.robot_mode_callback)

        # Track the agents to subscribe to
        self.agents_to_subscribe = set()
        self.agents_to_subscribe_subscriber = rospy.Subscriber('/agents_to_subscribe', Int16MultiArray, self.agents_to_subscribe_callback, queue_size=10)
        self.rendezvous_subscriber = rospy.Subscriber('/rendezvous', Bool, self.rendezvous_callback, queue_size=1)

        # Wait until we get the /map message
        map_msg = rospy.wait_for_message("/map", OccupancyGrid)
        self.occupancy = StochOccupancyGrid2D(
            map_msg.info.resolution,
            map_msg.info.width,
            map_msg.info.height,
            map_msg.info.origin.position.x,
            map_msg.info.origin.position.y,
            5,
            map_msg.data,
        )


    def agents_to_subscribe_callback(self, data):
        # Get the list of agents to subscribe to
        agents_to_subscribe = data.data

        self.agents_to_subscribe = set(agents_to_subscribe)

    def transformation_callback(self, data):
        # Get the transformation matrix
        transformation_matrix = data.data

        # Reshape the transformation matrix
        self.R = np.array(transformation_matrix[:4]).reshape(2, 2)
        self.t = np.array(transformation_matrix[4:])

    def robot_mode_callback(self, data):
        if data.data == 0:
            self.is_static = True
        else:
            self.is_static = False

    def rendezvous_callback(self, data):
        """
        If we get a rendezvous command, we publish a goal of our current location to 
        all other agents.
        """
        if data.data is False:
            return

        print("Rendezvous command received, publishing goals to other agents.")

        print("current location:", self.x, self.y)

         # Check if current location is available
        if self.x is None or self.y is None or (self.occupancy.is_free((self.x, self.y)) is False):
            print("Current position not valid, cannot publish rendezvous goals.")
            return
        
        # We need to provide a slightly different goal to each robot so that they don't collide
        num_agents = len(self.agents_to_subscribe) - 1 # exclude a human server

        # Other robot goals will be offset from our current location by radius
        radius = 0.6
        robot_diameter = 0.45
        other_goals = []

        while len(other_goals) < num_agents:
            angle = 0
            
            while angle < 2 * np.pi:
                offset_x = radius * np.cos(angle)
                offset_y = radius * np.sin(angle)
                goal_x = self.x + offset_x
                goal_y = self.y + offset_y

                if self.occupancy.is_free((goal_x, goal_y)):
                    other_goals.append((goal_x, goal_y))
                    if len(other_goals) >= num_agents:
                        break

                # Adjust angle based on robot size to avoid collisions
                angle += 2 * np.arcsin(robot_diameter / (2 * radius)) + 0.1  # small buffer angle
            radius += 0.5  # increase radius if not enough goals found

    
        # Create the Pose2D message of current location
        msg = Pose2D()
        
        # Now publish the message the each agent
        for agent_id in self.agents_to_subscribe:
            if agent_id != int(self.my_id):
                # Get the goal for the first agent
                goal_x, goal_y = other_goals.pop(0)

                # Convert to global frame
                transformed_goal = self.transform_point([goal_x, goal_y, 0])
                msg.x = transformed_goal[0]
                msg.y = transformed_goal[1]
                msg.theta = 0.0

                # Create DDS Data Message with goal
                goal_message = DataMessage(
                    message_type="goal",
                    sending_agent=int(self.my_id),
                    timestamp=int(time.time()),
                    data=json.dumps(message_converter.convert_ros_message_to_dictionary(msg))
                )

                # Publish a rendezvous goal to the agent
                data_topic = Topic(self.participant, 'DataTopic' + str(agent_id), DataMessage)
                data_writer = DataWriter(self.publisher, data_topic, qos=reliable_qos)
                data_writer.write(goal_message)

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

    def run(self):
        while not rospy.is_shutdown():

            # Get current position of the agent
            try:
                (translation, rotation) = self.trans_listener.lookupTransform("map", "base_footprint", rospy.Time(0))
                x = translation[0]
                y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                theta = euler[2]

                self.x = x
                self.y = y
                self.theta = theta

                transformed_point = self.transform_point([x, y, theta])
                x_new, y_new, theta_new = transformed_point
                
                location = Location(int(self.my_id), int(time.time()), x_new, y_new, theta_new, self.is_static)
                self.location_writer.write(location)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                # Location not available yet (not yet localized)
                pass

            # Sleep for LOCATION_PERIOD seconds
            rospy.sleep(LOCATION_PERIOD)

    def shutdown(self):
        rospy.loginfo("Shutting down DDS location publisher...")

if __name__ == '__main__':
    location_publisher = LocationPublisher()
    rospy.on_shutdown(location_publisher.shutdown)
    location_publisher.run()
