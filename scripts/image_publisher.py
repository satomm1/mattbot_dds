import rospy
from sensor_msgs.msg import Image
import sys
import time

from cyclonedds.topic import Topic
from cyclonedds.pub import Publisher, DataWriter

from dds_utils import (
    ImageMessage,
    RobotIdError,
    create_domain_participant,
    dispose_participant,
    image_topic_name,
    reliable_qos,
    require_robot_id_int,
)


class ImagePublisher:

    def __init__(self):
        rospy.init_node("dds_image_publisher", anonymous=True)

        try:
            self.my_id_int = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)
        self.my_id = str(self.my_id_int)

        self.participant = create_domain_participant(domain_qos=True)
        self.publisher = Publisher(self.participant)

        # Create my image topic
        self.image_topic = Topic(self.participant, image_topic_name(self.my_id_int), ImageMessage)
        self.image_writer = DataWriter(self.publisher, self.image_topic, qos=reliable_qos)

        # Create a subscriber for the image topic
        self.image_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

    def image_callback(self, msg):
        # Convert ROS Image message to DDS ImageMessage
        image_message = ImageMessage(
            agent_id=self.my_id_int,
            timestamp=int(time.time()),  # Convert to milliseconds
            data=msg.data,
            width=msg.width,
            height=msg.height,
            encoding=msg.encoding
        )

        # Publish the image message
        self.image_writer.write(image_message)
        rospy.loginfo(f"Published image from agent {self.my_id} at timestamp {image_message.timestamp}")

    def run(self):
        rospy.loginfo("Image Publisher is running...")
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Shutting down Image Publisher...")
        self.image_subscriber.unregister()
        self.image_writer = None
        self.publisher = None
        dispose_participant(self.participant)
        self.participant = None
        rospy.loginfo("Image Publisher shutdown complete.")


if __name__ == "__main__":
    image_publisher = ImagePublisher()
    rospy.on_shutdown(image_publisher.shutdown)
    image_publisher.run()
