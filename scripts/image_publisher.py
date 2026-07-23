import sys
import threading
import time

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from cyclonedds.topic import Topic
from cyclonedds.pub import Publisher, DataWriter

from dds_utils import (
    DdsLogger,
    ImageMessage,
    RobotIdError,
    create_domain_participant,
    dispose_participant,
    image_qos,
    image_topic_name,
    require_robot_id_int,
)

_log = DdsLogger("image_publisher")


class ImagePublisher:

    def __init__(self):
        rospy.init_node("dds_image_publisher", anonymous=True)

        try:
            self.my_id_int = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)
        self.my_id = str(self.my_id_int)

        self.image_topic_ros = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 4.0))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 80))
        self.max_width = int(rospy.get_param("~max_width", 640))
        # Tall mount: camera is upside-down (same as capture / OSOD).
        self.tall = bool(rospy.get_param("~tall", False))
        # Wall clock at publish = UI latency metric; capture stamp can lag ROS/camera time.
        self.use_capture_stamp = bool(rospy.get_param("~use_capture_stamp", False))
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_msg = None
        self._publish_timer = None

        self.participant = create_domain_participant(domain_qos=True)
        self.publisher = Publisher(self.participant)

        topic_name = image_topic_name(self.my_id_int)
        self.image_topic = Topic(self.participant, topic_name, ImageMessage)
        self.image_writer = DataWriter(self.publisher, self.image_topic, qos=image_qos)

        self.image_subscriber = rospy.Subscriber(
            self.image_topic_ros, Image, self.image_callback, queue_size=1, buff_size=2**24
        )

        if self.publish_rate_hz > 0.0:
            period = 1.0 / self.publish_rate_hz
            self._publish_timer = rospy.Timer(rospy.Duration(period), self._on_publish_timer)
        else:
            rospy.logwarn("publish_rate_hz <= 0; no frames will be published")

        _log.debug(
            "publishing %s -> DDS %s (rate=%.1f Hz jpeg_quality=%d max_width=%d tall=%s use_capture_stamp=%s)",
            self.image_topic_ros,
            topic_name,
            self.publish_rate_hz,
            self.jpeg_quality,
            self.max_width,
            self.tall,
            self.use_capture_stamp,
        )

    def image_callback(self, msg):
        # Keep only the newest frame; encode/publish on the timer so we never stream a backlog.
        with self._lock:
            self._latest_msg = msg

    def _on_publish_timer(self, _event):
        with self._lock:
            msg = self._latest_msg
            self._latest_msg = None
        if msg is None:
            return

        jpeg_bytes, width, height, err = self._encode_jpeg(msg)
        if err is not None:
            _log.warn("skip frame: %s", err)
            return

        now = time.time()
        if self.use_capture_stamp and msg.header.stamp.to_sec() > 0.0:
            stamp = msg.header.stamp.to_sec()
        else:
            stamp = float(now)

        image_message = ImageMessage(
            agent_id=self.my_id_int,
            timestamp=float(stamp),
            data=list(jpeg_bytes),
            width=width,
            height=height,
            encoding="jpeg",
        )
        self.image_writer.write(image_message)
        _log.debug(
            "published JPEG %dx%d (%d bytes) agent=%s ts=%s",
            width,
            height,
            len(jpeg_bytes),
            self.my_id,
            image_message.timestamp,
        )

    def _encode_jpeg(self, msg):
        try:
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            return None, None, None, "cv_bridge error: %s" % exc

        if self.tall:
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_180)

        height, width = cv_image.shape[:2]
        if self.max_width > 0 and width > self.max_width:
            scale = float(self.max_width) / float(width)
            new_w = self.max_width
            new_h = max(1, int(round(height * scale)))
            cv_image = cv2.resize(cv_image, (new_w, new_h), interpolation=cv2.INTER_AREA)
            height, width = new_h, new_w

        ok, encoded = cv2.imencode(
            ".jpg", cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            return None, None, None, "jpeg encode failed"
        return encoded.tobytes(), width, height, None

    def run(self):
        rospy.loginfo(
            "DDS image publisher running: %s -> %s (%.1f Hz, latest-frame)",
            self.image_topic_ros,
            image_topic_name(self.my_id_int),
            self.publish_rate_hz,
        )
        rospy.spin()

    def shutdown(self):
        _log.debug("Shutting down")
        if self._publish_timer is not None:
            self._publish_timer.shutdown()
            self._publish_timer = None
        if self.image_subscriber is not None:
            self.image_subscriber.unregister()
        self.image_writer = None
        self.publisher = None
        dispose_participant(self.participant)
        self.participant = None


if __name__ == "__main__":
    image_publisher = ImagePublisher()
    rospy.on_shutdown(image_publisher.shutdown)
    image_publisher.run()
