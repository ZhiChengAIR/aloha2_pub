# camera_publisher.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self, camera_name, device_id, topic_name):
        super().__init__(camera_name)
        self.get_logger().info(f"Device ID: {device_id}, Topic Name: {topic_name}")
        self.publisher_ = self.create_publisher(CompressedImage, topic_name, 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open camera at {device_id}")
        else:
            self.get_logger().info(f"Successfully opened camera at {device_id}")

        self.timer_period = 1.0 / 30.0  # 30 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().error(f"Failed to capture image from {self.get_name()}")
            return
        # Compress the image using JPEG format
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 100]
        result, encimg = cv2.imencode('.jpg', frame, encode_param)
        image_msg = CompressedImage()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.format = "jpeg"
        image_msg.data = encimg.tobytes()
        self.publisher_.publish(image_msg)
        # self.get_logger().info(f"Published image from {self.get_name()}")

    def __del__(self):
        self.cap.release()
        self.get_logger().info(f"Camera at {self.cap} released")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('camera_publisher_temp')
    node.declare_parameter('camera_name')
    node.declare_parameter('device_id')
    node.declare_parameter('topic_name')
    cam_name = node.get_parameter('camera_name').get_parameter_value().string_value
    device_id = node.get_parameter('device_id').get_parameter_value().string_value
    topic_name = node.get_parameter('topic_name').get_parameter_value().string_value
    node = CameraPublisher(cam_name, device_id, topic_name)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
