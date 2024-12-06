import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import serial
import time
from cv_bridge import CvBridge

class ArucoMarkerNode(Node): 
    def __init__(self):
        super().__init__('aruco_marker_node')
        self.get_logger().info("Initializing ArUco Marker Node...")

        # Publisher for detected marker IDs
        self.marker_publisher = self.create_publisher(String, 'aruco_markers', 10)

        # Subscriber for camera frames
        self.camera_subscriber = self.create_subscription(Image, 'camera_frames', self.process_frame, 10)
        self.bridge = CvBridge()

        # Set up ArUco dictionary and parameters
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.wanted_markers = [2, 9, 16]

        # Arduino connection
        try:
            self.arduino = serial.Serial('/dev/ttyUSB1', 9600)
            time.sleep(2)  # Wait for the connection to initialize
            self.get_logger().info("Arduino connected.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.arduino = None

    def process_frame(self, msg): 
        """Process incoming camera frames."""
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Detect ArUco markers in the frame
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            self.get_logger().info(f"Detected marker IDs: {ids.flatten().tolist()}")
            for marker_id in ids.flatten():
                if marker_id in self.wanted_markers:
                    self.send_arduino_command("yes")
                    self.get_logger().info("Correct marker detected. Sent 'yes' to Arduino.")
                else:
                    self.send_arduino_command("no")
                    self.get_logger().info("Incorrect marker detected. Sent 'no' to Arduino.")

            # Publish detected markers to ROS 2 topic
            detected_ids = ','.join(map(str, ids.flatten()))
            self.marker_publisher.publish(String(data=f"Detected marker IDs: {detected_ids}"))
        else:
            self.send_arduino_command("waiting")
            self.get_logger().info("No markers detected. Sent 'waiting' to Arduino.")

        # Display the frame with markers highlighted
        cv2.imshow("ArUco Marker Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def send_arduino_command(self, command):
        """Send a command to the Arduino."""
        if self.arduino:
            try:
                self.arduino.write((command + '\n').encode())
                time.sleep(2)  # Allow time for Arduino to process
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to send command to Arduino: {e}")
        else:
            self.get_logger().warning("Arduino not connected. Skipping command.")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ArUco Marker Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
