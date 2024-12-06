import rclpy
from rclpy.node import Node
import cv2
import depthai as dai
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import time


class CameraControlNode(Node):
    def __init__(self):
        super().__init__('camera_control_node')
        self.get_logger().info("Initializing Camera Control Node...")

        # State definitions
        self.state = "PERSON_DETECTION"  # Initial state
        self.distance_threshold = 1400  # Distance (mm) to stop driving
        self.stop_distance = 1400  # Minimum distance to person before stopping

        # ROS2 Publishers
        self.person_publisher = self.create_publisher(String, 'person_detection', 10)
        self.marker_publisher = self.create_publisher(String, 'aruco_markers', 10)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist_cmd = Twist()

        # Arduino Connection
        try:
            self.arduino = serial.Serial('/dev/ttyUSB0', 9600)
            time.sleep(2)  # Allow the connection to initialize
            self.get_logger().info("Arduino connected.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.arduino = None

        # DepthAI pipeline
        self.pipeline = dai.Pipeline()
        self.setup_pipeline()

        # Running DepthAI device
        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.q_detections = self.device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        self.get_logger().info("DepthAI device initialized.")

        # ArUco marker setup
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.wanted_markers = [2, 9, 16]  # Example marker IDs for "yes"

        # Timer
        self.create_timer(0.1, self.process)

    def setup_pipeline(self):
        # DepthAI pipeline components for person detection
        cam_rgb = self.pipeline.createColorCamera()
        spatial_network = self.pipeline.createMobileNetSpatialDetectionNetwork()
        mono_left = self.pipeline.createMonoCamera()
        mono_right = self.pipeline.createMonoCamera()
        stereo = self.pipeline.createStereoDepth()

        xout_rgb = self.pipeline.createXLinkOut()
        xout_detections = self.pipeline.createXLinkOut()

        xout_rgb.setStreamName("rgb")
        xout_detections.setStreamName("detections")

        cam_rgb.setPreviewSize(300, 300)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo.setOutputDepth(True)
        stereo.setConfidenceThreshold(255)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)

        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

        spatial_network.setBlobPath("/home/projects/ros2_ws/src/camera_control/camera_control/mobilenet-ssd_openvino_2021.4_6shave.blob")
        spatial_network.setConfidenceThreshold(0.5)
        spatial_network.setBoundingBoxScaleFactor(0.5)
        spatial_network.setDepthLowerThreshold(100)
        spatial_network.setDepthUpperThreshold(10000)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        stereo.depth.link(spatial_network.inputDepth)
        cam_rgb.preview.link(spatial_network.input)

        spatial_network.out.link(xout_detections.input)
        cam_rgb.preview.link(xout_rgb.input)

    def process(self):
        in_rgb = self.q_rgb.get()
        in_detections = self.q_detections.get()

        frame = in_rgb.getCvFrame()
        detections = in_detections.detections

        if self.state == "PERSON_DETECTION":
            self.detect_person(frame, detections)
        elif self.state == "DRIVING":
            self.drive_to_person()
        elif self.state == "ARUCO_DETECTION":
            self.detect_aruco(frame)

        # Show the camera view
        cv2.imshow("Camera View", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def detect_person(self, frame, detections):
        for detection in detections:
            label = detection.label
            if label == 15:  # Label 15 corresponds to "person"
                z_mm = int(detection.spatialCoordinates.z)
                x_mm = int(detection.spatialCoordinates.x)  # Horizontal offset of the person
                self.person_publisher.publish(String(data=f"Person detected at Z: {z_mm}mm, X: {x_mm}mm"))
                self.get_logger().info(f"Person detected at Z: {z_mm}mm, X: {x_mm}mm")

                # Ignore persons farther than 6000mm
                if z_mm > 6000:
                    self.get_logger().info("Person is too far (> 6000mm). Staying in PERSON_DETECTION.")
                    return

                # Start driving if within distance_threshold
                if z_mm >= self.distance_threshold:
                    self.state = "DRIVING"
                    self.get_logger().info("Switching to DRIVING state.")
                    self.target_distance = z_mm  # Store the target's current distance
                    self.target_horizontal_offset = x_mm  # Store the horizontal position
                return
        # If no person is detected, stay in PERSON_DETECTION state
        self.get_logger().info("No person detected. Staying in PERSON_DETECTION.")

    def drive_to_person(self):
        # Retrieve detections for current frame
        in_detections = self.q_detections.get()
        detections = in_detections.detections

        for detection in detections:
            z_mm = int(detection.spatialCoordinates.z)
            x_mm = int(detection.spatialCoordinates.x)  # Horizontal offset

            # Stop if within stop_distance
            if z_mm <= self.stop_distance:
                self.twist_cmd.linear.x = 0.0  # Stop
                self.twist_cmd.angular.z = 0.0
                self.twist_publisher.publish(self.twist_cmd)
                self.state = "ARUCO_DETECTION"  # Transition to ARUCO_DETECTION
                self.get_logger().info("Reached the person. Switching to ARUCO_DETECTION.")
                return

            # Calculate dynamic speed based on distance
            linear_speed = max(0.1, min(0.5, 0.5 * (z_mm / 5000)))  # Scales between 0.1 and 0.5
            self.twist_cmd.linear.x = linear_speed

            # Calculate angular velocity based on horizontal position
            angular_velocity = 0.001 * x_mm  # Proportional steering (adjust scaling as needed)
            self.twist_cmd.angular.z = angular_velocity

            # Publish the command
            self.twist_publisher.publish(self.twist_cmd)
            self.get_logger().info(
                f"Driving towards person: distance={z_mm}mm, speed={linear_speed}, steering={angular_velocity}"
            )

    def detect_aruco(self, frame):
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            for marker_id in ids.flatten():
                if marker_id in self.wanted_markers:  # Check for "yes" marker
                    self.get_logger().info(f"Correct marker detected: {marker_id}")

                    # Send the "yes" command to the Arduino first
                    self.send_arduino_command("yes")
                    self.get_logger().info("Sent 'yes' command to Arduino.")

                    # Step 1: 90-degree turn and reverse
                    self.twist_cmd.linear.x = -0.6  # Reverse movement
                    self.twist_cmd.angular.z = 0.9  # Turn 90 degrees
                    self.twist_publisher.publish(self.twist_cmd)
                    self.get_logger().info(
                        f"Turning and reversing: linear.x={self.twist_cmd.linear.x}, angular.z={self.twist_cmd.angular.z}"
                    )
                    time.sleep(2)  # Duration for a 90-degree turn

                    # Step 2: Reverse for full 180-degree turn
                    self.twist_cmd.linear.x = 0.6  # Reverse movement
                    self.twist_cmd.angular.z = -0.9  # Turn in the opposite direction
                    self.twist_publisher.publish(self.twist_cmd)
                    self.get_logger().info(
                        f"Reversing and turning the other way: linear.x={self.twist_cmd.linear.x}, angular.z={self.twist_cmd.angular.z}"
                    )
                    time.sleep(2)  # Duration for the 180-degree turn

                    # Step 3: Move forward
                    self.twist_cmd.linear.x = 0.4  # Move forward
                    self.twist_cmd.angular.z = -0.1  # Straight movement
                    self.twist_publisher.publish(self.twist_cmd)
                    self.get_logger().info(
                        f"Moving forward: linear.x={self.twist_cmd.linear.x}, angular.z={self.twist_cmd.angular.z}"
                    )
                    time.sleep(2)  # Duration for moving forward

                    # Step 4: Stop and wait
                    self.twist_cmd.linear.x = 0.0
                    self.twist_cmd.angular.z = 0.0
                    self.twist_publisher.publish(self.twist_cmd)
                    self.get_logger().info("Stopped and waiting.")
                    time.sleep(3)  # Wait duration

                    # Transition back to looking for a person
                    self.state = "PERSON_DETECTION"
                    return
                else:
                    self.send_arduino_command("no")
                    self.get_logger().info(f"Incorrect marker detected: {marker_id}")
        else:
            self.send_arduino_command("waiting")
            self.get_logger().info("No markers detected. Staying in ARUCO_DETECTION.")
    
    def send_arduino_command(self, command):
        if self.arduino:
            self.arduino.write((command + '\n').encode())
            time.sleep(1)
        else:
            self.get_logger().warning("Arduino not connected.")


def main(args=None):
    rclpy.init(args=args)
    node = CameraControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
