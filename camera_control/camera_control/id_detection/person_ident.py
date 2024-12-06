import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

class PersonDetectionNode(Node):
    def __init__(self):
        super().__init__('person_detection_node')
        self.get_logger().info("Initializing Person Detection Node...")

        # Publisher to send detected person details
        self.person_publisher = self.create_publisher(String, 'person_detection', 10)

        # Subscriber for camera frames
        self.camera_subscriber = self.create_subscription(Image, 'camera_frames', self.process_frame, 10)
        self.bridge = CvBridge()

        # Label map for the MobileNet-SSD model
        self.label_map = [
            "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
        ]

    def process_frame(self, msg):
        """Process incoming camera frames."""
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Mock detection logic since DepthAI pipeline is removed
        # Replace this with your actual detection model or logic
        detections = self.mock_detections(frame)

        for detection in detections:
            label = detection['label']
            if label == "person":
                x1, y1, x2, y2 = detection['bbox']
                z_mm = detection['depth']

                # Draw bounding box and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, f"Person Detected: {z_mm}mm", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                # Publish person detection data to topic
                detection_message = f"Person detected at Z: {z_mm}mm"
                self.person_publisher.publish(String(data=detection_message))
                self.get_logger().info(detection_message)

        # Display the frame
        cv2.imshow("Person Detection", frame)

        # Exit when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def mock_detections(self, frame):
        """Mock detections for demonstration. Replace with real detection logic."""
        # This is just an example. Replace it with your actual detection pipeline.
        h, w, _ = frame.shape
        return [{
            "label": "person",
            "bbox": [int(w * 0.3), int(h * 0.3), int(w * 0.7), int(h * 0.7)],  # Example bounding box
            "depth": 2000  # Example depth value in mm
        }]

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Person Detection Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
