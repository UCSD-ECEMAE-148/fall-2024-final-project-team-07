# import depthai as dai
# import cv2

# # Create pipeline
# pipeline = dai.Pipeline()

# # Define sources and outputs
# camRgb = pipeline.create(dai.node.ColorCamera)
# detectionNetwork = pipeline.create(dai.node.MobileNetDetectionNetwork)
# xoutRgb = pipeline.create(dai.node.XLinkOut)
# xoutNN = pipeline.create(dai.node.XLinkOut)

# xoutRgb.setStreamName("video")
# xoutNN.setStreamName("detections")

# # Properties
# camRgb.setPreviewSize(300, 300)
# camRgb.setInterleaved(False)
# camRgb.setFps(30)

# detectionNetwork.setConfidenceThreshold(0.5)  # Confidence threshold for detections
# detectionNetwork.setBlobPath(dai.OpenVINO.BlobPath("mobilenet-ssd_openvino_2021.4_6shave.blob"))
# detectionNetwork.setNumInferenceThreads(2)
# detectionNetwork.input.setBlocking(False)

# # Linking
# camRgb.preview.link(detectionNetwork.input)
# detectionNetwork.out.link(xoutNN.input)
# camRgb.preview.link(xoutRgb.input)

# # Connect to device and start pipeline
# with dai.Device(pipeline) as device:
#     # Output queues
#     videoQueue = device.getOutputQueue(name="video", maxSize=4, blocking=False)
#     detectionQueue = device.getOutputQueue(name="detections", maxSize=4, blocking=False)

#     while True:
#         videoFrame = videoQueue.get().getCvFrame()
#         detections = detectionQueue.get().detections

#         # Draw detections on the video frame
#         for detection in detections:
#             if detection.label == 15:  # 'Person' class in COCO dataset
#                 print("move")
#                 # Draw bounding box
#                 x1, y1, x2, y2 = int(detection.xmin * videoFrame.shape[1]), int(detection.ymin * videoFrame.shape[0]), \
#                                  int(detection.xmax * videoFrame.shape[1]), int(detection.ymax * videoFrame.shape[0])
#                 cv2.rectangle(videoFrame, (x1, y1), (x2, y2), (0, 255, 0), 2)
#                 cv2.putText(videoFrame, "Person Detected", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#         # Display video frame
#         cv2.imshow("Video", videoFrame)

#         if cv2.waitKey(1) == ord('q'):
#             break

# cv2.destroyAllWindows()


import rclpy
from rclpy.node import Node
import cv2
import depthai as dai
import numpy as np
from std_msgs.msg import String

class PersonDetectionNode(Node):
    def __init__(self):
        super().__init__('person_detection_node')
        self.get_logger().info("initializing Person Detection Node...")

        # publisher to send detected person details
        self.person_publisher = self.create_publisher(String, 'person_detection', 10)

        # DepthAI pipeline setup
        self.pipeline = dai.Pipeline()
        self.setup_pipeline()

        # running DepthAI device
        self.device = dai.Device(self.pipeline)
        self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.q_detections = self.device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        self.get_logger().info("DepthAI device initialized.")

        # label map for the MobileNet-SSD model
        self.label_map = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
                          "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

        # start processing loop
        self.create_timer(0.1, self.process)

    def setup_pipeline(self):
        """set up the DepthAI pipeline."""
        # define pipeline components
        cam_rgb = self.pipeline.createColorCamera()
        spatial_network = self.pipeline.createMobileNetSpatialDetectionNetwork()
        mono_left = self.pipeline.createMonoCamera()
        mono_right = self.pipeline.createMonoCamera()
        stereo = self.pipeline.createStereoDepth()

        xout_rgb = self.pipeline.createXLinkOut()
        xout_detections = self.pipeline.createXLinkOut()

        # set output stream names
        xout_rgb.setStreamName("rgb")
        xout_detections.setStreamName("detections")

        # camera properties
        cam_rgb.setPreviewSize(300, 300)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        # mono camera properties
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # stereo depth properties
        stereo.setOutputDepth(True)
        stereo.setConfidenceThreshold(255)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)

        # neural network properties
        spatial_network.setBlobPath("src/id_detection/id_detection/mobilenet-ssd_openvino_2021.4_6shave.blob")
        spatial_network.setConfidenceThreshold(0.5)
        spatial_network.setBoundingBoxScaleFactor(0.5)
        spatial_network.setDepthLowerThreshold(100)
        spatial_network.setDepthUpperThreshold(10000)

        # linking components
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        stereo.depth.link(spatial_network.inputDepth)
        cam_rgb.preview.link(spatial_network.input)

        spatial_network.out.link(xout_detections.input)
        cam_rgb.preview.link(xout_rgb.input)

    def process(self):
        """process frames and detect people."""
        in_rgb = self.q_rgb.get()
        in_detections = self.q_detections.get()

        frame = in_rgb.getCvFrame()
        detections = in_detections.detections

        for detection in detections:
            label = self.label_map[detection.label]
            if label == "person":
                # bounding box coordinates
                x1 = int(detection.xmin * frame.shape[1])
                y1 = int(detection.ymin * frame.shape[0])
                x2 = int(detection.xmax * frame.shape[1])
                y2 = int(detection.ymax * frame.shape[0])

                # spatial data
                z_mm = int(detection.spatialCoordinates.z)

                # draw bounding box and label
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), cv2.FONT_HERSHEY_SIMPLEX)

                # publish person detection data to topic
                detection_message = f"person detected at Z: {z_mm}mm"
                self.person_publisher.publish(String(data=detection_message))
                self.get_logger().info(detection_message)

        # Display the frame
        cv2.imshow("person", frame)

        # Exit when 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("shutting down Person Detection Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
