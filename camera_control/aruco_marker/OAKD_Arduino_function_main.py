import cv2
import depthai as dai
import numpy as np
import serial
import time

NODE_NAME = 'aruco_marker_node'

def main():
    # Set up ArUco dictionary and parameters
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)  # Choose the type you need
    aruco_params = cv2.aruco.DetectorParameters_create()
    wanted_markers = [2, 9, 16]

    # Start defining a pipeline
    pipeline = dai.Pipeline()

    # Define a source - color camera
    cam_rgb = pipeline.createColorCamera()
    cam_rgb.setPreviewSize(640, 480)
    cam_rgb.setInterleaved(False)
    cam_rgb.setFps(30)

    # Create output for the color camera
    xout_rgb = pipeline.createXLinkOut()
    xout_rgb.setStreamName("rgb")
    cam_rgb.preview.link(xout_rgb.input)

    # Arduino connection initialized
    # Replace '/dev/ttyUSB0' with the correct port for your Arduino
    arduino = serial.Serial('/dev/ttyUSB0', 9600)
    time.sleep(2)  # Wait for the connection to initialize

    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:
        # Output queue will be used to get rgb frames
        q_rgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        while True:
            in_rgb = q_rgb.get()  # Get the latest frame
            frame = in_rgb.getCvFrame()  # Convert to OpenCV format

            # Detect ArUco markers in the frame
            corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

            # If markers are detected
            if ids is not None:
                # Draw detected markers
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for i, corner in enumerate(corners):
                    # Print the marker ID for each detected marker
                    print(f"Detected marker ID: {ids[i][0]}")
                #print(ids)
                if set([i[0] for i in ids]).issubset(set(wanted_markers)):
                    #exec(open('LCD.py').read())

                    # Send yes = green screen to arduino when correct aruco markers are detected
                    commands = ["yes"]
                    for command in commands:
                        arduino.write((command + '\n').encode())  # Send command with newline
                        print(f"Sent: {command}")
                        time.sleep(2)  # Wait before sending the next command
                    break

            #commands = ["yes", "no", "waiting"]
            # Send command no = red screen to arduino when wrong aruco markers are spotted
                commands = ["no"]
                for command in commands:
                    arduino.write((command + '\n').encode())  # Send command with newline
                    print(f"Sent: {command}")
                    time.sleep(2)  # Wait before sending the next command

            # Display the frame with markers highlighted
            else:
                # Send waiting = no color to the arduino when no aruco markers are detected at all
                # commands = ["yes", "no", "waiting"]
                commands = ["waiting"]
                for command in commands:
                    arduino.write((command + '\n').encode())  # Send command with newline
                    print(f"Sent: {command}")
                    time.sleep(2)  # Wait before sending the next command

                # Display the frame with markers highlighted
                cv2.imshow("ArUco Marker Detection", frame)

            cv2.imshow("ArUco Marker Detection", frame)

            # Exit when 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

