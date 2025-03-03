#!/usr/bin/env python3

import cv2
import numpy as np
import json
import asyncio
import websockets
import base64
from io import BytesIO
from PIL import Image as PILImage
import argparse
import uuid  # For creating subscription IDs
import time

class ArUcoDetector:
    def __init__(self, websocket_url, camera_topic, use_compressed=True):
        self.websocket_url = websocket_url
        self.base_topic = camera_topic
        
        # Use the compressed topic if requested
        if use_compressed and not camera_topic.endswith('/compressed'):
            self.camera_topic = f"{camera_topic}/compressed"
        else:
            self.camera_topic = camera_topic
            
        # Print OpenCV version for debugging
        self.opencv_version = cv2.__version__
        print(f"Using OpenCV version: {self.opencv_version}")
        
        # Set up ArUco detector for version 4.10.0
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # Display window name
        self.window_name = "ArUco Tag Detection (Foxglove)"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        
        # Store available topics
        self.available_topics = []
        
        print(f"ArUco detector initialized. Will connect to: {websocket_url}")
        print(f"Looking for topic: {self.camera_topic}")

    async def wait_for_topics(self, websocket):
        """Wait for the server to advertise topics"""
        print("Waiting for topic advertisements...")
        
        try:
            start_time = time.time()
            while time.time() - start_time < 5:  # Wait up to 5 seconds
                response = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                data = json.loads(response)
                
                # Check if this is an advertise message
                if data.get("op") == "advertise":
                    topic = data.get("topic")
                    if topic:
                        print(f"Topic advertised: {topic}")
                        self.available_topics.append(topic)
                        
                        # If our target topic is advertised, we can proceed
                        if topic == self.camera_topic:
                            print(f"Found our target topic: {self.camera_topic}")
                            return True
                
                # Check for other useful messages
                if data.get("op") == "topics":
                    topics = data.get("topics", [])
                    for topic_info in topics:
                        topic_name = topic_info.get("name")
                        if topic_name:
                            print(f"Topic available: {topic_name}")
                            self.available_topics.append(topic_name)
                            
                            # If our target topic is in the list, we can proceed
                            if topic_name == self.camera_topic:
                                print(f"Found our target topic: {self.camera_topic}")
                                return True
                
                await asyncio.sleep(0.1)
            
            # Check if our topic was found
            if self.camera_topic in self.available_topics:
                return True
                
            print(f"Target topic {self.camera_topic} not found in available topics:")
            for topic in self.available_topics:
                print(f"  - {topic}")
                
            # Suggest alternative topics if any were found
            if self.available_topics:
                print("\nAvailable topics you could try:")
                for topic in self.available_topics:
                    if "image" in topic or "camera" in topic:
                        print(f"  --topic {topic}")
            
            return False
            
        except asyncio.TimeoutError:
            print("Timeout waiting for topic advertisements")
            return False
        except Exception as e:
            print(f"Error while waiting for topics: {e}")
            return False

    async def subscribe_to_topic(self, websocket):
        """Subscribe to a topic"""
        # Generate a unique subscription ID
        subscription_id = str(uuid.uuid4())
        
        # Make sure we have the topic name available
        if not self.available_topics and self.camera_topic not in self.available_topics:
            # Request the list of topics
            await websocket.send(json.dumps({"op": "getTopics"}))
            
            # Wait a bit for the response
            await asyncio.sleep(0.5)
        
        # Create a subscription message
        subscribe_msg = {
            "op": "subscribe",
            "id": subscription_id,
            "topic": self.camera_topic
        }
        
        # Send the subscription request
        await websocket.send(json.dumps(subscribe_msg))
        print(f"Sent subscription request for topic: {self.camera_topic} with ID: {subscription_id}")
        
        # Don't wait for a specific confirmation - this seems unreliable
        # Just proceed and let the message handler deal with incoming messages
        return subscription_id

    def process_image(self, cv_image):
        """Process an image with ArUco detector"""
        # Make a copy of the image for drawing on
        output_image = cv_image.copy()
        
        # Convert to grayscale for ArUco detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers using the ArucoDetector in OpenCV 4.10.0
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        # If at least one marker is detected
        if ids is not None:
            # Draw detected markers and their IDs
            cv2.aruco.drawDetectedMarkers(output_image, corners, ids)
            
            # Process each detected marker
            for i, marker_corners in enumerate(corners):
                # Get the four corners of the marker
                marker_corners = marker_corners.reshape((4, 2))
                marker_corners = marker_corners.astype(int)
                
                # Calculate bounding box
                x_min = min(marker_corners[:, 0])
                y_min = min(marker_corners[:, 1])
                x_max = max(marker_corners[:, 0])
                y_max = max(marker_corners[:, 1])
                
                # Draw bounding box (rectangle)
                cv2.rectangle(output_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                
                # Add ID text above the marker
                cv2.putText(output_image, f"ID: {ids[i][0]}", (x_min, y_min - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Print detection information
            print(f"Detected {len(ids)} ArUco markers. IDs: {ids.flatten()}")
        
        # Display the image with detected markers
        cv2.imshow(self.window_name, output_image)
        key = cv2.waitKey(1)
        
        # Press 'q' to quit
        if key == ord('q'):
            return False
        return True

    async def connect_and_process(self):
        """Main method to connect and process messages"""
        try:
            # Connect with the required subprotocol
            async with websockets.connect(
                self.websocket_url,
                subprotocols=["foxglove.websocket.v1"],
                ping_interval=None,  # Disable ping to avoid timeouts
                ping_timeout=None    # Disable ping timeout
            ) as websocket:
                print(f"Connected to {self.websocket_url}")
                
                # Request the list of topics
                await websocket.send(json.dumps({"op": "getTopics"}))
                
                # Wait for topic advertisements
                if not await self.wait_for_topics(websocket):
                    print("Could not find the requested topic. Please check the topic name.")
                    return
                
                # Subscribe to our camera topic
                subscription_id = await self.subscribe_to_topic(websocket)
                
                # Process incoming messages
                while True:
                    try:
                        message = await websocket.recv()
                        data = json.loads(message)
                        
                        # Handle different message types
                        op = data.get("op")
                        
                        if op == "publish":
                            # This is a publish message with image data
                            topic = data.get("topic")
                            
                            if topic == self.camera_topic:
                                msg = data.get("msg", {})
                                
                                # Handle different message formats
                                if self.camera_topic.endswith('/compressed'):
                                    # Compressed image
                                    if "format" in msg and "data" in msg:
                                        try:
                                            # Decode the base64 image
                                            img_data = base64.b64decode(msg["data"])
                                            
                                            # Use PIL to open the image data
                                            img = PILImage.open(BytesIO(img_data))
                                            img_np = np.array(img)
                                            
                                            # Convert RGB to BGR for OpenCV
                                            img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
                                            
                                            if not self.process_image(img_np):
                                                return
                                        except Exception as e:
                                            print(f"Error processing compressed image: {e}")
                                else:
                                    # Raw image
                                    if "encoding" in msg and "data" in msg and "height" in msg and "width" in msg:
                                        try:
                                            # Decode the base64 image data
                                            img_data = base64.b64decode(msg["data"])
                                            
                                            # Convert to numpy array
                                            img_np = np.frombuffer(img_data, dtype=np.uint8)
                                            
                                            # Reshape to image dimensions
                                            img_np = img_np.reshape((msg["height"], msg["width"], 3))
                                            
                                            # Convert to BGR if needed
                                            if msg["encoding"] == "rgb8":
                                                img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
                                                
                                            if not self.process_image(img_np):
                                                return
                                        except Exception as e:
                                            print(f"Error processing raw image: {e}")
                        
                        elif op == "status":
                            # Status messages about our subscription
                            status_id = data.get("id")
                            status = data.get("status")
                            
                            if status_id == subscription_id:
                                if status == "error":
                                    print(f"Subscription error: {data.get('message', 'Unknown error')}")
                                    # Try to resubscribe
                                    subscription_id = await self.subscribe_to_topic(websocket)
                            
                    except asyncio.CancelledError:
                        break
                    except json.JSONDecodeError:
                        print("Error decoding message")
                    except Exception as e:
                        print(f"Error processing message: {e}")
                        continue
                        
        except websockets.exceptions.ConnectionClosed as e:
            print(f"Connection closed: {e}")
        except Exception as e:
            print(f"Error: {type(e).__name__}: {e}")

    def run(self):
        """Run the detector"""
        try:
            # Start the asyncio event loop
            asyncio.get_event_loop().run_until_complete(self.connect_and_process())
        except KeyboardInterrupt:
            print("Shutting down ArUco detector")
        finally:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ArUco tag detector using Foxglove WebSocket')
    parser.add_argument('--url', type=str, default="ws://10.49.15.204:8775",
                        help='Foxglove WebSocket server URL')
    parser.add_argument('--topic', type=str, default="/camera/image_raw",
                        help='Camera topic to subscribe to')
    parser.add_argument('--compressed', action='store_true',
                        help='Use compressed image topic (adds /compressed to topic name)')
    
    args = parser.parse_args()
    
    # Create and run the detector
    detector = ArUcoDetector(args.url, args.topic, args.compressed)
    detector.run()