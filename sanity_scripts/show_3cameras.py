#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class Aurora930Viewer(Node):
    def __init__(self):
        super().__init__('aurora930_viewer')
        self.bridge = CvBridge()

        # Store latest images
        self.rgb_image = None
        self.depth_image = None
        self.ir_image = None

        # Create subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/rgb/image_raw',
            self.rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/depth/image_raw',
            self.depth_callback,
            10
        )

        self.ir_sub = self.create_subscription(
            Image,
            '/ir/image_raw',
            self.ir_callback,
            10
        )

        self.get_logger().info('Aurora930 Viewer started')

    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}')

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')

    def ir_callback(self, msg):
        try:
            self.ir_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'IR conversion error: {e}')


def main(args=None):
    rclpy.init(args=args)

    viewer = Aurora930Viewer()

    print("Press 'q' in the image window to quit")
    print("Receiving images from Aurora930...")

    # Open both cameras
    cap0 = cv2.VideoCapture(0)
    cap2 = cv2.VideoCapture(2)

    # Set resolution (optional)
    cap0.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap0.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    try:
        while rclpy.ok():
            rclpy.spin_once(viewer, timeout_sec=0.01)

            # Prepare images for display
            aurora_imgs = []

            if viewer.rgb_image is not None:
                rgb_resized = cv2.resize(viewer.rgb_image, [640, 480], fx=3, fy=3, interpolation=cv2.INTER_LINEAR)
                # Add label
                cv2.putText(rgb_resized, 'RGB_{Aurora}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                aurora_imgs.append(rgb_resized)
            else:
                aurora_imgs.append(np.zeros([480, 640, 3]))

            if viewer.depth_image is not None:
                # Normalize depth for visualization
                depth_normalized = cv2.normalize(viewer.depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
                depth_resized = cv2.resize(depth_colored, [640, 480], fx=3, fy=3, interpolation=cv2.INTER_LINEAR)
                cv2.putText(depth_resized, 'Depth_{Aurora}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                aurora_imgs.append(depth_resized)
            else:
                aurora_imgs.append(np.zeros([480, 640, 3]))

            if viewer.ir_image is not None:
                # Normalize IR for visualization
                ir_normalized = cv2.normalize(viewer.ir_image, None, 0, 255, cv2.NORM_MINMAX)
                ir_gray = ir_normalized.astype(np.uint8)
                ir_bgr = cv2.cvtColor(ir_gray, cv2.COLOR_GRAY2BGR)
                ir_resized = cv2.resize(ir_bgr, [640, 480], fx=3, fy=3, interpolation=cv2.INTER_LINEAR)
                cv2.putText(ir_resized, 'IR_{Aurora}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                aurora_imgs.append(ir_resized)
            else:
                aurora_imgs.append(np.zeros([480, 640, 3]))

            ret0, frame0 = cap0.read()
            ret2, frame2 = cap2.read()

            height = 480
            width = 640
            #frame0_resized = cv2.resize(frame0, (int(frame0.shape[1] * height / frame0.shape[0]), height))
            frame0_resized = cv2.resize(frame0, (width, height))
            cv2.putText(frame0_resized, 'RGB-Wide', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            #frame2_resized = cv2.resize(frame2, (int(frame2.shape[1] * height / frame2.shape[0]), height))
            frame2_resized = cv2.resize(frame2, (width, height))
            cv2.putText(frame2_resized, 'RGB-Narrow', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            frame_stub = 0*frame2_resized
            # Combine side by side
            combined_rgb = np.hstack([frame0_resized, frame2_resized, frame_stub])

            # Combine images horizontally
            if len(aurora_imgs) > 0:
                combined_aurora = np.hstack(aurora_imgs)
                combined = np.vstack([combined_aurora, combined_rgb])
                cv2.imshow('Aurora930 - RGB | Depth | IR | RGB camera wide | RGB camera narrow | (Press q to quit)', combined)

            # Check for 'q' key to quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                print("Quitting...")
                break

    except KeyboardInterrupt:
        print("Interrupted by user")

    finally:
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()