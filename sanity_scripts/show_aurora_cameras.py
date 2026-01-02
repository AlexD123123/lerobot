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

    try:
        while rclpy.ok():
            rclpy.spin_once(viewer, timeout_sec=0.01)

            # Prepare images for display
            images_to_show = []

            if viewer.rgb_image is not None:
                rgb_resized = cv2.resize(viewer.rgb_image, None, fx=3, fy=3, interpolation=cv2.INTER_LINEAR)
                # Add label
                cv2.putText(rgb_resized, 'RGB', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                images_to_show.append(rgb_resized)

            if viewer.depth_image is not None:
                # Normalize depth for visualization
                depth_normalized = cv2.normalize(viewer.depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)
                depth_resized = cv2.resize(depth_colored, None, fx=3, fy=3, interpolation=cv2.INTER_LINEAR)
                cv2.putText(depth_resized, 'Depth', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                images_to_show.append(depth_resized)

            if viewer.ir_image is not None:
                # Normalize IR for visualization
                ir_normalized = cv2.normalize(viewer.ir_image, None, 0, 255, cv2.NORM_MINMAX)
                ir_gray = ir_normalized.astype(np.uint8)
                ir_bgr = cv2.cvtColor(ir_gray, cv2.COLOR_GRAY2BGR)
                ir_resized = cv2.resize(ir_bgr, None, fx=3, fy=3, interpolation=cv2.INTER_LINEAR)
                cv2.putText(ir_resized, 'IR', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                images_to_show.append(ir_resized)

            # Combine images horizontally
            if len(images_to_show) > 0:
                combined = np.hstack(images_to_show)
                cv2.imshow('Aurora930 - RGB | Depth | IR (Press q to quit)', combined)

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