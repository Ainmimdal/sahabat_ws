#!/usr/bin/env python3
"""Show the ZED left image with detected AprilTags drawn on it.

Uses only Tk + numpy (no OpenCV / cv_bridge), so it works even when a pip
NumPy 2.x shadows the NumPy 1.x that the system cv2 was compiled against.

Green outline = good quality (usable for localization), orange = weak.
Prints a status line every second. Close the window (or press q) to quit.
"""

import math
import time
import tkinter as tk

import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformException, TransformListener

IMAGE_TOPIC = '/zed/zed_node/left/image_rect_color'
GOOD_MARGIN = 30.0  # same threshold the landmark manager uses
MAX_WIDTH = 960     # downscale wide images for a smooth display


def image_to_rgb(message):
    """Convert a sensor_msgs/Image (bgra8/bgr8/rgb8/rgba8/mono8) to RGB uint8."""
    data = np.frombuffer(message.data, dtype=np.uint8)
    encoding = message.encoding.lower()
    channels = {'bgra8': 4, 'rgba8': 4, 'bgr8': 3, 'rgb8': 3, 'mono8': 1}.get(encoding)
    if channels is None:
        raise ValueError(f'unsupported encoding {message.encoding}')
    rows = data.reshape(message.height, message.step)[:, :message.width * channels]
    pixels = rows.reshape(message.height, message.width, channels)
    if encoding in ('bgra8', 'bgr8'):
        return np.ascontiguousarray(pixels[:, :, 2::-1])
    if encoding in ('rgba8', 'rgb8'):
        return np.ascontiguousarray(pixels[:, :, :3])
    return np.ascontiguousarray(np.repeat(pixels, 3, axis=2))


class AprilTagViewer(Node):
    def __init__(self):
        super().__init__('apriltag_viewer')
        self.rgb = None
        self.detections = []
        self.detections_at = 0.0
        self.image_count = 0
        self.detection_msgs = 0
        self.started = time.monotonic()
        self.last_print = 0.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(Image, IMAGE_TOPIC, self.on_image, qos_profile_sensor_data)
        self.create_subscription(
            AprilTagDetectionArray, '/apriltag/detections', self.on_detections,
            qos_profile_sensor_data,
        )

    def on_image(self, message):
        try:
            self.rgb = image_to_rgb(message)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        self.image_count += 1

    def on_detections(self, message):
        self.detection_msgs += 1
        self.detections = []
        for detection in message.detections:
            distance = None
            try:
                transform = self.tf_buffer.lookup_transform(
                    message.header.frame_id,
                    f'{detection.family}:{detection.id}',
                    Time(), timeout=Duration(seconds=0.0),
                )
                t = transform.transform.translation
                distance = math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)
            except TransformException:
                pass
            self.detections.append((detection, distance))
        self.detections_at = time.monotonic()

    def current_tags(self):
        if time.monotonic() - self.detections_at > 0.5:
            return []
        return self.detections


class ViewerWindow:
    def __init__(self, node):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Sahabat AprilTag test (q to quit)')
        self.canvas = tk.Canvas(self.root, width=640, height=360, bg='black',
                                highlightthickness=0)
        self.canvas.pack()
        self.photo = None
        self.running = True
        self.root.bind('q', lambda _event: self.stop())
        self.root.protocol('WM_DELETE_WINDOW', self.stop)
        self.root.after(10, self.tick)

    def stop(self):
        self.running = False
        self.root.quit()

    def tick(self):
        if not self.running:
            return
        rclpy.spin_once(self.node, timeout_sec=0.0)
        for _ in range(10):  # drain queued callbacks
            rclpy.spin_once(self.node, timeout_sec=0.0)
        self.draw()
        self.root.after(30, self.tick)

    def draw(self):
        node = self.node
        self.canvas.delete('all')
        lines = []
        if node.rgb is None:
            waited = time.monotonic() - node.started
            self.canvas.config(width=640, height=360)
            self.canvas.create_text(320, 160, fill='white', font=('Sans', 16),
                                    text=f'Waiting for ZED images... {waited:.0f} s')
            self.canvas.create_text(320, 200, fill='gray', text=IMAGE_TOPIC)
        else:
            rgb = node.rgb
            step = max(1, int(math.ceil(rgb.shape[1] / MAX_WIDTH)))
            shown = rgb[::step, ::step]
            height, width = shown.shape[:2]
            header = f'P6 {width} {height} 255 '.encode()
            self.photo = tk.PhotoImage(data=header + shown.tobytes(), format='PPM')
            self.canvas.config(width=width, height=height)
            self.canvas.create_image(0, 0, image=self.photo, anchor='nw')
            for detection, distance in node.current_tags():
                good = detection.hamming == 0 and detection.decision_margin >= GOOD_MARGIN
                color = '#00c800' if good else '#ff8c00'
                points = []
                for corner in list(detection.corners) + [detection.corners[0]]:
                    points.extend((corner.x / step, corner.y / step))
                self.canvas.create_line(*points, fill=color, width=3)
                label = f'ID {detection.id}'
                if distance is not None:
                    label += f'  {distance:.2f} m'
                self.canvas.create_text(detection.centre.x / step,
                                        detection.centre.y / step - 18,
                                        text=label, fill=color, font=('Sans', 14, 'bold'))
                lines.append(
                    f'ID {detection.id} ({detection.family}) '
                    f'margin {detection.decision_margin:.0f} hamming {detection.hamming} '
                    f'distance {"?" if distance is None else f"{distance:.2f} m"} '
                    f'{"GOOD" if good else "WEAK"}'
                )
            self.canvas.create_text(10, 15, anchor='w', fill='white', font=('Sans', 12),
                                    text=f'{len(lines)} tag(s) | images {node.image_count}')
        now = time.monotonic()
        if now - node.last_print >= 1.0:
            node.last_print = now
            print(
                f'[viewer] images={node.image_count} detection_msgs={node.detection_msgs} | '
                + ('  |  '.join(lines) if lines else 'no tags in view'),
                flush=True,
            )


def main():
    rclpy.init()
    node = AprilTagViewer()
    window = ViewerWindow(node)
    try:
        window.root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
