"""Publish the Unitree built-in front camera to the web monitor."""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from unitree_api.msg import Request, Response


VIDEOHUB_GET_IMAGE_SAMPLE = 1001


class UnitreeCameraRelay(Node):
    """Request JPEG frames from videohub and forward them on ROS 2."""

    def __init__(self) -> None:
        super().__init__("unitree_camera_relay")

        self.declare_parameter("request_topic", "/api/videohub/request")
        self.declare_parameter("response_topic", "/api/videohub/response")
        self.declare_parameter(
            "output_topic", "/application_web_monitor/image/compressed")
        self.declare_parameter("frame_id", "unitree_front_camera")
        self.declare_parameter("frame_rate_hz", 10.0)
        self.declare_parameter("request_timeout_sec", 2.0)
        self.declare_parameter("yield_to_other_publishers", True)

        request_topic = str(self.get_parameter("request_topic").value)
        response_topic = str(self.get_parameter("response_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        frame_rate = max(
            0.2, float(self.get_parameter("frame_rate_hz").value))
        self.request_timeout = max(
            0.2, float(self.get_parameter("request_timeout_sec").value))
        self.yield_to_other_publishers = bool(
            self.get_parameter("yield_to_other_publishers").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.request_pub = self.create_publisher(Request, request_topic, qos)
        self.image_pub = self.create_publisher(
            CompressedImage, self.output_topic, qos)
        self.response_sub = self.create_subscription(
            Response, response_topic, self._on_response, qos)

        self.pending_id = None
        self.pending_since = 0.0
        self.frames = 0
        self.last_error_log = 0.0
        self.timer = self.create_timer(1.0 / frame_rate, self._request_frame)
        self.get_logger().info(
            f"Unitree camera relay: {request_topic} -> {self.output_topic} "
            f"at up to {frame_rate:.1f} Hz")

    def _another_output_publisher_exists(self) -> bool:
        return (
            self.yield_to_other_publishers
            and self.count_publishers(self.output_topic) > 1
        )

    def _request_frame(self) -> None:
        if self._another_output_publisher_exists():
            return

        now = time.monotonic()
        if self.pending_id is not None:
            if now - self.pending_since <= self.request_timeout:
                return
            self._log_error("videohub image request timed out")
            self.pending_id = None

        request_id = time.monotonic_ns()
        request = Request()
        request.header.identity.id = request_id
        request.header.identity.api_id = VIDEOHUB_GET_IMAGE_SAMPLE
        request.header.lease.id = 0
        request.header.policy.priority = 0
        request.header.policy.noreply = False
        request.parameter = ""
        request.binary = []
        self.pending_id = request_id
        self.pending_since = now
        self.request_pub.publish(request)

    @staticmethod
    def _binary_bytes(binary) -> bytes:
        try:
            return memoryview(binary).cast("B").tobytes()
        except (TypeError, ValueError):
            return bytes(value & 0xFF for value in binary)

    def _on_response(self, response: Response) -> None:
        if response.header.identity.api_id != VIDEOHUB_GET_IMAGE_SAMPLE:
            return
        if self.pending_id is None or (
                response.header.identity.id != self.pending_id):
            return

        self.pending_id = None
        if self._another_output_publisher_exists():
            return
        if response.header.status.code != 0:
            self._log_error(
                f"videohub returned code {response.header.status.code}")
            return

        jpeg = self._binary_bytes(response.binary)
        if len(jpeg) < 4 or not jpeg.startswith(b"\xff\xd8"):
            self._log_error(
                f"videohub returned invalid JPEG data ({len(jpeg)} bytes)")
            return

        image = CompressedImage()
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = self.frame_id
        image.format = "jpeg"
        image.data = jpeg
        self.image_pub.publish(image)

        self.frames += 1
        if self.frames == 1:
            self.get_logger().info(
                f"First Unitree camera frame published ({len(jpeg)} bytes)")

    def _log_error(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_error_log >= 5.0:
            self.get_logger().warning(message)
            self.last_error_log = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UnitreeCameraRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
