from __future__ import annotations

import json
import sys
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from PyQt5 import QtCore, QtWidgets


@dataclass
class TelemetrySnapshot:
    gps_lat: Optional[float] = None
    gps_lon: Optional[float] = None
    gps_fix: Optional[int] = None
    battery_v: Optional[float] = None
    battery_a: Optional[float] = None
    odom_distance: Optional[float] = None
    system_status: Optional[int] = None
    fire_status: Dict[str, Any] = field(default_factory=dict)


class OperatorNode(Node):
    def __init__(self) -> None:
        super().__init__("operator_gui")

        self.telemetry = TelemetrySnapshot()
        self._last_cmd = Twist()

        # ROS publishers
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.camera_select_pub = self.create_publisher(String, "/camera_select", 10)
        self.mission_pub = self.create_publisher(String, "/mission_command", 10)

        # ROS subscriptions
        self.create_subscription(String, "/lora/telemetry_in", self._on_telemetry, 10)

    def _on_telemetry(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse telemetry JSON: {exc}")
            return

        ts = TelemetrySnapshot()
        if "gps_lat" in data:
            ts.gps_lat = float(data["gps_lat"])
        if "gps_lon" in data:
            ts.gps_lon = float(data["gps_lon"])
        if "gps_fix" in data:
            ts.gps_fix = int(data["gps_fix"])
        if "battery_v" in data:
            ts.battery_v = float(data["battery_v"])
        if "battery_a" in data:
            ts.battery_a = float(data["battery_a"])
        if "odom_distance" in data:
            ts.odom_distance = float(data["odom_distance"])
        if "system_status" in data:
            ts.system_status = int(data["system_status"])

        fire = data.get("fire_status")
        if isinstance(fire, dict):
            ts.fire_status = fire

        self.telemetry = ts

    # Convenience methods used by the GUI widget ------------------------

    def send_teleop(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
        self._last_cmd = msg

    def send_camera_select(self, name: str) -> None:
        self.camera_select_pub.publish(String(data=name))

    def send_mission(self, command: str) -> None:
        # Simple string protocol for now: "manual", "rtb", "waypoint:N", "loiter", "estop".
        self.mission_pub.publish(String(data=command))


class OperatorWindow(QtWidgets.QWidget):
    def __init__(self, node: OperatorNode) -> None:
        super().__init__()
        self._node = node
        self.setWindowTitle("QWACR Operator GUI")
        self._build_ui()

        # GStreamer receiver process for embedded video
        self._gst_process = QtCore.QProcess(self)
        self._gst_process.errorOccurred.connect(self._on_gst_error)

        # Periodic refresh of telemetry display
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self._refresh)
        timer.start(200)

    def _build_ui(self) -> None:
        layout = QtWidgets.QHBoxLayout(self)

        # Left: video placeholder
        video_group = QtWidgets.QGroupBox("Video")
        video_layout = QtWidgets.QVBoxLayout(video_group)
        self.video_label = QtWidgets.QLabel(
            "Video feed is provided by the GStreamer receiver.\n"
            "Run the RTP client on the base-station Pi and place its window next to this GUI."
        )
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)
        video_layout.addWidget(self.video_label)

        # Right: controls + telemetry
        right = QtWidgets.QVBoxLayout()

        # Telemetry
        telem_group = QtWidgets.QGroupBox("Telemetry (LoRa)")
        telem_layout = QtWidgets.QFormLayout(telem_group)
        self.gps_label = QtWidgets.QLabel("---")
        self.batt_label = QtWidgets.QLabel("---")
        self.odom_label = QtWidgets.QLabel("---")
        self.fire_label = QtWidgets.QLabel("(no fire_status yet)")
        telem_layout.addRow("GPS:", self.gps_label)
        telem_layout.addRow("Battery:", self.batt_label)
        telem_layout.addRow("Odom:", self.odom_label)
        telem_layout.addRow("Fire:", self.fire_label)

        # Teleop buttons
        teleop_group = QtWidgets.QGroupBox("Teleop")
        tgrid = QtWidgets.QGridLayout(teleop_group)
        # Include primary keyboard shortcuts in labels (WASDX)
        btn_fwd = QtWidgets.QPushButton("Forward (W)")
        btn_back = QtWidgets.QPushButton("Back (X)")
        btn_left = QtWidgets.QPushButton("Left (A)")
        btn_right = QtWidgets.QPushButton("Right (D)")
        btn_stop = QtWidgets.QPushButton("Stop (S)")
        tgrid.addWidget(btn_fwd, 0, 1)
        tgrid.addWidget(btn_left, 1, 0)
        tgrid.addWidget(btn_stop, 1, 1)
        tgrid.addWidget(btn_right, 1, 2)
        tgrid.addWidget(btn_back, 2, 1)

        btn_fwd.clicked.connect(lambda: self._node.send_teleop(0.4, 0.0))
        btn_back.clicked.connect(lambda: self._node.send_teleop(-0.4, 0.0))
        btn_left.clicked.connect(lambda: self._node.send_teleop(0.0, 1.2))
        btn_right.clicked.connect(lambda: self._node.send_teleop(0.0, -1.2))
        btn_stop.clicked.connect(lambda: self._node.send_teleop(0.0, 0.0))

        # Camera select
        cam_group = QtWidgets.QGroupBox("Camera")
        cam_layout = QtWidgets.QHBoxLayout(cam_group)
        # Include keyboard hints in button labels
        btn_front = QtWidgets.QPushButton("Front (1)")
        btn_left_cam = QtWidgets.QPushButton("Left (2)")
        cam_layout.addWidget(btn_front)
        cam_layout.addWidget(btn_left_cam)
        btn_front.clicked.connect(lambda: self._node.send_camera_select("front"))
        btn_left_cam.clicked.connect(lambda: self._node.send_camera_select("left"))

        # Mission modes
        mission_group = QtWidgets.QGroupBox("Mission Mode")
        m_layout = QtWidgets.QVBoxLayout(mission_group)
        btn_manual = QtWidgets.QPushButton("Manual Teleop")
        btn_waypoint = QtWidgets.QPushButton("Waypoint (existing)")
        btn_rtb = QtWidgets.QPushButton("Return to Base")
        btn_loiter = QtWidgets.QPushButton("Loiter")
        btn_estop = QtWidgets.QPushButton("E-STOP")
        m_layout.addWidget(btn_manual)
        m_layout.addWidget(btn_waypoint)
        m_layout.addWidget(btn_rtb)
        m_layout.addWidget(btn_loiter)
        m_layout.addWidget(btn_estop)

        btn_manual.clicked.connect(lambda: self._node.send_mission("manual"))
        # Waypoint selection can be refined later; for now send a generic command
        btn_waypoint.clicked.connect(lambda: self._node.send_mission("waypoint:default"))
        btn_rtb.clicked.connect(lambda: self._node.send_mission("rtb"))
        btn_loiter.clicked.connect(lambda: self._node.send_mission("loiter"))
        btn_estop.clicked.connect(lambda: self._node.send_mission("estop"))

        right.addWidget(telem_group)
        right.addWidget(teleop_group)
        right.addWidget(cam_group)
        right.addWidget(mission_group)
        right.addStretch(1)

        layout.addWidget(video_group, stretch=2)
        layout.addLayout(right, stretch=3)

    def start_video_receiver(self, port: int = 5000) -> None:
        """Start a GStreamer UDP H.264 receiver into the video panel."""

        # Avoid starting multiple pipelines
        if self._gst_process.state() != QtCore.QProcess.NotRunning:
            return

        # Native window ID for ximagesink
        xid = int(self.video_label.winId())

        # Give the operator visual feedback even before frames arrive
        self.video_label.setText(f"Waiting for video on UDP port {port}…")

        pipeline_args = [
            "udpsrc",
            f"port={port}",
            "caps=application/x-rtp,media=video,encoding-name=H264,payload=96",
            "!",
            "rtph264depay",
            "!",
            "avdec_h264",
            "!",
            "videoconvert",
            "!",
            "ximagesink",
            "sync=false",
            f"xid={xid}",
        ]

        self._gst_process.start("gst-launch-1.0", pipeline_args)

    def _on_gst_error(self, error: QtCore.QProcess.ProcessError) -> None:  # type: ignore[override]
        # Fall back to a helpful message if the receiver cannot start.
        if self.video_label is not None:
            self.video_label.setText(
                "Failed to start embedded video receiver.\n"
                "Check that GStreamer (gst-launch-1.0) is installed and the HaLow stream is running."
            )

    def _refresh(self) -> None:
        t = self._node.telemetry
        if t.gps_lat is not None and t.gps_lon is not None:
            self.gps_label.setText(f"{t.gps_lat:.6f}, {t.gps_lon:.6f} (fix {t.gps_fix})")
        else:
            self.gps_label.setText("---")

        if t.battery_v is not None:
            self.batt_label.setText(f"{t.battery_v:.2f} V, {t.battery_a or 0.0:.2f} A")
        else:
            self.batt_label.setText("---")

        if t.odom_distance is not None:
            self.odom_label.setText(f"{t.odom_distance:.1f} m")
        else:
            self.odom_label.setText("---")

        if t.fire_status:
            # Compact one-line summary until we define a proper schema
            parts = [f"{k}={v}" for k, v in t.fire_status.items()]
            self.fire_label.setText(", ".join(parts))
        else:
            self.fire_label.setText("(no fire_status yet)")

    # Optional: basic keyboard teleop via WASD when window focused
    def keyPressEvent(self, event: QtWidgets.QKeyEvent) -> None:  # type: ignore[override]
        key = event.key()
        # Forward (W)
        if key == QtCore.Qt.Key_W:
            self._node.send_teleop(0.4, 0.0)
        # Stop (S)
        elif key == QtCore.Qt.Key_S:
            self._node.send_teleop(0.0, 0.0)
        # Turn left (A)
        elif key == QtCore.Qt.Key_A:
            self._node.send_teleop(0.0, 1.2)
        # Turn right (D)
        elif key == QtCore.Qt.Key_D:
            self._node.send_teleop(0.0, -1.2)
        # Backward (X)
        elif key == QtCore.Qt.Key_X:
            self._node.send_teleop(-0.4, 0.0)
        # Camera select shortcuts
        elif key == QtCore.Qt.Key_1:
            self._node.send_camera_select("front")
        elif key == QtCore.Qt.Key_2:
            self._node.send_camera_select("left")
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        # Ensure the GStreamer process is terminated when the window closes.
        if self._gst_process.state() != QtCore.QProcess.NotRunning:
            self._gst_process.terminate()
            self._gst_process.waitForFinished(2000)
        super().closeEvent(event)


def main() -> None:
    rclpy.init()
    node = OperatorNode()

    app = QtWidgets.QApplication(sys.argv)

    # Use a QTimer to integrate rclpy with the Qt event loop
    ros_timer = QtCore.QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    ros_timer.start(10)

    win = OperatorWindow(node)
    win.resize(1200, 700)
    win.show()

    # Start the embedded GStreamer video receiver once the window exists.
    QtCore.QTimer.singleShot(0, win.start_video_receiver)

    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
