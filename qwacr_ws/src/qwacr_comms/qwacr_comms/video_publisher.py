import os
import signal
import subprocess
from typing import List, Optional

import rclpy
from rclpy.node import Node


class VideoPublisher(Node):
    """Manage rpicam-vid → GStreamer pipelines to stream cameras over HaLow.

    This node does not currently publish ROS image topics; it simply
    spawns and supervises external shell pipelines of the form:

    ``rpicam-vid --codec yuv420 -o - | gst-launch-1.0 fdsrc ! videoparse !``
    ``videoconvert ! x264enc tune=zerolatency ! rtph264pay ! udpsink``

    The exact arguments (width, height, framerate, bitrate, host/port)
    are driven by parameters loaded from ``halow_config.yaml``. This
    mirrors the validated manual pipeline in
    ``QWACR_Project_Status/HALOW_VIDEO_STREAMING_GUIDE.md`` section 2.5.
    """

    def __init__(self) -> None:
        super().__init__("video_publisher")

        # Declare parameters
        self.declare_parameter("camera1_device", "/dev/video0")
        self.declare_parameter("camera1_name", "front")
        self.declare_parameter("camera1_index", 0)
        self.declare_parameter("camera1_width", 1280)
        self.declare_parameter("camera1_height", 720)
        self.declare_parameter("camera1_fps", 20)
        self.declare_parameter("camera1_bitrate", 3500)

        self.declare_parameter("camera2_device", "/dev/video1")
        self.declare_parameter("camera2_name", "rear")
        self.declare_parameter("camera2_index", 1)
        self.declare_parameter("camera2_width", 1280)
        self.declare_parameter("camera2_height", 720)
        self.declare_parameter("camera2_fps", 15)
        self.declare_parameter("camera2_bitrate", 3000)

        # Allow enabling/disabling the second stream explicitly so that
        # robots with only one camera do not try to start a dead pipeline.
        self.declare_parameter("enable_camera2", False)

        self.declare_parameter("compression", "h264")
        self.declare_parameter("quality", 60)

        self.declare_parameter("halow_interface", "wlan1")
        self.declare_parameter("base_station_ip", "10.20.0.100")
        self.declare_parameter("stream1_port", 5000)
        self.declare_parameter("stream2_port", 5001)

        self._pipelines: List[subprocess.Popen] = []

        self._start_pipelines()

        # Add a small timer just to keep the node alive and allow health logs.
        self.create_timer(10.0, self._health_check_timer)

        rclpy.get_default_context().on_shutdown(self._on_shutdown)

    # ------------------------------------------------------------------
    # Pipeline management
    # ------------------------------------------------------------------

    def _start_pipelines(self) -> None:
        """Start one or more rpicam-vid → GStreamer pipelines based on parameters."""

        base_ip = self.get_parameter("base_station_ip").get_parameter_value().string_value
        if not base_ip:
            self.get_logger().error("base_station_ip parameter is empty; not starting video streams")
            return

        self.get_logger().info(f"Starting HaLow video streams to {base_ip} using rpicam-vid → GStreamer")

        cam1_cmd = self._build_rpicam_gstreamer_pipeline_command(
            camera_index=self.get_parameter("camera1_index").get_parameter_value().integer_value,
            width=self.get_parameter("camera1_width").get_parameter_value().integer_value,
            height=self.get_parameter("camera1_height").get_parameter_value().integer_value,
            fps=self.get_parameter("camera1_fps").get_parameter_value().integer_value,
            bitrate=self.get_parameter("camera1_bitrate").get_parameter_value().integer_value,
            host=base_ip,
            port=self.get_parameter("stream1_port").get_parameter_value().integer_value,
        )

        cam2_cmd = None
        if self.get_parameter("enable_camera2").get_parameter_value().bool_value:
            cam2_cmd = self._build_rpicam_gstreamer_pipeline_command(
                camera_index=self.get_parameter("camera2_index").get_parameter_value().integer_value,
                width=self.get_parameter("camera2_width").get_parameter_value().integer_value,
                height=self.get_parameter("camera2_height").get_parameter_value().integer_value,
                fps=self.get_parameter("camera2_fps").get_parameter_value().integer_value,
                bitrate=self.get_parameter("camera2_bitrate").get_parameter_value().integer_value,
                host=base_ip,
                port=self.get_parameter("stream2_port").get_parameter_value().integer_value,
            )

        for name, cmd in (("camera1", cam1_cmd), ("camera2", cam2_cmd)):
            if cmd is None:
                continue

            try:
                self.get_logger().info(f"Launching {name} pipeline via bash -lc: {cmd}")
                proc = subprocess.Popen(
                    ["bash", "-lc", cmd],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid,
                )
                self._pipelines.append(proc)
            except FileNotFoundError:
                self.get_logger().error("gst-launch-1.0 not found. Is GStreamer installed?")
                break
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"Failed to start {name} pipeline: {exc}")

    def _build_rpicam_gstreamer_pipeline_command(
        self,
        *,
        camera_index: Optional[int],
        width: int,
        height: int,
        fps: int,
        bitrate: int,
        host: str,
        port: int,
    ) -> Optional[str]:
        """Build the validated rpicam-vid → GStreamer shell pipeline.

        This matches the manual command documented in the HaLow video
        guide (section 2.5), but with parameters substituted from
        halow_config.yaml.
        """

        if fps <= 0:
            fps = 15

        camera_flag = ""
        if camera_index is not None and camera_index >= 0:
            camera_flag = f"--camera {camera_index} "

        cmd = (
            "rpicam-vid -t 0 -n {camera_flag}--width {width} --height {height} --framerate {fps} "
            "--codec yuv420 -o - | "
            "gst-launch-1.0 -v fdsrc ! "
            "videoparse width={width} height={height} framerate={fps}/1 format=i420 ! "
            "videoconvert ! "
            "x264enc tune=zerolatency bitrate={bitrate} speed-preset=ultrafast ! "
            "rtph264pay config-interval=1 pt=96 ! "
            "udpsink host={host} port={port} sync=false"
        ).format(
            camera_flag=camera_flag,
            width=width,
            height=height,
            fps=fps,
            bitrate=bitrate,
            host=host,
            port=port,
        )

        return cmd

    def _health_check_timer(self) -> None:
        # Log if any child pipelines have died.
        alive = 0
        for proc in list(self._pipelines):
            if proc.poll() is None:
                alive += 1
            else:
                self.get_logger().warn("A video pipeline exited unexpectedly; return code %d" % proc.returncode)
                self._pipelines.remove(proc)
        self.get_logger().debug(f"Video pipelines alive: {alive}")

    def _on_shutdown(self) -> None:
        self.get_logger().info("Shutting down video pipelines")
        for proc in self._pipelines:
            try:
                # Terminate the whole process group so that gst-launch is cleaned up.
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except ProcessLookupError:
                continue
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Error while terminating pipeline: {exc}")

        self._pipelines.clear()


def main(args: Optional[list] = None) -> None:  # type: ignore[override]
    rclpy.init(args=args)
    node = VideoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
