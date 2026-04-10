#!/usr/bin/env python3

import math
import sys

from sensor_msgs.msg import Imu
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions


def mean_and_std(samples):
    n = len(samples)
    if n == 0:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    sums = [0.0, 0.0, 0.0]
    for x, y, z in samples:
        sums[0] += x
        sums[1] += y
        sums[2] += z
    means = [s / n for s in sums]
    var = [0.0, 0.0, 0.0]
    for x, y, z in samples:
        var[0] += (x - means[0]) ** 2
        var[1] += (y - means[1]) ** 2
        var[2] += (z - means[2]) ** 2
    std = [math.sqrt(v / n) for v in var]
    return means, std


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: analyze_imu_bias.py <bag_directory>")
        sys.exit(1)

    bag_path = sys.argv[1]
    topic = "/imu/data"

    # Jazzy's ros2 bag default is MCAP; your bag files are *.mcap, so we
    # use the "mcap" storage plugin here.
    storage_options = StorageOptions(uri=bag_path, storage_id="mcap")
    converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    lin_samples = []
    ang_samples = []

    while reader.has_next():
        this_topic, data, _ = reader.read_next()
        if this_topic != topic:
            continue

        msg = deserialize_message(data, Imu)

        lin_samples.append(
            (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )
        )
        ang_samples.append(
            (
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            )
        )

    ang_mean, ang_std = mean_and_std(ang_samples)
    lin_mean, lin_std = mean_and_std(lin_samples)

    print("Angular velocity bias (rad/s):", ang_mean)
    print("Angular velocity noise std (rad/s):", ang_std)
    print("Linear acceleration mean (m/s^2):", lin_mean)
    print("Linear acceleration noise std (m/s^2):", lin_std)
    print()
    print("Note: linear accel mean includes gravity; the axis with |mean| ~= 9.8 m/s^2 is gravity.")


if __name__ == "__main__":
    main()
