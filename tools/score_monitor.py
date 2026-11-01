import csv
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from rm_aim_sim_bridge.msg import ArmorCornersArray
from rosgraph_msgs.msg import Clock


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1.0e-9


class ScoreMonitor(Node):
    def __init__(self):
        super().__init__("score_monitor")
        self.scenario = str(
            self.declare_parameter("scenario", "unknown").value
        )
        output_name = str(
            self.declare_parameter("output_csv", "").value
        )
        self.output_path = (
            Path(output_name).expanduser()
            if output_name
            else Path.home() / "aim_kill_times.csv"
        )

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.clock_subscription = self.create_subscription(
            Clock,
            "/clock",
            self.on_clock,
            qos,
        )
        self.armors_subscription = self.create_subscription(
            ArmorCornersArray,
            "/aim_sim/armors",
            self.on_armors,
            qos,
        )

        self.sim_time = 0.0
        self.session_id = ""
        self.session_start = None
        self.had_target = False
        self.first_empty_stamp = None
        self.reported = False
        self.get_logger().info(
            f"Score monitor started for scenario={self.scenario}; "
            f"results={self.output_path}"
        )

    def on_clock(self, msg):
        self.sim_time = stamp_seconds(msg.clock)

    def on_armors(self, msg):
        stamp = stamp_seconds(msg.header.stamp)
        self.sim_time = max(self.sim_time, stamp)

        if msg.session_id != self.session_id:
            self.session_id = msg.session_id
            self.session_start = stamp
            self.had_target = False
            self.first_empty_stamp = None
            self.reported = False
            self.get_logger().info(
                f"New session={self.session_id}, start={self.session_start:.3f} s"
            )

        if msg.armors:
            self.had_target = True
            self.first_empty_stamp = None
            self.reported = False
            return

        if not self.had_target or self.reported:
            return
        if self.first_empty_stamp is None:
            self.first_empty_stamp = stamp
            return

        hidden_duration = stamp - self.first_empty_stamp
        if hidden_duration < 0.30:
            return

        kill_time = max(0.0, self.first_empty_stamp - self.session_start)
        self.reported = True
        self.get_logger().info(
            f"TARGET_LOST scenario={self.scenario} "
            f"estimated_kill_time={kill_time:.3f} s"
        )
        self.write_result(kill_time)

    def write_result(self, kill_time):
        new_file = not self.output_path.exists()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("a", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            if new_file:
                writer.writerow(
                    ["recorded_at", "scenario", "session_id", "estimated_kill_time_s"]
                )
            writer.writerow(
                [
                    datetime.now().isoformat(timespec="seconds"),
                    self.scenario,
                    self.session_id,
                    f"{kill_time:.3f}",
                ]
            )


def main(args=None):
    rclpy.init(args=args)
    node = ScoreMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
