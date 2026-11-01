from collections import deque
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time

from rm_aim_sim_bridge.msg import (
    AimCommand,
    ArmorCornersArray,
    GimbalState,
    Telemetry,
)


PITCH_MIN = math.radians(-20.0)
PITCH_MAX = math.radians(30.0)
MAX_INPUT_AGE = 0.10


def add(left, right):
    return tuple(left[index] + right[index] for index in range(3))


def subtract(left, right):
    return tuple(left[index] - right[index] for index in range(3))


def scale(value, factor):
    return tuple(value[index] * factor for index in range(3))


def dot(left, right):
    return sum(left[index] * right[index] for index in range(3))


def cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def normalize(value, fallback=(0.0, 0.0, 0.0)):
    length = math.sqrt(dot(value, value))
    if length <= 1.0e-9:
        return fallback
    return scale(value, 1.0 / length)


def point_tuple(point):
    return point.x, point.y, point.z


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1.0e-9


def fit_slope(samples, accessor):
    if len(samples) < 4:
        return 0.0
    mean_time = sum(sample[0] for sample in samples) / len(samples)
    times = [sample[0] - mean_time for sample in samples]
    denominator = sum(value * value for value in times)
    if denominator <= 1.0e-12:
        return 0.0
    values = [accessor(sample[1]) for sample in samples]
    mean_value = sum(values) / len(values)
    numerator = sum(
        times[index] * (values[index] - mean_value)
        for index in range(len(values))
    )
    return numerator / denominator


def solve_three_by_three(matrix, values):
    rows = [list(matrix[row]) + [values[row]] for row in range(3)]
    for column in range(3):
        pivot = max(range(column, 3), key=lambda row: abs(rows[row][column]))
        if abs(rows[pivot][column]) <= 1.0e-12:
            return None
        rows[column], rows[pivot] = rows[pivot], rows[column]
        divisor = rows[column][column]
        rows[column] = [value / divisor for value in rows[column]]
        for row in range(3):
            if row == column:
                continue
            factor = rows[row][column]
            rows[row] = [
                rows[row][index] - factor * rows[column][index]
                for index in range(4)
            ]
    return [rows[index][3] for index in range(3)]


def fit_motion(samples, accessor):
    if len(samples) < 6:
        return 0.0, 0.0

    last_time = samples[-1][0]
    mean_time = sum(sample[0] for sample in samples) / len(samples)
    times = [sample[0] - mean_time for sample in samples]
    values = [accessor(sample[1]) for sample in samples]
    matrix = (
        (len(times), sum(times), sum(value ** 2 for value in times)),
        (
            sum(times),
            sum(value ** 2 for value in times),
            sum(value ** 3 for value in times),
        ),
        (
            sum(value ** 2 for value in times),
            sum(value ** 3 for value in times),
            sum(value ** 4 for value in times),
        ),
    )
    right_side = (
        sum(values),
        sum(times[index] * values[index] for index in range(len(times))),
        sum(times[index] ** 2 * values[index] for index in range(len(times))),
    )
    coefficients = solve_three_by_three(matrix, right_side)
    if coefficients is None:
        return 0.0, 0.0

    current_offset = last_time - mean_time
    velocity = coefficients[1] + 2.0 * coefficients[2] * current_offset
    acceleration = 2.0 * coefficients[2]
    if (
        not math.isfinite(velocity)
        or not math.isfinite(acceleration)
        or abs(velocity) > 5.0
        or abs(acceleration) > 10.0
    ):
        return 0.0, 0.0
    return velocity, acceleration


def plate_geometry(armor):
    corners = [point_tuple(point) for point in armor.corners]
    center = scale(add(add(corners[0], corners[1]), add(corners[2], corners[3])), 0.25)
    right_edge = scale(add(corners[1], corners[2]), 0.5)
    left_edge = scale(add(corners[0], corners[3]), 0.5)
    top_edge = scale(add(corners[0], corners[1]), 0.5)
    bottom_edge = scale(add(corners[2], corners[3]), 0.5)
    right_vector = normalize(subtract(right_edge, left_edge))
    up_vector = normalize(subtract(top_edge, bottom_edge))
    normal = normalize(cross(right_vector, up_vector))
    horizontal_normal = normalize((normal[0], normal[1], 0.0))
    theta = math.atan2(horizontal_normal[1], horizontal_normal[0])
    return {
        'center': center,
        'normal': normal,
        'horizontal_normal': horizontal_normal,
        'theta': theta,
    }


def estimate_target_geometry(plates):
    best = None
    for first_index in range(len(plates)):
        for second_index in range(first_index + 1, len(plates)):
            first = plates[first_index]
            second = plates[second_index]
            first_normal = first['horizontal_normal']
            second_normal = second['horizontal_normal']
            normal_pair = (
                second_normal[0] - first_normal[0],
                second_normal[1] - first_normal[1],
            )
            denominator = normal_pair[0] ** 2 + normal_pair[1] ** 2
            if denominator <= 1.0e-6:
                continue

            alignment = abs(
                first_normal[0] * second_normal[0]
                + first_normal[1] * second_normal[1]
            )
            center_delta = (
                second['center'][0] - first['center'][0],
                second['center'][1] - first['center'][1],
            )
            radius = (
                center_delta[0] * normal_pair[0]
                + center_delta[1] * normal_pair[1]
            ) / denominator
            if not 0.10 <= radius <= 0.40:
                continue

            center = (
                first['center'][0] - radius * first_normal[0],
                first['center'][1] - radius * first_normal[1],
            )
            predicted_second = (
                center[0] + radius * second_normal[0],
                center[1] + radius * second_normal[1],
            )
            residual = math.hypot(
                predicted_second[0] - second['center'][0],
                predicted_second[1] - second['center'][1],
            )
            score = alignment + residual
            if best is None or score < best[0]:
                best = (score, center, radius)

    if best is None:
        return None
    return best[1], best[2]


def solve_aim_angles(center, telemetry):
    x, y, z = center
    horizontal = math.hypot(x, y)
    speed = telemetry.speed
    gravity = telemetry.gravity
    barrel_length = telemetry.barrel_length

    if horizontal <= barrel_length or speed <= 0.0:
        return None

    yaw = math.atan2(y, x)
    elevation = math.atan2(z, horizontal)

    for _ in range(8):
        distance = horizontal - barrel_length * math.cos(elevation)
        height = z - barrel_length * math.sin(elevation)
        if distance <= 0.0:
            return None

        if gravity < 1.0e-9:
            next_elevation = math.atan2(height, distance)
        else:
            speed_squared = speed * speed
            discriminant = speed_squared * speed_squared - gravity * (
                gravity * distance * distance
                + 2.0 * height * speed_squared
            )
            if discriminant < 0.0:
                return None
            next_elevation = math.atan(
                (speed_squared - math.sqrt(discriminant))
                / (gravity * distance)
            )

        if abs(next_elevation - elevation) < 1.0e-9:
            elevation = next_elevation
            break
        elevation = next_elevation

    if not (
        math.isfinite(yaw)
        and math.isfinite(elevation)
        and PITCH_MIN <= elevation <= PITCH_MAX
    ):
        return None

    distance = horizontal - barrel_length * math.cos(elevation)
    flight_time = distance / (speed * math.cos(elevation))
    return yaw, elevation, flight_time


class AimNode(Node):
    def __init__(self):
        super().__init__('aim_node')

        self.enable_fire = bool(
            self.declare_parameter('enable_fire', True).value
        )
        self.track_samples = max(
            6,
            min(
                12,
                int(self.declare_parameter('track_samples', 7).value),
            ),
        )
        self.fire_delay_scale = float(
            self.declare_parameter('fire_delay_scale', 1.0).value
        )
        self.aim_tolerance = math.radians(
            max(
                0.05,
                float(
                    self.declare_parameter('aim_tolerance_deg', 0.35).value
                ),
            )
        )

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.latest_armors = None
        self.latest_gimbal = None
        self.latest_telemetry = None
        self.plates = []

        self.session_id = ''
        self.target_center = None
        self.target_radius = None
        self.geometry_has_pair = False
        self.center_velocity = (0.0, 0.0)
        self.center_acceleration = (0.0, 0.0)
        self.center_samples = deque(maxlen=self.track_samples)
        self.rotation_phase = None
        self.rotation_samples = deque(maxlen=8)
        self.angular_velocity = 0.0
        self.last_stamp = 0.0

        self.armors_subscription = self.create_subscription(
            ArmorCornersArray,
            '/aim_sim/armors',
            self.on_armors,
            qos,
        )
        self.gimbal_subscription = self.create_subscription(
            GimbalState,
            '/aim_sim/gimbal',
            self.on_gimbal,
            qos,
        )
        self.telemetry_subscription = self.create_subscription(
            Telemetry,
            '/aim_sim/telemetry',
            self.on_telemetry,
            qos,
        )
        self.command_publisher = self.create_publisher(
            AimCommand,
            '/aim_sim/command',
            qos,
        )
        self.timer = self.create_timer(0.02, self.on_timer)

        self.firing = False
        self.last_status_time = 0.0
        self.get_logger().info('Rotation-prediction aim node started')

    def reset_track(self):
        self.plates = []
        self.target_center = None
        self.target_radius = None
        self.geometry_has_pair = False
        self.center_velocity = (0.0, 0.0)
        self.center_acceleration = (0.0, 0.0)
        self.center_samples.clear()
        self.rotation_phase = None
        self.rotation_samples.clear()
        self.angular_velocity = 0.0
        self.last_stamp = 0.0

    def on_armors(self, msg):
        self.latest_armors = msg
        if not msg.armors:
            return

        if msg.session_id != self.session_id:
            self.session_id = msg.session_id
            self.reset_track()

        stamp = stamp_seconds(msg.header.stamp)
        if stamp < self.last_stamp:
            self.reset_track()
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp

        self.plates = [plate_geometry(armor) for armor in msg.armors]
        geometry = estimate_target_geometry(self.plates)
        if geometry is not None:
            center, radius = geometry
            if not self.geometry_has_pair:
                self.target_center = center
                self.target_radius = radius
                self.center_samples.clear()
                self.geometry_has_pair = True
            else:
                self.target_center = center
                self.target_radius = 0.75 * self.target_radius + 0.25 * radius
            self.center_samples.append((stamp, center))
            velocity_x, acceleration_x = fit_motion(
                self.center_samples,
                lambda value: value[0],
            )
            velocity_y, acceleration_y = fit_motion(
                self.center_samples,
                lambda value: value[1],
            )
            self.center_velocity = velocity_x, velocity_y
            self.center_acceleration = acceleration_x, acceleration_y
        elif not self.geometry_has_pair:
            plate_center = self.plates[0]['center']
            self.target_center = plate_center[0], plate_center[1]
            self.target_radius = 0.0
            self.center_samples.append(
                (stamp, (plate_center[0], plate_center[1]))
            )
            velocity_x, acceleration_x = fit_motion(
                self.center_samples,
                lambda value: value[0],
            )
            velocity_y, acceleration_y = fit_motion(
                self.center_samples,
                lambda value: value[1],
            )
            self.center_velocity = velocity_x, velocity_y
            self.center_acceleration = acceleration_x, acceleration_y

        sine = sum(math.sin(4.0 * plate['theta']) for plate in self.plates)
        cosine = sum(math.cos(4.0 * plate['theta']) for plate in self.plates)
        raw_phase = math.atan2(sine, cosine) * 0.25
        if self.rotation_phase is None:
            self.rotation_phase = raw_phase
        else:
            quarter_turn = math.pi * 0.5
            while raw_phase - self.rotation_phase > math.pi * 0.25:
                raw_phase -= quarter_turn
            while raw_phase - self.rotation_phase < -math.pi * 0.25:
                raw_phase += quarter_turn
            self.rotation_phase = raw_phase
        self.rotation_samples.append((stamp, self.rotation_phase))
        self.angular_velocity = fit_slope(
            self.rotation_samples,
            lambda value: value,
        )

    def on_gimbal(self, msg):
        self.latest_gimbal = msg

    def on_telemetry(self, msg):
        self.latest_telemetry = msg

    def input_age(self, stamp):
        now = self.get_clock().now()
        return max(
            0.0,
            (now - Time.from_msg(stamp)).nanoseconds * 1.0e-9,
        )

    def predict_best_plate(self, telemetry):
        if (
            self.target_center is None
            or self.target_radius is None
            or not self.plates
        ):
            return None

        observation_age = self.input_age(self.latest_armors.header.stamp)
        best = None
        for plate in self.plates:
            flight_time = math.hypot(
                self.target_center[0],
                self.target_center[1],
            ) / max(telemetry.speed, 1.0e-6)
            solved = None
            facing = 0.0

            for _ in range(6):
                horizon = (
                    observation_age
                    + self.fire_delay_scale * telemetry.delay
                    + flight_time
                )
                predicted_theta = plate['theta'] + self.angular_velocity * horizon
                predicted_center = (
                    self.target_center[0]
                    + self.center_velocity[0] * horizon
                    + 0.5 * self.center_acceleration[0] * horizon * horizon
                    + self.target_radius * math.cos(predicted_theta),
                    self.target_center[1]
                    + self.center_velocity[1] * horizon
                    + 0.5 * self.center_acceleration[1] * horizon * horizon
                    + self.target_radius * math.sin(predicted_theta),
                    plate['center'][2],
                )

                normal_horizontal_size = math.hypot(
                    plate['normal'][0],
                    plate['normal'][1],
                )
                predicted_normal = (
                    normal_horizontal_size * math.cos(predicted_theta),
                    normal_horizontal_size * math.sin(predicted_theta),
                    plate['normal'][2],
                )
                to_origin = normalize(scale(predicted_center, -1.0))
                facing = dot(predicted_normal, to_origin)
                if facing <= 0.15:
                    solved = None
                    break

                next_solved = solve_aim_angles(predicted_center, telemetry)
                if next_solved is None:
                    solved = None
                    break
                if (
                    solved is not None
                    and abs(next_solved[2] - flight_time) < 1.0e-7
                ):
                    solved = next_solved
                    break
                solved = next_solved
                flight_time = solved[2]

            if solved is None:
                continue
            score = facing
            if best is None or score > best[0]:
                best = (score, solved, predicted_center)

        return best

    def on_timer(self):
        armors_msg = self.latest_armors
        gimbal = self.latest_gimbal
        telemetry = self.latest_telemetry
        if armors_msg is None:
            return

        command = AimCommand()
        command.header = armors_msg.header
        command.session_id = armors_msg.session_id
        command.enable = False
        command.fire = False

        if armors_msg.armors and telemetry is not None:
            prediction = self.predict_best_plate(telemetry)
            if prediction is not None:
                _, solved, _ = prediction
                command.yaw, command.pitch, _ = solved
                command.enable = True

                inputs_fresh = (
                    gimbal is not None
                    and self.input_age(armors_msg.header.stamp) <= MAX_INPUT_AGE
                    and self.input_age(gimbal.header.stamp) <= MAX_INPUT_AGE
                    and self.input_age(telemetry.header.stamp) <= MAX_INPUT_AGE
                )
                if gimbal is not None:
                    yaw_error = abs(
                        math.atan2(
                            math.sin(gimbal.yaw - command.yaw),
                            math.cos(gimbal.yaw - command.yaw),
                        )
                    )
                    pitch_error = abs(gimbal.pitch - command.pitch)
                else:
                    yaw_error = math.inf
                    pitch_error = math.inf
                fire_allowed = (
                    inputs_fresh
                    and yaw_error <= self.aim_tolerance
                    and pitch_error <= self.aim_tolerance
                )
                command.fire = self.enable_fire and fire_allowed

        if command.fire != self.firing:
            self.firing = command.fire
            gate = 'OPEN' if self.firing else 'CLOSED'
            self.get_logger().info(f'Fire gate: {gate}')

        now = time.monotonic()
        if now - self.last_status_time >= 0.5:
            self.last_status_time = now
            actual_angles = (
                f'({gimbal.yaw:.3f}, {gimbal.pitch:.3f})'
                if gimbal is not None
                else '(unavailable)'
            )
            center_text = (
                f'({self.target_center[0]:.3f}, {self.target_center[1]:.3f})'
                if self.target_center is not None
                else '(unavailable)'
            )
            self.get_logger().info(
                f'armors={len(armors_msg.armors)} center={center_text} '
                f'radius={self.target_radius if self.target_radius is not None else -1.0:.3f} '
                f'center_velocity=({self.center_velocity[0]:.3f}, {self.center_velocity[1]:.3f}) '
                f'center_acceleration=({self.center_acceleration[0]:.3f}, '
                f'{self.center_acceleration[1]:.3f}) '
                f'omega={self.angular_velocity:.3f} '
                f'command=({command.yaw:.3f}, {command.pitch:.3f}) '
                f'actual={actual_angles} fire={command.fire}'
            )

        self.command_publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = AimNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
