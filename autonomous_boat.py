import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan, Imu, NavSatFix
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from mechaship_interfaces.msg import RgbwLedColor, ToneTopic
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import math
import numpy as np
from scipy.signal import medfilt
from sklearn.cluster import DBSCAN


class AutonomousBoat(Node):
    RGBW_BRIGHTNESS = 30
    TONE_HZ = 2000  # 부저음 주파수
    TONE_DURATION_MS = 100  # 부저음 지속시간 (밀리초)

    def __init__(self):
        super().__init__("autonomous_boat")

        # QoS 설정: 안정적인 데이터 통신을 위해 설정
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # QoS 호환성 보장
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  # 메시지 버퍼 크기
        )

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self.imu_callback, qos_profile
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, qos_profile
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, qos_profile
        )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/visualization_marker_array", 10
        )
        self.thruster_publisher = self.create_publisher(
            Float32, "/actuator/thruster/percentage", 10
        )
        self.key_publisher = self.create_publisher(Float32, "/actuator/key/degree", 10)
        self.rgbw_publisher = self.create_publisher(
            RgbwLedColor, "actuator/rgbwled/color", 10
        )
        self.tone_publisher = self.create_publisher(ToneTopic, "actuator/tone/play", 10)

        # 메시지 초기화
        self.rgbw_msg = RgbwLedColor()
        self.tone_msg = ToneTopic()
        self.tone_msg.hz = self.TONE_HZ
        self.tone_msg.duration_ms = self.TONE_DURATION_MS
        # Parameters
        self.safe_angle_min = 20
        self.min_safe_angle = 5
        self.lidar_range = 2.0
        self.boat_width = 0.4
        self.min_obstacle_distance = 0.13  # 장애물 최소 크기 30cm
        self.fixed_obstacle_distance = 0.6  # 장애물 간 거리 60cm 고정
        self.obstacles = []
        self.current_heading = 0.0
        self.corrected_heading = 0.0  # 보정된 헤딩 값 추가
        self.mag_declination = 5.0  # 자기 편차 추가 (예: 5도)
        self.last_log_time = 0.0
        self.heading_log_count = 0  # 헤딩 로그 빈도를 제어

        # Speed control
        self.max_motor_speed = 0.75  # 최대 모터 속도 (75%)
        self.current_speed = 0.0  # 초기 모터 속도 50%
        self.base_speed = 0.5  # 웨이포인트 따라갈 때 기본 속도 (50%)
        self.target_speed = self.base_speed

        # Key control
        self.key_degree = 90.0  # 초기 키 각도

        # GPS and waypoints
        self.waypoints = [(35.23159117, 129.08252817), (35.23160083, 129.08249467),  (35.23160083, 129.08249467),   (35.23167017, 129.08252333), (35.23168067, 129.08248600)]
        self.current_waypoint_index = 0

        # LED control
        # 기본 모터 출력 설정
        self.set_default_motor_speed()

    def set_rgb(self, red: int, green: int, blue: int, white: int = 0):
        """RGBW 값을 설정하고 메시지를 퍼블리시."""
        self.rgbw_msg.red = red
        self.rgbw_msg.green = green
        self.rgbw_msg.blue = blue
        self.rgbw_msg.white = white

        self.rgbw_publisher.publish(self.rgbw_msg)
        self.get_logger().info("출발 전 노란색 LED 점등 완료.")

        self.tone_publisher.publish(self.tone_msg)

    def turn_off(self):
        """RGBW LED를 끄는 함수."""
        if self.context.ok():  # 노드가 활성 상태인지 확인
            self.set_rgb(0, 0, 0, 0)  # LED 끄기
            self.get_logger().info("LED가 꺼졌습니다.")
        else:
            self.get_logger().warning("노드가 이미 종료되어 LED를 끌 수 없습니다.")

    def set_default_motor_speed(self):
        thruster_msg = Float32()
        thruster_msg.data = 50.0  # 기본 출력값 50%
        self.thruster_publisher.publish(thruster_msg)
        self.get_logger().info("기본 모터 출력이 50%로 설정되었습니다.")

        # 상태를 1로 변경하여 다음 번에 LED 2를 켜도록 설정

    def imu_callback(self, msg):
        # IMU 데이터를 이용해 현재 헤딩값 업데이트
        w, x, y, z = (
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        )
        yaw = self.quaternion_to_yaw(w, x, y, z)
        self.current_heading = yaw

        # 자기 편차를 적용한 보정 헤딩 값 계산
        self.corrected_heading = (self.current_heading + self.mag_declination) % 360

        # 로그 출력은 3번으로 제한
        if self.heading_log_count < 3:
            self.get_logger().info(
                f"현재 헤딩 (Yaw): {self.current_heading:.2f}°, 보정된 헤딩: {self.corrected_heading:.2f}°"
            )
            self.heading_log_count += 1

    def quaternion_to_yaw(self, w, x, y, z):
        # 쿼터니언 -> 오일러 각도 변환 (Yaw)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.degrees(math.atan2(siny_cosp, cosy_cosp))

    def gps_callback(self, msg):
        # GPS 데이터를 받아 현재 위치 업데이트
        latitude = msg.latitude
        longitude = msg.longitude
        fix_status = msg.status.status

        self.current_position = (latitude, longitude)
        self.check_waypoint_reached()

    def check_waypoint_reached(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info("모든 웨이포인트에 도달했습니다.")
            return

        waypoint = self.waypoints[self.current_waypoint_index]

        # 거리와 베어링 계산
        distance, bearing = self.calculate_distance_and_bearing(
            self.current_position, waypoint
        )

        # 로그 출력
        self.get_logger().info(
            f"웨이포인트까지 거리: {distance:.2f}m, 베어링: {bearing:.2f}°"
        )

        if distance <= 0.5:  # 웨이포인트에 도달한 경우
            self.get_logger().info(
                f"웨이포인트 {self.current_waypoint_index + 1}에 도달했습니다."
            )
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.navigate_to_next_waypoint()

    def calculate_distance(self, position1, position2):
        # 두 위치 간 거리 계산 (단위: 미터)
        lat1, lon1 = position1
        lat2, lon2 = position2
        # 단순 유클리드 거리 계산 (위도/경도 차이를 단위 거리로 간주)
        return math.sqrt((lat1 - lat2) ** 2 + (lon1 - lon2) ** 2)

    def calculate_distance_and_bearing(self, current_position, waypoint):
        lat1, lon1 = current_position
        lat2, lon2 = waypoint

        # 거리 계산 (Haversine formula)
        R = 6371000  # 지구 반지름 (미터)
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = (
            math.sin(delta_phi / 2) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        # 베어링 계산
        y = math.sin(delta_lambda) * math.cos(phi2)
        x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(
            phi2
        ) * math.cos(delta_lambda)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

        return distance, bearing

    def navigate_to_angle(self, target_angle):
        # 주어진 각도로 이동
        angular_diff = (target_angle - self.corrected_heading + 180) % 360 - 180

        # 키 각도 조정
        if 85 < angular_diff <= 90:  # 최대 우측
            self.key_degree = 120.0
        elif 75 < angular_diff <= 85:
            self.key_degree = 118.0
        elif 65 < angular_diff <= 75:
            self.key_degree = 116.0
        elif 55 < angular_diff <= 65:
            self.key_degree = 114.0
        elif 45 < angular_diff <= 55:
            self.key_degree = 112.0
        elif 35 < angular_diff <= 45:
            self.key_degree = 110.0
        elif 25 < angular_diff <= 35:
            self.key_degree = 108.0
        elif 15 < angular_diff <= 25:
            self.key_degree = 105.0
        elif 5 < angular_diff <= 15:
            self.key_degree = 95.0
        elif -5 <= angular_diff <= 5:  # 중앙
            self.key_degree = 90.0
        elif -15 <= angular_diff < -5:
            self.key_degree = 85.0
        elif -25 <= angular_diff < -15:
            self.key_degree = 80.0
        elif -35 <= angular_diff < -25:
            self.key_degree = 75.0
        elif -45 <= angular_diff < -35:
            self.key_degree = 70.0
        elif -55 <= angular_diff < -45:
            self.key_degree = 68.0
        elif -65 <= angular_diff < -55:
            self.key_degree = 66.0
        elif -75 <= angular_diff < -65:
            self.key_degree = 64.0
        elif -85 <= angular_diff < -75:
            self.key_degree = 62.0
        elif -90 <= angular_diff <= -85:  # 최대 좌측
            self.key_degree = 60.0
        else:  # 기타 값은 중앙으로 설정
            self.key_degree = 90.0

        # 키 각도를 퍼블리시
        key_msg = Float32()
        key_msg.data = self.key_degree
        self.key_publisher.publish(key_msg)

        self.get_logger().info(f"키 명령 발행: Key Degree={self.key_degree}°")

        # 속도 서서히 조정
        previous_speed = self.current_speed
        if self.current_speed < self.target_speed:
            self.current_speed += 0.10  # 서서히 증가
        elif self.current_speed > self.target_speed:
            self.current_speed -= 0.1  # 서서히 감소

        self.current_speed = min(self.current_speed, self.max_motor_speed)

        # Thruster 퍼센티지 발행
        thruster_msg = Float32()
        thruster_msg.data = self.current_speed * 100  # 퍼센티지로 변환
        self.thruster_publisher.publish(thruster_msg)

        # 속도 변화 로그 출력
        if self.current_speed > previous_speed:
            self.get_logger().info("속도가 올라갑니다.")
        elif self.current_speed < previous_speed:
            self.get_logger().info("속도가 내려갑니다.")

        self.get_logger().info(
            f"모터 명령 발행: Thruster 퍼센티지={thruster_msg.data:.2f}%"
        )

    def lidar_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_log_time < 1.0:  # 1초에 한 번만 로그 출력
            return
        self.last_log_time = current_time

        # 라이다 데이터를 장애물 좌표로 변환
        lidar_ranges = np.array(msg.ranges)
        angle_increment = msg.angle_increment
        lidar_angles = np.array([i * angle_increment for i in range(len(lidar_ranges))])

        lidar_ranges = np.clip(lidar_ranges, 0.2, self.lidar_range)
        lidar_ranges = self.apply_median_filter(lidar_ranges, kernel_size=5)

        valid_indices = (lidar_ranges > 0.3) & (
            lidar_ranges < self.lidar_range
        )  # 장애물 최소 크기 적용
        lidar_ranges = lidar_ranges[valid_indices]
        lidar_angles = lidar_angles[valid_indices]

        if len(lidar_ranges) == 0:
            self.obstacles = []
            self.get_logger().info("장애물이 감지되지 않았습니다.")
            return

        cartesian_coords = self.lidar_to_cartesian(lidar_ranges, lidar_angles)
        clusters = self.cluster_with_dbscan(cartesian_coords, eps=0.15, min_samples=3)

        self.obstacles = []
        for cluster in clusters:
            x, y = np.mean(cluster, axis=0)
            self.obstacles.append((x, y))

        self.calculate_path()

    def apply_median_filter(self, data, kernel_size=5):
        return medfilt(data, kernel_size=kernel_size)

    def lidar_to_cartesian(self, ranges, angles):
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        return np.vstack((x, y)).T

    def cluster_with_dbscan(self, cartesian_coords, eps=0.15, min_samples=3):
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        labels = dbscan.fit_predict(cartesian_coords)

        clusters = []
        for label in np.unique(labels):
            if label == -1:
                continue
            clusters.append(cartesian_coords[labels == label])
        return clusters

    def calculate_path(self):
        if len(self.obstacles) == 1:
            self.avoid_single_obstacle()
        elif len(self.obstacles) > 1:
            self.avoid_obstacles()

    def avoid_single_obstacle(self):
        obstacle_x, obstacle_y = self.obstacles[0]
        obstacle_angle_left = math.degrees(math.atan2(obstacle_y, obstacle_x)) - 10
        obstacle_angle_right = math.degrees(math.atan2(obstacle_y, obstacle_x)) + 10

        # 경로 1과 경로 2 설정
        path1_start = -90
        path1_end = obstacle_angle_left
        path1_center = (path1_start + path1_end) / 2

        path2_start = obstacle_angle_right
        path2_end = 90
        path2_center = (path2_start + path2_end) / 2

        # 웨이포인트와의 각도 계산
        waypoint = (
            self.waypoints[self.current_waypoint_index]
            if self.current_waypoint_index < len(self.waypoints)
            else (0, 0)
        )
        dx = waypoint[0] - self.current_position[0]
        dy = waypoint[1] - self.current_position[1]
        angle_to_waypoint = math.degrees(math.atan2(dy, dx))

        # 경로 간 각도 차이 비교
        diff_path1 = abs(path1_center - angle_to_waypoint)
        diff_path2 = abs(path2_center - angle_to_waypoint)

        if diff_path1 < diff_path2:
            chosen_path = path1_center
        elif diff_path1 > diff_path2:
            chosen_path = path2_center
        else:
            # 경로1과 경로2가 같은 경우 목표 웨이포인트 방향 비교
            waypoint_direction = (
                1 if angle_to_waypoint >= 0 else -1
            )  # 웨이포인트가 오른쪽(+)인지 왼쪽(-)인지
            path1_direction = 1 if path1_center >= 0 else -1

            if path1_direction == waypoint_direction:
                chosen_path = path1_center
            else:
                chosen_path = path2_center

        self.get_logger().info(f"장애물 회피: 경로 선택 - 중심 각도 {chosen_path:.2f}°")
        self.adjust_speed(single_obstacle=True)
        self.navigate_to_angle(chosen_path)

    def avoid_obstacles(self):
        # 장애물 간 각도와 거리 기반 회피 경로 설정
        self.adjust_speed(single_obstacle=False)
        obstacle_angles = [math.degrees(math.atan2(y, x)) for x, y in self.obstacles]
        obstacle_angles.sort()

        angle_weights = []
        for i in range(len(obstacle_angles) - 1):
            angle_diff = obstacle_angles[i + 1] - obstacle_angles[i]
            safety_angle = math.degrees(
                math.atan(self.fixed_obstacle_distance / self.lidar_range)
            )

            if angle_diff > safety_angle:
                weight_a = (2 + (angle_diff - safety_angle) // 5)

                # 소각도 변침 기반 가중치
                mid_angle = (obstacle_angles[i] + obstacle_angles[i + 1]) / 2
                turn_angle = abs(self.current_heading - mid_angle)
                weight_b = 1 / (1 + turn_angle)

                combined_weight = weight_a + weight_b
                angle_weights.append((combined_weight, mid_angle, i))
                self.get_logger().info(
                    f"장애물 {i}: 가중치 A: {weight_a:.2f}, 가중치 B: {weight_b:.2f}, 결합 가중치: {combined_weight:.2f}"
                )

        if angle_weights:
            best_path = max(angle_weights, key=lambda x: x[0])
            self.get_logger().info(
                f"최적 경로 선택: 장애물 {best_path[2]}와 장애물 {best_path[2] + 1} 사이의 중간 각도 {best_path[1]:.2f}°"
            )
            self.navigate_to_angle(best_path[1])
        else:
            self.get_logger().info("안전한 경로를 찾을 수 없습니다.")

    def adjust_speed(self, single_obstacle):
        if single_obstacle:
            self.targesst_speed = self.base_speed * 0.9  # 속도를 30% 줄임
        else:
            self.target_speed = self.base_speed * 0.6 # 속도를 50%로 감소

    def navigate_to_next_waypoint(self):
        waypoint = self.waypoints[self.current_waypoint_index]
        dx = waypoint[0] - self.current_position[0]
        dy = waypoint[1] - self.current_position[1]
        target_angle = math.degrees(math.atan2(dy, dx))
        self.navigate_to_angle(target_angle)

    def tkoff(self):
        """모터를 정지 상태로 설정"""
        if self.context.ok():  # 노드가 활성 상태인지 확인
            # 스러스터(모터) 값을 0으로 설정
            thruster_msg = Float32()
            thruster_msg.data = 0.0
            self.thruster_publisher.publish(thruster_msg)

             # 키 각도(조향)를 중립으로 설정
            key_msg = Float32()
            key_msg.data = 90.0  # 90도는 중립 상태로 가정
            self.key_publisher.publish(key_msg)

            self.get_logger().info("모터가 정지 상태로 설정되었습니다.")
        else:
            self.get_logger().warning("노드가 이미 종료되어 모터를 정지할 수 없습니다.")


def main(args=None):
    rclpy.init(args=args)
    boat = AutonomousBoat()
    try:
        rclpy.spin(boat)
        boat.set_rgb(boat.RGBW_BRIGHTNESS, boat.RGBW_BRIGHTNESS, 0)

    except KeyboardInterrupt:
        pass

    finally:
        if boat.context.ok():
            boat.turn_off()
            boat.tkoff()
        boat.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
